#include "flasher/UDSReader.hpp"

#include <common/CommonData.hpp>
#include <common/Util.hpp>
#include <common/protocols/UDSProtocolCommonSteps.hpp>
#include <common/protocols/UDSRequest.hpp>

#include <easylogging++.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace flasher {

namespace {

void setFailure(const std::string& message, const std::function<void(const std::string&)>& errorUpdater)
{
    LOG(ERROR) << message;
    errorUpdater(message);
    throw std::runtime_error(message);
}

bool hasSecurityPin(const std::array<uint8_t, 5>& pin)
{
    for (const auto byte : pin) {
        if (byte != 0) {
            return true;
        }
    }
    return false;
}

}

UDSReader::UDSReader(j2534::J2534& j2534, FlasherParameters&& flasherParameters,
                     UDSReaderParameters&& udsReaderParameters, std::vector<uint8_t>& output)
    : FlasherBase{ j2534, std::move(flasherParameters) }
    , _udsReaderParameters{ std::move(udsReaderParameters) }
    , _output{ output }
{
}

UDSReader::~UDSReader()
{
}

bool commitUploadResult(const common::UploadReadResult& upload, std::vector<uint8_t>& output)
{
    // Hand the payload out unconditionally and before any failure decision. The integrity
    // verdict never flips success: only a real read failure (NRC, timeout, undershoot, size
    // mismatch) makes this return false. A dump whose bytes arrived must never be dropped.
    output = upload.payload;
    return upload.success;
}

std::vector<std::unique_ptr<j2534::J2534Channel>> UDSReader::openChannels()
{
    LOG(INFO) << "UDSReader opening target ECU channel only, ecu=0x" << std::hex
        << getFlasherParameters().ecuId;
    std::vector<std::unique_ptr<j2534::J2534Channel>> channels;
    auto channel = openChannelForEcu(getFlasherParameters().ecuId);
    if (!channel) {
        throw std::runtime_error("Failed to open UDS target ECU channel");
    }
    channels.emplace_back(std::move(channel));
    LOG(INFO) << "UDSReader target ECU channel opened";
    return channels;
}

void UDSReader::startImpl(std::vector<std::unique_ptr<j2534::J2534Channel>>& channels)
{
    LOG(INFO) << "UDSReader startImpl enter, channels=" << channels.size()
              << " start=0x" << std::hex << _udsReaderParameters.startAddress
              << " size=0x" << _udsReaderParameters.dataSize;
    const auto ecuInfo{ common::getEcuInfoByEcuId(getFlasherParameters().carPlatform,
        getFlasherParameters().ecuId) };
    const uint32_t canId = std::get<1>(ecuInfo).canId;
    auto errorUpdater = [this](const std::string& error) {
        setLastError(error);
    };

    try {
        std::unique_ptr<common::VBF> bootloader;
        if (!_udsReaderParameters.attachRunningSbl) {
            bootloader = std::make_unique<common::VBF>(getFlasherParameters().sblProvider->getSBL(
                getFlasherParameters().carPlatform, getFlasherParameters().ecuId, getFlasherParameters().additionalData));
        }
        setMaximumProgress((_udsReaderParameters.attachRunningSbl ? 0u : FlasherBase::getProgressFromVBF(*bootloader))
            + _udsReaderParameters.dataSize);

        if (_udsReaderParameters.attachRunningSbl) {
            setCurrentState(FlasherState::FallAsleep);
            LOG(INFO) << "Attaching to already running UDS SBL, skipping programming-session broadcast/load/start";
        }
        else if (_udsReaderParameters.skipFallAsleep) {
            setCurrentState(FlasherState::FallAsleep);
            LOG(INFO) << "Programming-session broadcast skipped, vehicle programming mode was prepared by CEM";
        }
        else {
            setCurrentState(FlasherState::FallAsleep);
            // Bench prelude: raise every module with the suppressed 10 82 form, then confirm with
            // 10 02 that at least one module actually answered 50 02.
            if (!common::UDSProtocolCommonSteps::broadcastProgrammingSessionPrelude(channels)) {
                setFailure("No module confirmed programming session", errorUpdater);
            }
        }

        auto& channel{ common::getChannelByEcuId(getFlasherParameters().carPlatform,
            getFlasherParameters().ecuId, channels) };

        if (_udsReaderParameters.attachRunningSbl) {
            // A resident/read SBL usually does not implement 0x27, and attaching means it is
            // already up. Only re-authorize when a PIN was actually given and --no-sbl-auth
            // was not set; otherwise a doomed 27 01 just fails and triggers the ECUReset cleanup.
            if (_udsReaderParameters.noSblAuth || !hasSecurityPin(_udsReaderParameters.pin)) {
                LOG(INFO) << "Skipping running-SBL SecurityAccess ("
                          << (_udsReaderParameters.noSblAuth ? "--no-sbl-auth" : "no PIN provided")
                          << "), going straight to upload";
            }
            else {
                setCurrentState(FlasherState::Authorize);
                if (!common::UDSProtocolCommonSteps::authorize(channel, canId, _udsReaderParameters.pin)) {
                    setFailure("Running SBL authorization failed", errorUpdater);
                }
            }
        }
        else {
            common::UDSProtocolCommonSteps::keepAlive(channel);

            setCurrentState(FlasherState::Authorize);
            try {
                common::UDSRequest programmingSession{ canId, { 0x10, 0x02 } };
                programmingSession.process(channel, { 0x02 }, 1, 1000);
            }
            catch (const std::exception& ex) {
                LOG(WARNING) << "Programming session request before authorize failed: " << ex.what();
            }
            if (!common::UDSProtocolCommonSteps::authorize(channel, canId, _udsReaderParameters.pin)) {
                setFailure("Authorization failed", errorUpdater);
            }

            setCurrentState(FlasherState::LoadBootloader);
            if (bootloader->chunks.empty() || !common::UDSProtocolCommonSteps::transferData(channel, canId, *bootloader,
                                                                                           [this](size_t progress) {
                                                                                               incCurrentProgress(progress);
                                                                                           })) {
                setFailure("Bootloader loading failed", errorUpdater);
            }

            setCurrentState(FlasherState::StartBootloader);
            if (!common::UDSProtocolCommonSteps::startRoutine(channel, canId, bootloader->header.call)) {
                setFailure("Bootloader starting failed", errorUpdater);
            }
            if (!_udsReaderParameters.noSblAuth
                && !common::UDSProtocolCommonSteps::authorize(channel, canId, _udsReaderParameters.pin)) {
                setFailure("SBL post-start authorization failed", errorUpdater);
            }
        }

        setCurrentState(FlasherState::ReadFlash);
        const auto upload = common::UDSProtocolCommonSteps::readDataByUpload(
            channel, canId, _udsReaderParameters.startAddress, _udsReaderParameters.dataSize,
            [this](size_t progress) {
                incCurrentProgress(progress);
            });
        _lastUploadResult = upload;
        LOG(INFO) << "Read integrity: success=" << upload.success
                  << " status=" << common::integrityStatusName(upload.integrityStatus)
                  << " imageCrc=0x" << std::hex << upload.computedImageCrc
                  << " sdaWireCrc=0x" << upload.computedSdaWireCrc;
        if (!commitUploadResult(upload, _output)) {
            setFailure("Flash reading failed", errorUpdater);
        }

        setCurrentState(FlasherState::WakeUp);
        common::UDSProtocolCommonSteps::broadcastEcuReset(channels);
        setCurrentState(FlasherState::Done);
    }
    catch (const std::exception& ex) {
        if (getLastError().empty()) {
            setLastError(ex.what());
        }
        LOG(WARNING) << "UDSReader failed after SBL workflow";
        if (getCurrentState() != FlasherState::WakeUp) {
            setCurrentState(FlasherState::WakeUp);
            common::UDSProtocolCommonSteps::broadcastEcuReset(channels);
        }
        setCurrentState(FlasherState::Error);
        throw;
    }
    catch (...) {
        if (getLastError().empty()) {
            setLastError("Unknown UDS reading error");
        }
        LOG(WARNING) << "UDSReader failed after SBL workflow";
        if (getCurrentState() != FlasherState::WakeUp) {
            setCurrentState(FlasherState::WakeUp);
            common::UDSProtocolCommonSteps::broadcastEcuReset(channels);
        }
        setCurrentState(FlasherState::Error);
        throw;
    }
}

} // namespace flasher
