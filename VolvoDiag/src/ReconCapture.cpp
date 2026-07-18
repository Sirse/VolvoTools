#include "ReconCapture.hpp"

#include "DiagContext.hpp"
#include "OutputFormat.hpp"
#include "SoftIsoTp.hpp"

#include <common/Util.hpp>
#include <j2534/J2534.hpp>
#include <j2534/J2534Channel.hpp>
#include <easylogging++.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <iomanip>
#include <sstream>
#ifdef _WIN32
#include <windows.h>
#include <bcrypt.h>
#endif

namespace volvodiag {

namespace {

std::vector<uint8_t> payload(const PASSTHRU_MSG& msg)
{
    if (msg.DataSize <= 4) return {};
    return {msg.Data + 4, msg.Data + msg.DataSize};
}

std::unique_ptr<j2534::J2534Channel> openCanRawChannel(j2534::J2534& j2534,
                                                       const common::BusConfiguration& bus,
                                                       std::optional<uint32_t> baudrateOverride)
{
    const auto baudrate = baudrateOverride.value_or(bus.baudrate);
    const auto flags = bus.canIdBitSize == 29 ? CAN_29BIT_ID : 0;
    auto channel = std::make_unique<j2534::J2534Channel>(j2534, CAN, flags, baudrate, flags);
    std::vector<SCONFIG> config(3);
    config[0] = {DATA_RATE, static_cast<unsigned long>(baudrate)};
    config[1] = {LOOPBACK, 0};
    config[2] = {BIT_SAMPLE_POINT, baudrate == 500000 ? 80u : 68u};
    if (channel->setConfig(config) != STATUS_NOERROR)
        throw std::runtime_error("Failed to configure raw CAN protocol");
    auto mask = common::makePassThruMsg(CAN, flags, {0, 0, 0, 0});
    auto pattern = common::makePassThruMsg(CAN, flags, {0, 0, 0, 0});
    unsigned long filterId = 0;
    if (channel->startMsgFilter(PASS_FILTER, &mask, &pattern, nullptr, filterId) != STATUS_NOERROR)
        throw std::runtime_error("Failed to start raw CAN pass-all filter");
    LOG(INFO) << "Opened recon raw CAN channel protocol=CAN baudrate=" << baudrate;
    return channel;
}

std::unique_ptr<j2534::J2534Channel> openReconRawChannel(j2534::J2534& j2534,
                                                          const common::BusConfiguration& bus,
                                                          const RunOptions& options)
{
    if (options.reconRawProtocol == "can-ps")
        return common::openRawCanChannel(j2534, bus, options.baudrateOverride);
    try {
        return openCanRawChannel(j2534, bus, options.baudrateOverride);
    } catch (const std::exception& ex) {
        if (options.reconRawProtocol == "can") throw;
        LOG(WARNING) << "Recon CAN raw protocol failed, trying CAN_PS: " << ex.what();
        return common::openRawCanChannel(j2534, bus, options.baudrateOverride);
    }
}

std::vector<PASSTHRU_MSG> readBatch(const j2534::J2534Channel& channel, unsigned long timeout)
{
    std::vector<PASSTHRU_MSG> messages(32);
    const auto status = channel.readMsgs(messages, timeout);
    if (messages.empty() && status != STATUS_NOERROR && status != ERR_TIMEOUT
        && status != ERR_BUFFER_EMPTY) {
        throw std::runtime_error("Failed to read recon CAN frames: "
            + common::j2534StatusToString(status));
    }
    return messages;
}

void expectPrefix(const std::vector<uint8_t>& actual, const std::vector<uint8_t>& expected,
                  const char* what)
{
    if (actual.size() < expected.size()
        || !std::equal(expected.begin(), expected.end(), actual.begin())) {
        throw std::runtime_error(std::string(what) + " mismatch: got "
            + common::toHexString(actual) + ", expected " + common::toHexString(expected));
    }
}

std::string sha256Hex(const std::vector<uint8_t>& data)
{
#ifdef _WIN32
    BCRYPT_ALG_HANDLE algorithm = nullptr;
    BCRYPT_HASH_HANDLE hash = nullptr;
    DWORD objectLength = 0;
    DWORD resultLength = 0;
    if (BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, 0) != 0
        || BCryptGetProperty(algorithm, BCRYPT_OBJECT_LENGTH,
            reinterpret_cast<PUCHAR>(&objectLength), sizeof(objectLength), &resultLength, 0) != 0) {
        if (algorithm) BCryptCloseAlgorithmProvider(algorithm, 0);
        throw std::runtime_error("Unable to initialize SHA-256");
    }
    std::vector<uint8_t> object(objectLength), digest(32);
    if (BCryptCreateHash(algorithm, &hash, object.data(), objectLength, nullptr, 0, 0) != 0
        || BCryptHashData(hash, const_cast<PUCHAR>(data.data()), static_cast<ULONG>(data.size()), 0) != 0
        || BCryptFinishHash(hash, digest.data(), static_cast<ULONG>(digest.size()), 0) != 0) {
        if (hash) BCryptDestroyHash(hash);
        BCryptCloseAlgorithmProvider(algorithm, 0);
        throw std::runtime_error("Unable to calculate SHA-256");
    }
    BCryptDestroyHash(hash);
    BCryptCloseAlgorithmProvider(algorithm, 0);
    std::ostringstream result;
    result << std::hex << std::setfill('0');
    for (const auto byte : digest) result << std::setw(2) << static_cast<unsigned>(byte);
    return result.str();
#else
    throw std::runtime_error("SHA-256 verification is only implemented on Windows");
#endif
}

void writeManifest(const RunOptions& options, const std::string& verdict,
                   const ReconImage& image, size_t frameCount, uint32_t baudrate,
                   const std::string& stubSha256 = {})
{
    if (options.reconManifestPath.empty()) return;
    const auto parent = std::filesystem::path(options.reconManifestPath).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
    std::ofstream out(options.reconManifestPath);
    if (!out) throw std::runtime_error("Failed to open recon manifest: " + options.reconManifestPath);
    out << "{\n"
        << "  \"can_id\": \"" << hexNumber(options.reconCanId, 3) << "\",\n"
        << "  \"baudrate\": " << baudrate << ",\n"
        << "  \"routine\": \"" << hexNumber(options.reconRoutineId, 4) << "\",\n"
        << "  \"address\": \"" << hexNumber(image.address, 4) << "\",\n"
        << "  \"frames\": " << frameCount << ",\n"
        << "  \"image\": \"" << common::toHexString(image.bytes) << "\",\n"
        << "  \"stub_sha256\": \"" << stubSha256 << "\",\n"
        << "  \"verdict\": \"" << verdict << "\"\n}\n";
}

struct ReconManifestGuard {
    const RunOptions& options;
    uint32_t baudrate{0};
    std::string verdict{"failed"};
    ReconImage image;
    size_t frameCount{0};
    std::string stubSha256;
    ~ReconManifestGuard() noexcept {
        try { writeManifest(options, verdict, image, frameCount, baudrate, stubSha256); }
        catch (...) {}
    }
};

} // namespace

ReconImage reconstructReconImage(const std::vector<ReconFrame>& frames,
                                 uint32_t markerId,
                                 const std::vector<uint8_t>& signature,
                                 size_t frameCount)
{
    if (signature.size() > 8) throw std::runtime_error("Recon marker must fit in one CAN frame");
    std::map<uint8_t, ReconFrame> bySequence;
    for (const auto& frame : frames) {
        if (frame.canId != markerId || frame.data.size() != 8
            || !std::equal(signature.begin(), signature.end(), frame.data.begin())) continue;
        const auto inserted = bySequence.emplace(frame.data[1], frame);
        if (!inserted.second) throw std::runtime_error("Duplicate recon sequence frame");
    }
    if (bySequence.size() != frameCount) {
        throw std::runtime_error("Incomplete recon capture: expected "
            + std::to_string(frameCount) + " frames, got " + std::to_string(bySequence.size()));
    }
    ReconImage image;
    uint8_t expectedSequence = 0;
    uint32_t expectedAddress = 0;
    for (const auto& [sequence, frame] : bySequence) {
        if (sequence != expectedSequence) throw std::runtime_error("Recon sequence gap");
        const auto address = (static_cast<uint32_t>(frame.data[2]) << 8) | frame.data[3];
        if (expectedSequence == 0) {
            image.address = address;
            expectedAddress = address;
        }
        if (address != expectedAddress) throw std::runtime_error("Recon address gap");
        image.sequence.push_back(sequence);
        image.bytes.insert(image.bytes.end(), frame.data.begin() + 4, frame.data.end());
        expectedAddress += 4;
        ++expectedSequence;
    }
    return image;
}

void runReconCapture(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    ReconManifestGuard manifest{options};
    if (options.reconCanId > 0x1FFFFFFF || options.reconMarker.empty()
        || options.reconRequestCanId > 0x1FFFFFFF || options.reconResponseCanId > 0x1FFFFFFF)
        throw std::runtime_error("recon-capture requires a capture CAN id and marker");
    if (options.reconMarker.size() > 8)
        throw std::runtime_error("recon marker must fit in one CAN frame");
    if (options.reconCaptureCount == 0)
        throw std::runtime_error("recon capture count must be greater than zero");
    if (options.reconStubPath.empty() && options.reconLoadLength != 0)
        throw std::runtime_error("recon-capture load address/length require --load-stub-bin");

    const auto device = common::selectSingleDevice(devices, options.deviceName);
    auto j2534 = openDevice(device);
    const auto bus = selectedRawCanBus(options);
    manifest.baudrate = options.baudrateOverride.value_or(bus.baudrate);
    auto raw = openReconRawChannel(*j2534, bus, options);
    raw->clearRx();
    SoftIsoTp iso{*raw, options.reconRequestCanId, options.reconResponseCanId,
        {options.reconIsoTpPadding, options.reconIsoTpPadToEight,
         options.reconIsoTpBlockSize, options.reconIsoTpStmin}};
    const auto udsRequest = [&](const std::vector<uint8_t>& request) {
        iso.sendRequest(request, options.timeoutMs);
        return iso.receiveResponse(options.timeoutMs);
    };
    if (options.reconSmokeOnly) {
        expectPrefix(udsRequest(options.reconHealthSessionRequest),
            options.reconHealthSessionExpect, "ISO-TP smoke health session");
        manifest.verdict = "smoke-passed";
        return;
    }

    if (!options.reconStubPath.empty()) {
        std::ifstream input(options.reconStubPath, std::ios::binary);
        if (!input) throw std::runtime_error("Failed to open recon stub: " + options.reconStubPath);
        std::vector<uint8_t> stub((std::istreambuf_iterator<char>(input)), {});
        if (stub.size() != options.reconLoadLength)
            throw std::runtime_error("Recon stub length mismatch");
        if (options.reconStubSha256.empty())
            throw std::runtime_error("--stub-sha256 is required when loading a stub");
        manifest.stubSha256 = sha256Hex(stub);
        if (manifest.stubSha256 != options.reconStubSha256)
            throw std::runtime_error("Recon stub SHA-256 mismatch: " + manifest.stubSha256);
        std::vector<uint8_t> request{0x34, 0x00, 0x44,
            static_cast<uint8_t>(options.reconLoadAddress >> 24), static_cast<uint8_t>(options.reconLoadAddress >> 16),
            static_cast<uint8_t>(options.reconLoadAddress >> 8), static_cast<uint8_t>(options.reconLoadAddress),
            static_cast<uint8_t>(stub.size() >> 24), static_cast<uint8_t>(stub.size() >> 16),
            static_cast<uint8_t>(stub.size() >> 8), static_cast<uint8_t>(stub.size())};
        expectPrefix(udsRequest(request), {0x74}, "RequestDownload");
        request = {0x36, 0x01}; request.insert(request.end(), stub.begin(), stub.end());
        expectPrefix(udsRequest(request), {0x76, 0x01}, "TransferData");
        expectPrefix(udsRequest({0x37}), {0x77}, "RequestTransferExit");
    }

    std::vector<uint8_t> trigger{0x31, 0x01,
        static_cast<uint8_t>(options.reconRoutineId >> 8), static_cast<uint8_t>(options.reconRoutineId)};
    trigger.insert(trigger.end(), options.reconRoutineData.begin(), options.reconRoutineData.end());
    std::vector<ReconFrame> frames;
    const auto appendFrame = [&](uint64_t timeMs, uint32_t id, std::vector<uint8_t> data) {
        if (id == options.reconCanId && data.size() == 8
            && std::equal(options.reconMarker.begin(), options.reconMarker.end(), data.begin())) {
            frames.push_back({timeMs, id, std::move(data)});
        }
    };
    expectPrefix(udsRequest(trigger), {0x71, 0x01}, "RoutineControl");

    // The stub may emit A4 frames before its positive 71 response. SoftIsoTp
    // preserves those non-ISO-TP frames while waiting for the UDS response.
    for (auto& rawFrame : iso.takeNonIsoTpFrames()) {
        appendFrame(rawFrame.driverTimestamp / 1000, rawFrame.canId, std::move(rawFrame.data));
    }

    const auto start = std::chrono::steady_clock::now();
    while (frames.size() < options.reconCaptureCount && !stopRequested.load()) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= static_cast<long long>(options.reconCaptureTimeoutMs)) break;
        for (const auto& msg : readBatch(*raw, 50)) {
            if (msg.DataSize < 4) continue;
            const auto id = common::canIdFromFrame(msg);
            auto data = payload(msg);
            appendFrame(static_cast<uint64_t>(elapsed), id, std::move(data));
        }
    }
    manifest.frameCount = frames.size();
    std::filesystem::path outputPath(options.reconOutputPath);
    if (!outputPath.parent_path().empty()) std::filesystem::create_directories(outputPath.parent_path());
    std::ofstream csv(outputPath);
    if (!csv) throw std::runtime_error("Failed to open recon CSV: " + options.reconOutputPath);
    csv << "time_ms,can_id,dlc,data\n";
    for (const auto& frame : frames) csv << frame.timeMs << "," << hexNumber(frame.canId, 3)
        << "," << frame.data.size() << "," << csvEscape(formatBytes(frame.data)) << "\n";
    manifest.image = reconstructReconImage(frames, options.reconCanId, options.reconMarker, options.reconCaptureCount);
    // Health is the safety gate; it must run even when reference bytes mismatch.
    expectPrefix(udsRequest(options.reconHealthSessionRequest),
        options.reconHealthSessionExpect, "Health session");
    expectPrefix(udsRequest(options.reconHealthDidRequest),
        options.reconHealthDidExpect, "Health DID");
    if (!options.reconReferenceBytes.empty() && manifest.image.address != options.reconReferenceAddress)
        throw std::runtime_error("Recon image starts at an unexpected address: "
            + hexNumber(manifest.image.address, 4));
    if (!options.reconReferenceBytes.empty() && manifest.image.bytes != options.reconReferenceBytes)
        throw std::runtime_error("Recon image does not match reference bytes");
    manifest.verdict = "passed";
}

} // namespace volvodiag
