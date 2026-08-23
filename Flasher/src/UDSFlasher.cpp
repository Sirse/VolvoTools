#include "flasher/UDSFlasher.hpp"

#include <j2534/J2534.hpp>
#include <j2534/J2534Channel.hpp>

#include <common/CliSupport.hpp>
#include <common/CommonData.hpp>
#include <common/protocols/UDSProtocolCommonSteps.hpp>
#include <common/protocols/UDSRequest.hpp>
#include <common/Util.hpp>

#include <easylogging++.h>

#include <optional>
#include <stdexcept>

#define HFSM2_ENABLE_ALL
#include <common/hfsm2/machine.hpp>

namespace flasher {

    class UDSFlasherImpl {
    public:
        UDSFlasherImpl(const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels,
                       const FlasherParameters& flasherParameters,
                       const UDSFlasherParameters& udsFlasherParameters,
                       uint32_t canId,
                       const std::function<void(FlasherState)>& stateUpdater,
                       const std::function<void(size_t)>& progressUpdater,
                       const std::function<void(size_t)> progressSetter,
                       const std::function<size_t()> progressGetter,
                       const std::function<void(const std::string&)>& errorUpdater)
            : _channels{ channels }
            , _flasherParameters{ flasherParameters }
            , _udsFlasherParameters{ udsFlasherParameters }
            , _canId{ canId }
            , _isFailed{ false }
            , _stateUpdater{ stateUpdater }
            , _progressUpdater{ progressUpdater }
            , _progressSetter{ progressSetter }
            , _progressGetter{ progressGetter }
            , _errorUpdater{ errorUpdater }
        {
        }

        size_t getMaximumProgress()
        {
            size_t progress = FlasherBase::getProgressFromVBF(_flasherParameters.flash);
            if (!_udsFlasherParameters.attachRunningSbl && _flasherParameters.sblProvider) {
                const auto bootloader{ _flasherParameters.sblProvider->getSBL(
                    _flasherParameters.carPlatform, _flasherParameters.ecuId, _flasherParameters.additionalData)};
                progress += FlasherBase::getProgressFromVBF(bootloader);
            }
            return progress;
        }

        void fallAsleep()
        {
            _stateUpdater(FlasherState::FallAsleep);
            if (_udsFlasherParameters.attachRunningSbl) {
                LOG(INFO) << "Attaching to already running UDS SBL, skipping programming-session broadcast";
                return;
            }
            if (_udsFlasherParameters.skipFallAsleep) {
                LOG(INFO) << "Programming-session broadcast skipped, vehicle programming mode was prepared by CEM";
                return;
            }
            // Bench prelude: raise every module with the suppressed 10 82 form, then confirm
            // with 10 02 that a module actually answered 50 02. The old code only sent 10 02 and
            // treated a successful periodic-message start as success even if nothing came up.
            if (!common::UDSProtocolCommonSteps::broadcastProgrammingSessionPrelude(_channels)) {
                setFailed("No module confirmed programming session");
            }
        }

        void keepAlive()
        {
            if (_udsFlasherParameters.attachRunningSbl) {
                return;
            }
            auto& channel{ common::getChannelByEcuId(_flasherParameters.carPlatform, _flasherParameters.ecuId, _channels) };
            common::UDSProtocolCommonSteps::keepAlive(channel);
        }

        void authorize()
        {
            _stateUpdater(FlasherState::Authorize);
            if (_udsFlasherParameters.attachRunningSbl) {
                return;
            }
            auto& channel{ common::getChannelByEcuId(_flasherParameters.carPlatform, _flasherParameters.ecuId, _channels) };
            try {
                common::UDSRequest programmingSession{ _canId, { 0x10, 0x02 } };
                programmingSession.process(channel, { 0x02 }, 1, 1000);
            }
            catch (const std::exception& ex) {
                LOG(WARNING) << "Programming session request before authorize failed: " << ex.what();
            }
            if (!common::UDSProtocolCommonSteps::authorizeWithRetry(channel, _canId, _udsFlasherParameters.pin)) {
                setFailed("Authorization failed");
            }
        }

        void loadBootloader()
        {
            _stateUpdater(FlasherState::LoadBootloader);
            if (_udsFlasherParameters.attachRunningSbl) {
                return;
            }
            if (_flasherParameters.sblProvider) {
                const auto bootloader{ _flasherParameters.sblProvider->getSBL(
                    _flasherParameters.carPlatform, _flasherParameters.ecuId, _flasherParameters.additionalData)};
                auto& channel{ common::getChannelByEcuId(_flasherParameters.carPlatform, _flasherParameters.ecuId, _channels) };
                if (bootloader.chunks.empty() || !common::UDSProtocolCommonSteps::transferData(channel, _canId, bootloader,
                                                                                               _progressUpdater)) {
                    setFailed("Bootloader loading failed");
                }
            }
        }

        void startBootloader()
        {
            _stateUpdater(FlasherState::StartBootloader);
            if (_udsFlasherParameters.attachRunningSbl) {
                auto& channel{ common::getChannelByEcuId(_flasherParameters.carPlatform, _flasherParameters.ecuId, _channels) };
                if (!authorizeRunningSbl(channel)) {
                    setFailed("SBL attach authorization failed");
                }
            }
            else if (_flasherParameters.sblProvider) {
                const auto bootloader{ _flasherParameters.sblProvider->getSBL(
                    _flasherParameters.carPlatform, _flasherParameters.ecuId, _flasherParameters.additionalData) };
                auto& channel{ common::getChannelByEcuId(_flasherParameters.carPlatform, _flasherParameters.ecuId, _channels) };
                if (!common::UDSProtocolCommonSteps::startRoutine(channel, _canId, bootloader.header.call)) {
                    setFailed("Bootloader starting failed");
                    return;
                }
                if (!authorizeRunningSbl(channel)) {
                    setFailed("SBL post-start authorization failed");
                }
            }
        }

        void writeFlash()
        {
            auto& channel{ common::getChannelByEcuId(_flasherParameters.carPlatform, _flasherParameters.ecuId, _channels) };
            // This path writes chunks directly, so it bypasses the guard in transferData.
            // Check here too: writing packed payload as plain data bricks the ECU.
            if (common::isPackedDataFormat(_flasherParameters.flash.header)) {
                setFailed("Flash file has packed payload (data_format_identifier=0x"
                    + common::toHexString({ _flasherParameters.flash.header.dataFormatIdentifier })
                    + "), unpacking is not implemented");
                return;
            }
            // From here on interruption cannot be made safe: erased flash leaves the ECU
            // without a valid application. Swallow stop events and run erase/write/verify
            // to completion; the region ends with this object, covering checkValidApplication
            // and the FSM teardown states as well.
            _destructiveRegion.emplace("ECU flashing (erase/write/verify)");
            _stateUpdater(FlasherState::EraseFlash);
            if (!common::UDSProtocolCommonSteps::eraseFlash(channel, _canId, _flasherParameters.flash)) {
                setFailed("Flash erasing failed");
                return;
            }
            for(const auto& chunk: _flasherParameters.flash.chunks) {
                _stateUpdater(FlasherState::WriteFlash);
                bool chunkWritten = false;
                size_t progressBeforeChunk = _progressGetter();
                // A failed first attempt already advanced the progress bar; rewind to the
                // chunk start so the retry does not count its bytes twice.
                for (size_t attempt = 1; attempt <= 2 && !chunkWritten; ++attempt) {
                    if (attempt > 1) {
                        LOG(WARNING) << "Retry transferChunk attempt=" << attempt
                                     << " offset=0x" << std::hex << chunk.writeOffset;
                        _progressSetter(progressBeforeChunk);
                    }
                    chunkWritten = common::UDSProtocolCommonSteps::transferChunk(channel, _canId, chunk,
                                                                                 _progressUpdater);
                }
                if (!chunkWritten) {
                    setFailed("Flash writing failed");
                    return;
                }
            }
            // Ask the ECU whether what we just wrote hangs together, before we tell it the
            // application is valid. Per written region, mirroring the factory tool.
            for (const auto& chunk : _flasherParameters.flash.chunks) {
                if (chunk.data.empty()) {
                    continue;
                }
                if (!common::UDSProtocolCommonSteps::checkProgrammingDependencies(
                        channel, _canId, chunk.writeOffset, static_cast<uint32_t>(chunk.data.size()))) {
                    setFailed("Programming dependencies check failed");
                    return;
                }
            }
        }

        void checkValidApplication()
        {
            auto& channel{ common::getChannelByEcuId(_flasherParameters.carPlatform, _flasherParameters.ecuId, _channels) };
            if (!common::UDSProtocolCommonSteps::checkValidApplication(channel, _canId)) {
                setFailed("Application validation failed");
            }
        }

        void wakeUp()
        {
            _stateUpdater(FlasherState::WakeUp);
            common::UDSProtocolCommonSteps::broadcastEcuReset(_channels);
        }


        void done()
        {
            _stateUpdater(FlasherState::Done);
        }

        void error()
        {
            _stateUpdater(FlasherState::Error);
        }

        bool isFailed() const
        {
            return _isFailed;
        }

        // Cooperative stop, honored only before the destructive phase: fails the plan so
        // the FSM unwinds through its normal Finish/WakeUp(ECUReset) teardown. Once the
        // uninterruptible region is active the console handler swallows Ctrl+C anyway.
        void abortBeforeErasing()
        {
            setFailed("Stopped by user before erasing; no destructive step had been done");
        }

    private:
        bool authorizeRunningSbl(const j2534::J2534Channel& channel)
        {
            return common::UDSProtocolCommonSteps::authorizeWithRetry(channel, _canId, _udsFlasherParameters.pin);
        }

        void setFailed(const std::string& message)
        {
            _isFailed = true;
            LOG(ERROR) << message;
            _errorUpdater(message);
        }

    private:
        const std::vector<std::unique_ptr<j2534::J2534Channel>>& _channels;
        const FlasherParameters& _flasherParameters;
        const UDSFlasherParameters& _udsFlasherParameters;
        const uint32_t _canId;
        bool _isFailed;
        // Active from the first erase call until the whole flash run (including application
        // validation and FSM teardown) has finished; see writeFlash().
        std::optional<common::UninterruptibleRegion> _destructiveRegion;
        const std::function<void(FlasherState)> _stateUpdater;
        const std::function<void(size_t)> _progressUpdater;
        const std::function<void(size_t)> _progressSetter;
        const std::function<size_t()> _progressGetter;
        const std::function<void(const std::string&)> _errorUpdater;
    };

using M = hfsm2::MachineT<hfsm2::Config::ContextT<UDSFlasherImpl&>>;
    using FSM = M::PeerRoot<
        M::Composite<
            struct StartWork,
            struct FallAsleep,
            struct KeepAlive,
            struct Authorize,
            struct LoadBootloader,
            struct StartBootloader,
            struct WriteFlash,
            struct CheckValidApplication>,
        M::Composite<
            struct Finish,
            struct WakeUp,
            struct Done,
            struct Error>
        >;

    struct BaseState : public FSM::State {
    public:
        void update(FullControl& control)
        {
            if (!control.context().isFailed()) {
                control.succeed();
            }
            else {
                control.fail();
            }
        }
    };

    struct BaseSuccesState : public FSM::State {
    public:
        void update(FullControl& control)
        {
            control.succeed();
        }
    };

    struct StartWork : public FSM::State {
        void enter(PlanControl& control)
        {
            auto plan = control.plan();
            plan.change<FallAsleep, KeepAlive>();
            plan.change<KeepAlive, Authorize>();
            plan.change<Authorize, LoadBootloader>();
            plan.change<LoadBootloader, StartBootloader>();
            plan.change<StartBootloader, WriteFlash>();
            plan.change<WriteFlash, CheckValidApplication>();
        }

        void planSucceeded(FullControl& control) {
            control.changeTo<Finish>();
        }

        void planFailed(FullControl& control)
        {
            control.changeTo<Finish>();
        }
    };

    struct FallAsleep : public BaseState {
        void enter(PlanControl& control)
        {
            control.context().fallAsleep();
        }
    };

    struct KeepAlive : public BaseState {
        void enter(PlanControl& control)
        {
            control.context().keepAlive();
        }
    };

    struct Authorize : public BaseState {
        void enter(PlanControl& control)
        {
            control.context().authorize();
        }
    };

    struct LoadBootloader : public BaseState {
        void enter(PlanControl& control)
        {
            control.context().loadBootloader();
        }
    };

    struct StartBootloader : public BaseState {
        void enter(PlanControl& control)
        {
            control.context().startBootloader();
        }
    };

    struct WriteFlash : public BaseState {
        void enter(PlanControl& control)
        {
            control.context().writeFlash();
        }
    };

    struct CheckValidApplication : public BaseState {
        void enter(PlanControl& control)
        {
            control.context().checkValidApplication();
        }
    };

    struct Finish : public FSM::State {
        void enter(PlanControl& control)
        {
            auto plan = control.plan();
            if (control.context().isFailed()) {
                plan.change<WakeUp, Error>();
            }
            else {
                plan.change<WakeUp, Done>();
            }
        }
    };

    struct WakeUp : public BaseSuccesState {
        void enter(PlanControl& control)
        {
            control.context().wakeUp();
        }
    };

    struct Done : public BaseSuccesState {
        void enter(PlanControl& control)
        {
            control.context().done();
        }
    };

    struct Error : public BaseSuccesState {
        void enter(PlanControl& control)
        {
            control.context().error();
        }
    };

    UDSFlasher::UDSFlasher(j2534::J2534& j2534, FlasherParameters&& flasherParameters, UDSFlasherParameters&& udsFlasherParameters)
        : FlasherBase{ j2534, std::move(flasherParameters) }
        , _udsFlasherParameters{ std::move(udsFlasherParameters) }
    {
    }

    UDSFlasher::~UDSFlasher()
    {
    }

    std::vector<std::unique_ptr<j2534::J2534Channel>> UDSFlasher::openChannels()
    {
        LOG(INFO) << "UDSFlasher opening target ECU channel only, ecu=0x" << std::hex
            << getFlasherParameters().ecuId;
        std::vector<std::unique_ptr<j2534::J2534Channel>> channels;
        auto channel = openChannelForEcu(getFlasherParameters().ecuId);
        if (!channel) {
            throw std::runtime_error("Failed to open UDS target ECU channel");
        }
        channels.emplace_back(std::move(channel));
        LOG(INFO) << "UDSFlasher target ECU channel opened";
        return channels;
    }

    void UDSFlasher::startImpl(std::vector<std::unique_ptr<j2534::J2534Channel>>& channels)
    {
        LOG(INFO) << "UDSFlasher startImpl enter, channels=" << channels.size();
        const auto ecuInfo{ common::getEcuInfoByEcuId(getFlasherParameters().carPlatform,
            getFlasherParameters().ecuId) };

        LOG(INFO) << "UDSFlasher target CAN ID=0x" << std::hex << std::get<1>(ecuInfo).canId;
        UDSFlasherImpl impl(channels, getFlasherParameters(), _udsFlasherParameters,
            std::get<1>(ecuInfo).canId, [this](FlasherState state) {
            setCurrentState(state);
        },
            [this](size_t progress) {
                incCurrentProgress(progress);
            },
            [this](size_t value) {
                setCurrentProgress(value);
            },
            [this]() {
                return getCurrentProgress();
            },
            [this](const std::string& error) {
                setLastError(error);
            });

        LOG(INFO) << "UDSFlasher calculating maximum progress";
        setMaximumProgress(impl.getMaximumProgress());
        LOG(INFO) << "UDSFlasher maximum progress=" << std::dec << getMaximumProgress();

        LOG(INFO) << "UDSFlasher FSM enter";
        FSM::Instance fsm{ impl };

        while(getCurrentState() != FlasherState::Done && getCurrentState() != FlasherState::Error) {
            // A stop requested before any destructive step is a clean abort: fail the plan
            // and let the FSM unwind through Finish/WakeUp(ECUReset) teardown. Once the
            // destructive region is active the console handler swallows stop events anyway
            // and the flash runs to completion by design.
            if (!impl.isFailed() && !common::isUninterruptibleRegionActive()
                && common::stopRequested.exchange(false)) {
                impl.abortBeforeErasing();
            }
            fsm.update();
        }
        LOG(INFO) << "UDSFlasher FSM exit, state=" << static_cast<int>(getCurrentState());
    }

} // namespace flasher
