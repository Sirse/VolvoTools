#include "logger/Logger.hpp"

#include "logger/LoggerCallback.hpp"

#include <common/CommonData.hpp>
#include <common/protocols/UDSRequest.hpp>
#include <common/protocols/UDSProtocolCommonSteps.hpp>
#include <common/Util.hpp>
#include <j2534/J2534.hpp>
#include <j2534/J2534Channel.hpp>

#include <easylogging++.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <numeric>
#include <ios>
#include <sstream>

namespace logger {

    template<typename T>
    std::string dumpArray(const T& vec)
    {
        std::stringstream ss;
        for(const auto& i: vec) {
            ss << std::hex << int(i) << " ";
        }
        return ss.str();
    }

	class LoggerImpl {
	public:
        LoggerImpl() {}

		virtual void registerParameters(j2534::J2534Channel& channel,
			const LogParameters& parameters) = 0;
		virtual std::vector<uint32_t>
			requestMemory(j2534::J2534Channel& channel,
				const LogParameters& parameters) = 0;
	};

	class UDSLoggerImpl : public LoggerImpl {
	public:
        UDSLoggerImpl(uint32_t canId, UdsLoggerOptions options)
            : LoggerImpl()
            , _canId{ canId }
			, _didBase(options.didBase)
            , _didMaxDataSize{ options.didMaxDataSize }
		{
            if (_didMaxDataSize == 0) {
                throw std::runtime_error("UDS DID max data size must be greater than 0");
            }
		}

	private:
        /**
         * @brief Get DID index or create new
         * @param logParameter
         * @return
         */
        size_t getFittingDidIndex(const LogParameter& logParameter) {
            if (logParameter.size() > _didMaxDataSize) {
                throw std::runtime_error("Parameter \"" + logParameter.name()
                    + "\" of size " + std::to_string(logParameter.size())
                    + " does not fit into a single UDS DID (max " + std::to_string(_didMaxDataSize)
                    + " bytes); increase --uds-did-max-data-size");
            }
            uint16_t maxId = _didBase - 1;
            for(size_t i = 0; i < _didRequests.size(); ++i) {
                if(_didRequests[i].freeSize >= logParameter.size()) {
                    return i;
                }
                maxId = std::max(maxId, _didRequests[i].didId);
            }
            _didRequests.emplace_back(DidInfo(maxId + 1, _didMaxDataSize));
            return _didRequests.size() - 1;
        }

		virtual void
			registerParameters(j2534::J2534Channel& channel,
				const LogParameters& parameters) override {

            common::UDSRequest diagSessionRequest{_canId, { 0x10, 0x03 }};
            // Registration failures must be loud: a silent return left the DDDIDs
            // unregistered, the loop read nothing but unregistered DID ids, and the real
            // problem surfaced later as ten anonymous "request failed" errors.
            if(diagSessionRequest.process(channel).empty()) {
                throw std::runtime_error("Logger registration: extended session (10 03) got no response");
            }
            _didRequests.clear();
            for (size_t i = 0; i < parameters.parameters().size(); ++i) {
                const auto& param = parameters.parameters()[i];
                size_t didIndex = getFittingDidIndex(param);
                _didRequests[didIndex].paramIndexes.push_back(i);
                _didRequests[didIndex].freeSize -= param.size();
            }
            for (const auto& didRequest: _didRequests) {
                const auto did = didRequest.didId;
                const auto didText = common::toHexString({ static_cast<uint8_t>(did >> 8),
                    static_cast<uint8_t>(did & 0xFF) });
                common::UDSRequest clearDDDIRequest{_canId, { 0x2C, 0x03, static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did) }};
                const auto clearResponse{clearDDDIRequest.process(channel)};
                if(clearResponse.empty()) {
                    throw std::runtime_error("Logger registration: dynamic DID clear (2C 03) for DID "
                        + didText + " got no response");
                }
                constexpr uint8_t addrLength = 4;
                constexpr uint8_t dataLength = 2;
                constexpr uint8_t dataFormat = (dataLength << 4) + addrLength;
                std::vector<uint8_t> formattedParams{ 0x2C, 0x02, static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did), dataFormat };
                for(const auto& paramIndex: didRequest.paramIndexes) {
                    const auto& param = parameters.parameters()[paramIndex];
                    const auto formattedAddr = common::toVector(param.addr());
                    const auto formattedSize = common::toVector(static_cast<uint16_t>(param.size()));
                    formattedParams.insert(formattedParams.end(), formattedAddr.cbegin(), formattedAddr.cend());
                    formattedParams.insert(formattedParams.end(), formattedSize.cbegin(), formattedSize.cend());
                }
                common::UDSRequest registerRequest(_canId, formattedParams);
                if(registerRequest.process(channel).empty()) {
                    throw std::runtime_error("Logger registration: dynamic DID define (2C 02) for DID "
                        + didText + " got no response");
                }
            }
		}

		virtual std::vector<uint32_t>
			requestMemory(j2534::J2534Channel& channel,
				const LogParameters& parameters) override {
            std::vector<uint32_t> result(parameters.parameters().size());
            for (const auto& didRequest: _didRequests) {
                const auto did = didRequest.didId;
                common::UDSRequest requestDid{_canId, { 0x22, static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did) }};
                const auto data{requestDid.process(channel)};
                size_t paramIndex = 0;
                size_t paramOffset = 0;
                uint32_t value = 0;
                for(size_t i = 7; i < data.size(); ++i) {
                    const size_t initialParamIndex{didRequest.paramIndexes[paramIndex]};
                    const auto& param = parameters.parameters()[initialParamIndex];
                    value += data[i] << ((param.size() - paramOffset - 1) * 8);
                    ++paramOffset;
                    if (paramOffset >= param.size()) {
                        result[initialParamIndex] = value;
                        ++paramIndex;
                        paramOffset = 0;
                        value = 0;
                    }
                    if (paramIndex >= didRequest.paramIndexes.size()) {
                        break;
                    }
                }
            }
			return result;
		}

        struct DidInfo {
            DidInfo(uint16_t didId, size_t freeSize)
                : didId{ didId }
                , freeSize{ freeSize }
            {
            }
            DidInfo(const DidInfo&) = default;
            DidInfo(DidInfo&&) = default;

            uint16_t didId;
            std::vector<size_t> paramIndexes;
            size_t freeSize;
        };

        const uint32_t _canId;
		const uint16_t _didBase;
        const size_t _didMaxDataSize;
        std::vector<DidInfo> _didRequests;
	};


    std::unique_ptr<LoggerImpl> createLoggerImpl(common::CarPlatform carPlatform, uint32_t cmId,
                                                 UdsLoggerOptions udsOptions)
	{
        const common::ECUInfo ecuInfo{ std::get<1>(common::getEcuInfoByEcuId(carPlatform, cmId)) };
        // The fork is P3-only, served by the UDS logger; cmId selects which ECU to read.
        return std::make_unique<UDSLoggerImpl>(ecuInfo.canId, udsOptions);
	}

    Logger::Logger(j2534::J2534& j2534, common::CarPlatform carPlatform, uint32_t ecuId,
                   const std::string& cmInfo, std::optional<uint32_t> baudrateOverride,
                   UdsLoggerOptions udsOptions)
        : _j2534ChannelProvider{ j2534, carPlatform, baudrateOverride }
		, _carPlatform{ carPlatform }
		, _ecuId{ ecuId }
		, _cmInfo{ cmInfo }
		, _loggingInterval{ std::chrono::milliseconds(50) }
		, _udsOptions{ udsOptions }
		, _loggingThread{}
		, _stopped{ true }
        , _loggerImpl(createLoggerImpl(_carPlatform, _ecuId, _udsOptions)) {
        LOG(DEBUG) << "Logger ctor done";
	}

	Logger::~Logger() { stop(); }

	void Logger::registerCallback(LoggerCallback& callback) {
        LOG(DEBUG) << "Logger registerCallback enter callback=0x"
            << std::hex << reinterpret_cast<uintptr_t>(&callback);
		std::unique_lock<std::mutex> lock{ _callbackMutex };
		if (std::find(_callbacks.cbegin(), _callbacks.cend(), &callback) ==
			_callbacks.cend()) {
			_callbacks.push_back(&callback);
		}
        LOG(DEBUG) << "Logger registerCallback exit count=" << std::dec
            << _callbacks.size();
	}

	void Logger::unregisterCallback(LoggerCallback& callback) {
		std::unique_lock<std::mutex> lock{ _callbackMutex };
		_callbacks.erase(std::remove(_callbacks.begin(), _callbacks.end(), &callback),
			_callbacks.end());
	}

	void Logger::start(const LogParameters& parameters, std::chrono::milliseconds loggingInterval) {
		std::unique_lock<std::mutex> lock{ _mutex };
		if (!_stopped) {
			throw std::runtime_error("Logging already started");
		}
		if (loggingInterval.count() <= 0) {
			throw std::runtime_error("Logging interval must be positive");
		}

		_parameters = parameters;
		_loggingInterval = loggingInterval;
		_stopped = false;

		_callbackThread = std::thread([this]() { callbackFunction(); });
		_loggingThread = std::thread([this]() { logFunction(); });
	}

	void Logger::stop() {
		{
			std::unique_lock<std::mutex> lock{ _mutex };
			_stopped = true;
		}
		// Wake the logging thread out of wait_until: without this, stop() blocked until
		// the end of the current interval (a minute at --interval-ms 60000).
		_cond.notify_all();
		if (_loggingThread.joinable())
			_loggingThread.join();

		{
			std::unique_lock<std::mutex> lock{ _callbackMutex };
			_callbackCond.notify_all();
		}

		if (_callbackThread.joinable())
			_callbackThread.join();

	}

	bool Logger::isStarted() const {
		std::unique_lock<std::mutex> lock{ _mutex };
		return !_stopped;
	}

	void Logger::registerParameters(j2534::J2534Channel& channel) {
        LOG(INFO) << "Logger register parameters enter";
        _loggerImpl->registerParameters(channel, _parameters);
        LOG(INFO) << "Logger register parameters exit";
	}

	void Logger::logFunction() {
		{
			std::unique_lock<std::mutex> lock{ _callbackMutex };
			for (const auto callback : _callbacks) {
				callback->onStatusChanged(true);
			}
		}
		const size_t maxErrorCount = 10;
		size_t errorCount = 0;
		try {
			LOG(INFO) << "Logger opening ECU channel in logging thread, ecu=0x" << std::hex << _ecuId;
			auto channel = _j2534ChannelProvider.getChannelForEcu(_ecuId);
			LOG(INFO) << "Logger ECU channel opened";
			registerParameters(*channel);
			LOG(INFO) << "Logger loop enter";
			const auto startTimepoint{ std::chrono::steady_clock::now() };
			const auto loggingInterval = _loggingInterval;
			auto nextWakeup = startTimepoint + loggingInterval;
			while (errorCount < maxErrorCount) {
				{
					std::unique_lock<std::mutex> lock{ _mutex };
					if (_stopped)
						break;
				}
				try {
					channel->clearRx();
					channel->clearTx();
					auto logRecord = _loggerImpl->requestMemory(*channel, _parameters);
					const auto now{ std::chrono::steady_clock::now() };
					pushRecord(LogRecord(std::chrono::duration_cast<std::chrono::milliseconds>(
						now - startTimepoint),
						std::move(logRecord)));
					errorCount = 0;
				}
				catch(const std::exception& ex) {
					LOG(ERROR) << "Logger request failed: " << ex.what();
					++errorCount;
				}
				catch(...) {
					LOG(ERROR) << "Logger request failed with unknown exception";
					++errorCount;
				}
				std::unique_lock<std::mutex> lock{ _mutex };
				_cond.wait_until(lock, nextWakeup);
				const auto now = std::chrono::steady_clock::now();
				do {
					nextWakeup += loggingInterval;
				} while (nextWakeup <= now);
			}
			LOG(INFO) << "Logger loop exit, errorCount=" << errorCount;
		}
		catch(const std::exception& ex) {
            LOG(ERROR) << "Logger initialization failed: " << ex.what();
            {
                std::unique_lock<std::mutex> lock{ _mutex };
                _stopped = true;
            }
            {
                std::unique_lock<std::mutex> lock{ _callbackMutex };
                _callbackCond.notify_all();
                for (const auto callback : _callbacks) {
                    callback->onStatusChanged(false);
                }
            }
            return;
        }
        catch(...) {
            LOG(ERROR) << "Logger initialization failed with unknown exception";
            {
                std::unique_lock<std::mutex> lock{ _mutex };
                _stopped = true;
            }
            {
                std::unique_lock<std::mutex> lock{ _callbackMutex };
                _callbackCond.notify_all();
                for (const auto callback : _callbacks) {
                    callback->onStatusChanged(false);
                }
            }
            return;
        }
        {
            std::unique_lock<std::mutex> lock{ _mutex };
            _stopped = true;
        }
        {
			std::unique_lock<std::mutex> lock{ _callbackMutex };
            _callbackCond.notify_all();
            for (const auto callback : _callbacks) {
				callback->onStatusChanged(false);
			}
		}
	}

	void Logger::pushRecord(Logger::LogRecord&& record) {
		std::unique_lock<std::mutex> lock{ _callbackMutex };
		// Bound the queue: a slow writer must not grow memory for the whole session.
		constexpr size_t kMaxQueuedRecords = 10000;
		if (_loggedRecords.size() >= kMaxQueuedRecords) {
			_loggedRecords.pop_front();
			if (++_droppedRecords == 1 || (_droppedRecords % 1000) == 0) {
				LOG(WARNING) << "Logger dropped " << _droppedRecords << " records total (consumer too slow)";
			}
		}
		_loggedRecords.emplace_back(std::move(record));
		_callbackCond.notify_all();
	}

	void Logger::callbackFunction() {
		for (;;) {
			LogRecord logRecord;
			{
				std::unique_lock<std::mutex> lock{ _callbackMutex };
				_callbackCond.wait(
					lock, [this] { return !isStarted() || !_loggedRecords.empty(); });
				if (!isStarted() && _loggedRecords.empty()) {
					break;
				}
				logRecord = _loggedRecords.front();
				_loggedRecords.pop_front();
			}
			std::vector<double> formattedValues(_parameters.parameters().size());
			for (size_t i = 0; i < formattedValues.size() && i < logRecord.values.size(); ++i) {
				formattedValues[i] =
					_parameters.parameters()[i].formatValue(logRecord.values[i]);
			}
			{
				std::unique_lock<std::mutex> lock{ _callbackMutex };
				for (const auto callback : _callbacks) {
					callback->onLogMessage(logRecord.timePoint, formattedValues);
				}
			}
		}
	}

} // namespace logger
