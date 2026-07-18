#pragma once

#include "LogParameters.hpp"

#include <common/CarPlatform.hpp>
#include <common/ConfigurationInfo.hpp>
#include <common/J2534ChannelProvider.hpp>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace logger {
	class LoggerCallback;

	enum class LoggerType { LT_D2, LT_UDS };

	class LoggerImpl;

	struct UdsLoggerOptions {
		uint16_t didBase = 0xF200;
		size_t didMaxDataSize = 7;
	};

	class Logger final {
	public:
        explicit Logger(j2534::J2534& j2534, common::CarPlatform carPlatform, uint32_t ecuId,
                        const std::string& cmInfo, std::optional<uint32_t> baudrateOverride = std::nullopt,
                        UdsLoggerOptions udsOptions = {});
		~Logger();

		void registerCallback(LoggerCallback& callback);
		void unregisterCallback(LoggerCallback& callback);

		void start(const LogParameters& parameters, std::chrono::milliseconds loggingInterval);
		void stop();
		bool isStarted() const;

	private:
		void registerParameters(j2534::J2534Channel& channel);

		void logFunction();

		struct LogRecord {
			LogRecord() = default;
			LogRecord(std::chrono::milliseconds timePoint,
				std::vector<uint32_t>&& values)
				: timePoint{ timePoint }, values(std::move(values)) {
			}
			std::chrono::milliseconds timePoint;
			std::vector<uint32_t> values;
		};

		void pushRecord(LogRecord&& record);
		void callbackFunction();

	private:
		common::J2534ChannelProvider _j2534ChannelProvider;
		common::CarPlatform _carPlatform;
        uint32_t _ecuId;
		std::string _cmInfo;
		LogParameters _parameters;
		std::chrono::milliseconds _loggingInterval;
		UdsLoggerOptions _udsOptions;
		std::thread _loggingThread;
		std::thread _callbackThread;
		mutable std::mutex _mutex;
		mutable std::mutex _callbackMutex;
		std::condition_variable _cond;
		std::condition_variable _callbackCond;
		bool _stopped;

		std::unique_ptr<LoggerImpl> _loggerImpl;

		std::deque<LogRecord> _loggedRecords;
		std::vector<LoggerCallback*> _callbacks;
	};

} // namespace logger
