#include "FileLogWriter.hpp"
#include "LoggerApplication.hpp"

#include <logger/LogParameters.hpp>
#include <logger/Logger.hpp>
#include <logger/LoggerCallback.hpp>
#include <common/CliSupport.hpp>
#include <common/J2534ChannelProvider.hpp>
#include <common/RuntimeDiagnostics.hpp>
#include <common/Util.hpp>
#include <common/DeviceInfo.hpp>
#include <j2534/J2534.hpp>

#include <argparse/argparse.hpp>

#include <easylogging++.h>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

INITIALIZE_EASYLOGGINGPP

class ConsoleLogWriter final : public logger::LoggerCallback {
public:
  explicit ConsoleLogWriter(size_t printLimit) : _printLimit{printLimit} {}

  void onLogMessage(std::chrono::milliseconds timePoint,
                    const std::vector<double> &values) override {
    std::cout << std::fixed << std::setprecision(2) << (timePoint.count() / 1000.0) << ",";

    for (size_t i = 0; i < _printLimit && i < values.size(); ++i) {
      std::cout << std::fixed << std::setprecision(2) << values[i] << ",";
    }
    std::cout << std::endl;
  }

  void onStatusChanged(bool /*started*/) override {}

private:
  const size_t _printLimit;
};

std::string makeStartMessage(const j2534::DeviceInfo& device,
                             const std::string& platformName,
                             uint8_t cmId,
                             std::chrono::milliseconds loggingInterval,
                             std::optional<uint32_t> baudrateOverride,
                             logger::UdsLoggerOptions udsOptions) {
  std::stringstream ss;
  ss << "Starting logger: device=\"" << device.deviceName
     << "\" platform=" << platformName
     << " ecu=0x" << std::hex << static_cast<unsigned>(cmId)
     << std::dec << " interval-ms=" << loggingInterval.count();
  if (baudrateOverride.has_value()) {
    ss << " baudrate-override=" << *baudrateOverride;
  } else {
    ss << " baudrate=config";
  }
  ss << " uds-did-base=0x" << std::hex << udsOptions.didBase
     << std::dec << " uds-did-max-data-size=" << udsOptions.didMaxDataSize;
  return ss.str();
}

static bool getRunOptions(int argc, const char *argv[], std::string &deviceName,
                   std::optional<uint32_t> &baudrateOverride, std::string &paramsFilePath,
                   std::string &outputPath, unsigned &printCount, common::CarPlatform& carPlatform,
                   uint8_t& cmId, std::chrono::milliseconds& loggingInterval,
                   bool& listDevices, std::string& platformName,
                   logger::UdsLoggerOptions& udsOptions, bool& argsError) {
  argparse::ArgumentParser program("VolvoLogger");
  program.add_argument("--debug").default_value(false).implicit_value(true).nargs(0)
      .help("Enable verbose debug logging");
  program.add_argument("--list-devices").default_value(false).implicit_value(true).nargs(0)
      .help("List available J2534 devices and exit");
  program.add_argument("-d", "--device").default_value(std::string{}).help("Device name");
  program.add_argument("-b", "--baudrate").scan<'u', unsigned>().help("Override configured CAN bus speed");
  program.add_argument("-v", "--variables").help("Path to memory variables");
  program.add_argument("-o", "--output").help("Path to save logs");
  program.add_argument("-p", "--print").scan<'u', unsigned>().default_value(5u).help("Number of variables which prints to console");
  program.add_argument("-f", "--platform").default_value(std::string{"p3"}).help("Car's platform, supported values: p3, p3_y413, p3_y283_iam, p3_y283_icm, p3_p313_icm, p3_p313_iam, p3_y555_iam, p3_y555_icm, p3_y312h_iam, p3_y312h_icm");
  program.add_argument("-e", "--ecu").scan<'x', uint8_t>().default_value(uint8_t(0x10)).help("ECU id to log (P3 default: ECM 0x10)");
  program.add_argument("--interval-ms").scan<'u', unsigned>().default_value(50u).help("Logging interval in milliseconds");
  program.add_argument("--uds-did-base").scan<'x', uint16_t>().default_value(uint16_t(0xF200)).help("Base dynamic DID for UDS logger");
  program.add_argument("--uds-did-max-data-size").scan<'u', unsigned>().default_value(7u).help("Max bytes per dynamic DID for UDS logger");

  try {
      program.parse_args(argc, argv);
      deviceName = program.get<std::string>("-d");
      listDevices = program.is_used("--list-devices");
      if (listDevices) {
        return true;
      }
      baudrateOverride = program.is_used("-b")
          ? std::optional<uint32_t>{program.get<unsigned>("-b")}
          : std::nullopt;
      if (!program.is_used("-v")) {
        throw std::runtime_error("Missing required argument: -v/--variables");
      }
      if (!program.is_used("-o")) {
        throw std::runtime_error("Missing required argument: -o/--output");
      }
      paramsFilePath = program.get<std::string>("-v");
      if (!std::filesystem::is_regular_file(paramsFilePath)) {
        throw std::runtime_error("Variables file does not exist: " + paramsFilePath);
      }
      outputPath = program.get<std::string>("-o");
      if (outputPath.empty()) {
        throw std::runtime_error("Log output path is empty");
      }
      printCount = program.get<unsigned>("-p");
      platformName = program.get<std::string>("-f");
      carPlatform = common::parseCarPlatform(platformName);
      cmId = program.get<uint8_t>("-e");
      const auto intervalMs = program.get<unsigned>("--interval-ms");
      if (intervalMs == 0) {
        throw std::runtime_error("--interval-ms must be greater than 0");
      }
      loggingInterval = std::chrono::milliseconds(intervalMs);
      udsOptions.didBase = program.get<uint16_t>("--uds-did-base");
      const auto didMaxDataSize = program.get<unsigned>("--uds-did-max-data-size");
      if (didMaxDataSize == 0) {
        throw std::runtime_error("--uds-did-max-data-size must be greater than 0");
      }
      udsOptions.didMaxDataSize = didMaxDataSize;
      return true;
  }
  catch (const std::exception& err) {
      std::cerr << err.what() << std::endl;
      std::cerr << program;
      // Distinguish a parse failure from "no arguments": scripts must see a nonzero
      // exit code.
      argsError = true;
  }
  return false;
}

std::string getSehModuleName(void* address)
{
  MEMORY_BASIC_INFORMATION info{};
  if (address == nullptr || VirtualQuery(address, &info, sizeof(info)) == 0 || info.AllocationBase == nullptr) {
    return "<unknown>";
  }
  char modulePath[MAX_PATH]{};
  if (GetModuleFileNameA(reinterpret_cast<HMODULE>(info.AllocationBase), modulePath, MAX_PATH) == 0) {
    return "<unknown>";
  }
  return modulePath;
}

LONG WINAPI SehLoggingFilter(EXCEPTION_POINTERS* ep) {
  const auto* record = ep ? ep->ExceptionRecord : nullptr;
  const auto code = record ? record->ExceptionCode : 0;
  void* address = record ? record->ExceptionAddress : nullptr;
  LOG(ERROR) << "Unhandled SEH 0x" << std::hex << code
    << " at 0x" << reinterpret_cast<uintptr_t>(address)
    << " module=" << getSehModuleName(address);
  if (record && code == EXCEPTION_ACCESS_VIOLATION && record->NumberParameters >= 2) {
    LOG(ERROR) << "Access violation "
      << (record->ExceptionInformation[0] ? "write" : "read")
      << " address=0x" << std::hex << record->ExceptionInformation[1];
  }
  return EXCEPTION_EXECUTE_HANDLER;
}

int main(int argc, const char *argv[]) {
  common::initLogger("VolvoLogger.log", common::isDebugLoggingRequested(argc, argv));
  common::printRuntimeDiagnostics("VolvoLogger");
  SetUnhandledExceptionFilter(SehLoggingFilter);
  common::installConsoleCtrlHandler([] {
    logger::LoggerApplication::instance().stop();
  });
  std::optional<uint32_t> baudrateOverride;
  std::string deviceName;
  std::string paramsFilePath;
  std::string outputPath;
  std::string platformName;
  common::CarPlatform carPlatform;
  uint8_t cmId;
  unsigned printCount;
  std::chrono::milliseconds loggingInterval{50};
  bool listDevices = false;
  logger::UdsLoggerOptions udsOptions;
  bool argsError = false;
  const auto devices = common::getAvailableDevices();
  const bool optionsOk = getRunOptions(argc, argv, deviceName, baudrateOverride, paramsFilePath,
                    outputPath, printCount, carPlatform, cmId, loggingInterval,
                    listDevices, platformName, udsOptions, argsError);
  if (argsError) {
    return 2;
  }
  if (optionsOk) {
    if (listDevices) {
      common::printAvailableDevices(std::cout, devices);
      return 0;
    }
    j2534::DeviceInfo device;
    try {
      device = common::selectSingleDevice(devices, deviceName);
    } catch (const std::exception &ex) {
      LOG(WARNING) << ex.what();
      std::cout << ex.what() << std::endl;
      if (devices.empty()) {
        common::printJ2534ArchitectureHint(std::cout);
      }
      return 1;
    }
    {
      try {
        std::unique_ptr<j2534::J2534> j2534{
            std::make_unique<j2534::J2534>(device.libraryName)};
        std::string name =
            device.deviceName.find("DiCE-") != std::string::npos
                ? device.deviceName
                : "";
        j2534->PassThruOpen(name);
        logger::LogParameters params{paramsFilePath};
        logger::FileLogWriter fileLogWriter(outputPath, params);
        ConsoleLogWriter consoleLogWriter{printCount};
        const auto startMessage = makeStartMessage(device, platformName, cmId, loggingInterval, baudrateOverride, udsOptions);
        LOG(INFO) << startMessage;
        std::cout << startMessage << std::endl;
        logger::LoggerApplication::instance().start(
            baudrateOverride, loggingInterval, udsOptions, *j2534, params, carPlatform, cmId,
            {&fileLogWriter, &consoleLogWriter});
        while (!common::stopRequested.load() && logger::LoggerApplication::instance().isStarted()) {
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        logger::LoggerApplication::instance().stop();
      } catch (const std::exception &ex) {
        LOG(ERROR) << "Logger command failed: " << ex.what();
        std::cout << ex.what() << std::endl;
        return 1;
      } catch (const char *ex) {
        LOG(ERROR) << "Logger command failed: " << ex;
        std::cout << ex << std::endl;
        return 1;
      } catch (...) {
        LOG(ERROR) << "Logger command failed with unknown exception";
        std::cout << "exception" << std::endl;
        return 1;
      }
    }
  }
  return 0;
}
