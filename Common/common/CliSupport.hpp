#pragma once

#include "DeviceInfo.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <iosfwd>
#include <vector>

namespace common {

extern std::atomic_bool stopRequested;

bool isDebugLoggingRequested(int argc, const char* argv[]);

void requestStop();
void resetStopRequested();
bool waitForStopOrTimeout(std::chrono::milliseconds duration);

void installConsoleCtrlHandler(const std::function<void()>& onStop = {});

void printAvailableDevices(std::ostream& output, const std::vector<j2534::DeviceInfo>& devices);

} // namespace common
