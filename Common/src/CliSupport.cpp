#include "common/CliSupport.hpp"

#include "common/RuntimeDiagnostics.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

namespace common {

std::atomic_bool stopRequested{false};

namespace {

// Set once by installConsoleCtrlHandler() at startup, before the handler can fire,
// and only read afterwards from the OS control-handler thread. That set-before-use
// ordering is the invariant that keeps this single-writer global race-free.
std::function<void()> stopCallback;

BOOL WINAPI HandlerRoutine(_In_ DWORD ctrlType)
{
    if (ctrlType == CTRL_C_EVENT || ctrlType == CTRL_BREAK_EVENT || ctrlType == CTRL_CLOSE_EVENT) {
        const bool alreadyRequested = stopRequested.exchange(true);
        if (stopCallback) {
            stopCallback();
        }
        return alreadyRequested ? FALSE : TRUE;
    }
    return FALSE;
}

} // namespace

bool isDebugLoggingRequested(int argc, const char* argv[])
{
    return std::any_of(argv + 1, argv + argc, [](const char* arg) {
        return std::string(arg) == "--debug";
    });
}

void requestStop()
{
    stopRequested.store(true);
}

void resetStopRequested()
{
    stopRequested.store(false);
}

bool waitForStopOrTimeout(std::chrono::milliseconds duration)
{
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
        if (stopRequested.load()) {
            return true;
        }
        const auto remaining = deadline - std::chrono::steady_clock::now();
        const auto sleepTime = std::max(std::chrono::milliseconds(1),
            std::min(std::chrono::milliseconds(100),
                std::chrono::duration_cast<std::chrono::milliseconds>(remaining)));
        std::this_thread::sleep_for(sleepTime);
    }
    return stopRequested.load();
}

void installConsoleCtrlHandler(const std::function<void()>& onStop)
{
    stopCallback = onStop;
    if (!SetConsoleCtrlHandler(HandlerRoutine, TRUE)) {
        throw std::runtime_error("Can't set console control handler");
    }
}

void printAvailableDevices(std::ostream& output, const std::vector<j2534::DeviceInfo>& devices)
{
    output << "Available J2534 devices (" << getProcessArchitecture()
           << " only):" << std::endl;
    for (const auto& device : devices) {
        output << "    " << device.deviceName << std::endl;
    }
    if (devices.empty()) {
        printJ2534ArchitectureHint(output);
    }
}

} // namespace common
