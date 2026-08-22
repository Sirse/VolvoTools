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

// Uninterruptible region state. The reason pointer is published (release) before the
// active flag flips on; the handler reads it only after an acquire-load of that flag.
std::atomic<const char*> uninterruptibleReason{ nullptr };
std::atomic_bool uninterruptibleActive{ false };

BOOL WINAPI HandlerRoutine(_In_ DWORD ctrlType)
{
    if (ctrlType == CTRL_C_EVENT || ctrlType == CTRL_BREAK_EVENT || ctrlType == CTRL_CLOSE_EVENT) {
        switch (decideCtrlAction(stopRequested.load(), isUninterruptibleRegionActive())) {
        case StopDecision::Ignore: {
            // Destructive phase in progress (e.g. flash erase/write): refuse and say why,
            // on every press, so the user is not left guessing whether the key works.
            const auto* reason = uninterruptibleReason.load(std::memory_order_acquire);
            std::cerr << "\nInterrupt ignored: " << (reason && *reason ? reason : "critical operation")
                      << " is in progress and will run to completion." << std::endl;
            return TRUE;
        }
        case StopDecision::GracefulStop:
            stopRequested.store(true);
            if (stopCallback) {
                stopCallback();
            }
            return TRUE;
        case StopDecision::ForceKill:
        default:
            // The historical escape hatch outside critical regions: a repeated request
            // hands control back to the OS to terminate the process immediately. Never
            // reached while an uninterruptible region is active.
            stopRequested.store(true);
            if (stopCallback) {
                stopCallback();
            }
            return FALSE;
        }
    }
    return FALSE;
}

} // namespace

StopDecision decideCtrlAction(bool alreadyRequested, bool uninterruptibleActive)
{
    if (uninterruptibleActive) {
        return StopDecision::Ignore;
    }
    return alreadyRequested ? StopDecision::ForceKill : StopDecision::GracefulStop;
}

void beginUninterruptibleRegion(const char* reason)
{
    uninterruptibleReason.store(reason ? reason : "", std::memory_order_release);
    uninterruptibleActive.store(true, std::memory_order_release);
}

void endUninterruptibleRegion()
{
    uninterruptibleActive.store(false, std::memory_order_release);
    // Discard a stop request that raced into the flag just before the region began:
    // nothing inside the region honors it, and a stale flag would leak into later phases.
    stopRequested.store(false);
}

bool isUninterruptibleRegionActive()
{
    return uninterruptibleActive.load(std::memory_order_acquire);
}

UninterruptibleRegion::UninterruptibleRegion(const char* reason)
{
    beginUninterruptibleRegion(reason);
}

UninterruptibleRegion::~UninterruptibleRegion()
{
    endUninterruptibleRegion();
}

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
