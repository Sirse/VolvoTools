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

// How the console Ctrl-handler reacts to a stop event.
enum class StopDecision {
	GracefulStop, // first request outside a critical region: request a cooperative stop
	Ignore,       // an uninterruptible region is active: refuse and keep running
	ForceKill     // repeated request outside a critical region: let the OS terminate
};

// Pure policy of the console Ctrl-handler, split out so it can be unit-tested.
StopDecision decideCtrlAction(bool alreadyRequested, bool uninterruptibleActive);

// While an uninterruptible region is active, console stop events are swallowed with a
// warning instead of being honored. This exists for flash phases whose interruption cannot
// be made safe: once erasing has begun, stopping mid-write leaves the ECU without a valid
// application, and hard-killing the process on top of that also skips driver/channel
// cleanup - so the operation must run to completion. Leaving the region also discards a
// stop request that raced into the flag just before it was entered.
void beginUninterruptibleRegion(const char* reason);
void endUninterruptibleRegion();
bool isUninterruptibleRegionActive();

// RAII form of begin/end for code paths with early exits and failures.
class UninterruptibleRegion {
public:
	explicit UninterruptibleRegion(const char* reason);
	~UninterruptibleRegion();
	UninterruptibleRegion(const UninterruptibleRegion&) = delete;
	UninterruptibleRegion& operator=(const UninterruptibleRegion&) = delete;
};

void installConsoleCtrlHandler(const std::function<void()>& onStop = {});

void printAvailableDevices(std::ostream& output, const std::vector<j2534::DeviceInfo>& devices);

} // namespace common
