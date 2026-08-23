#pragma once

#include <common/J2534ChannelProvider.hpp>
#include <common/protocols/PinSearch.hpp>

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <optional>

namespace common {

class UDSPinFinder {
public:
    enum class State {
        Initial,
        FallAsleep,
        KeepAlive,
        Work,
        WakeUp,
        Done,
        Error
    };

    enum class Direction {
        Up,
        Down
    };

    explicit UDSPinFinder(j2534::J2534& j2534, CarPlatform carPlatform, uint8_t ecuId,
        const std::function<void(State, uint64_t)> stateCallback,
        Direction direction = Direction::Up, uint64_t startPin = 0,
        PinSearchWindow window = {});
    ~UDSPinFinder();

    State getCurrentState() const;
    std::optional<uint64_t> getFoundPin() const;

    bool start();
    void stop();

private:
    const J2534ChannelProvider _channelProvider;
    std::thread _thread;
    std::unique_ptr<struct FinderData> _data;
    // The worker thread creates the impl after the (slow) channel open; these guard its
    // publication and carry a stop request made before that, so it can never be lost.
    mutable std::mutex _implMutex;
    std::unique_ptr<class UDSPinFinderImpl> _impl;
    std::atomic<bool> _stopRequested{ false };
    std::atomic<bool> _started{ false };
};

} // namespace common
