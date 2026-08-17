#pragma once

#include <j2534/J2534_v0404.h>

#include <condition_variable>
#include <map>
#include <mutex>
#include <vector>
#include <thread>

namespace j2534 {
class J2534Channel;
} // namespace j2534

namespace common {

class ICanMessagesReceiver {
public:
    virtual ~ICanMessagesReceiver() {}
    /**
     * @brief Called then fully completed message was received over CAN
     * @param data CAN message
     * @return If this function returns false then receiving of CAN messages is stopped.
     *         If you need to continue receiving of CAN messages then you should return true.
     */
    virtual bool onCanMessage(const uint8_t* buffer, size_t bufferSize) = 0;
};

// Per-ECU assembly state and subscriber map, shared by CanMessagesTransceiver and the pure
// frame-processing function. Exposed so the packet assembly logic can be unit-tested without a
// J2534 device.
using ReceivedMessageMap = std::map<uint8_t, std::vector<uint8_t>>;
using SubscriberMap = std::multimap<uint8_t, ICanMessagesReceiver*>;

// Processes one received D2 frame: updates the per-ECU assembly buffer (bit 7 of the packet
// type starts a new packet and overwrites the buffer; bit 6 appends to it) and dispatches the
// accumulated payload to every subscriber of that ECU. Pure function over the two maps, so the
// begin/continue masks and the subscriber dispatch are testable offline.
void processD2Frame(ReceivedMessageMap& received, const SubscriberMap& subscribers,
                    const PASSTHRU_MSG& msg);

/**
 * @brief This class is used for sending and receiving CAN messages with preprocessing.
 */
class CanMessagesTransceiver {
public:
    explicit CanMessagesTransceiver(std::unique_ptr<j2534::J2534Channel> j2534Channel,
                                    unsigned long protocolID,
                                    unsigned long txFlags
                                    );
    ~CanMessagesTransceiver();

    void subscribe(uint8_t, ICanMessagesReceiver& receiver);
    void unsubscribeAll(const ICanMessagesReceiver& receiver);

    void sendMessage(const std::vector<uint8_t>& data);
    void runRead(bool enabled);

private:
    void readThread();
    void processMessages(const std::vector<PASSTHRU_MSG>& msgs);

    std::unique_ptr<j2534::J2534Channel> _j2534Channel;
    unsigned long _protocolID;
    unsigned long _txFlags;
    std::mutex _mutex;
    std::condition_variable _cond;

    ReceivedMessageMap _receivedMessages;
    SubscriberMap _subscribers;

    bool _isReadEnabled;
    bool _isShutdown;
    std::thread _thread;
};

} // namespace common
