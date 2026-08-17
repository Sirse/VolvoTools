#include "common/CanMessagesTransceiver.hpp"

#include "common/protocols/D2Message.hpp"

#include <j2534/J2534Channel.hpp>

#include <stdexcept>

namespace common {

CanMessagesTransceiver::CanMessagesTransceiver(std::unique_ptr<j2534::J2534Channel> j2534Channel,
                                               unsigned long protocolID,
                                               unsigned long txFlags)
    : _j2534Channel{std::move(j2534Channel)}
    , _protocolID{protocolID}
    , _txFlags{txFlags}
    , _isReadEnabled{false}
    , _isShutdown{false}
    , _thread(&CanMessagesTransceiver::readThread, this)
{
}

CanMessagesTransceiver::~CanMessagesTransceiver()
{
    {
        std::unique_lock<std::mutex> lock{_mutex};
        _isShutdown = true;
        _cond.notify_all();
    }
    _thread.join();
}

void CanMessagesTransceiver::subscribe(uint8_t ecuId, ICanMessagesReceiver& receiver)
{
    std::unique_lock<std::mutex> lock{_mutex};
    _subscribers.insert(std::make_pair(ecuId, &receiver));
}

void CanMessagesTransceiver::unsubscribeAll(const ICanMessagesReceiver& receiver)
{
    std::unique_lock<std::mutex> lock{_mutex};
    for(auto it = _subscribers.begin(); it != _subscribers.end();) {
        if(it->second == &receiver) {
            it = _subscribers.erase(it);
        }
        else {
            ++it;
        }
    }
}

void CanMessagesTransceiver::sendMessage(const std::vector<uint8_t>& data)
{
    // This used to be a silent no-op: the body was commented out, so any caller got the
    // impression a frame was sent when nothing hit the bus. There is no CAN id to target and
    // the old CEMCanMessages type no longer exists, so a working implementation cannot be
    // guessed. Fail loudly instead of silently dropping frames.
    (void)data;
    throw std::logic_error("CanMessagesTransceiver::sendMessage is not implemented");
}

void CanMessagesTransceiver::runRead(bool enabled)
{
    std::unique_lock<std::mutex> lock{_mutex};
    _isReadEnabled = enabled;
    _cond.notify_all();
}

void CanMessagesTransceiver::readThread()
{
    for(;;) {
        {
            std::unique_lock<std::mutex> lock{_mutex};
            _cond.wait(lock, [&]() {
                return _isShutdown || _isReadEnabled;
            });
            if(_isShutdown)
                break;
        }
        std::vector<PASSTHRU_MSG> msgs{1};
        if(_j2534Channel->readMsgs(msgs) == STATUS_NOERROR) {
            processMessages(msgs);
        }
    }
}

void processD2Frame(ReceivedMessageMap& received, const SubscriberMap& subscribers,
                    const PASSTHRU_MSG& msg)
{
    if (msg.DataSize < 5) {
        return;
    }
    const uint8_t ecuType = D2Message::getECUType(msg.Data);
    const uint8_t packetType = msg.Data[4];
    // Bit 7 starts a new packet and overwrites the per-ECU buffer; bit 6 appends. Bitwise
    // tests - "packetType && 0x80" used to be just "packetType is non-zero", which made every
    // frame a packet start and the continuation branch unreachable.
    if ((packetType & 0x80) != 0) {
        received[ecuType] = { msg.Data + 5, msg.Data + msg.DataSize };
    }
    else if ((packetType & 0x40) != 0) {
        received[ecuType].insert(received[ecuType].end(), msg.Data + 5, msg.Data + msg.DataSize);
    }
    const auto range = subscribers.equal_range(ecuType);
    for (auto callback = range.first; callback != range.second; ++callback) {
        callback->second->onCanMessage(&msg.Data[4], msg.DataSize - 4);
    }
}

void CanMessagesTransceiver::processMessages(const std::vector<PASSTHRU_MSG>& msgs)
{
    for(auto it = msgs.cbegin(); it != msgs.cend(); ++it) {
        processD2Frame(_receivedMessages, _subscribers, *it);
    }
}

} // namespace common
