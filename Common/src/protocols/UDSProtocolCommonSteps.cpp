#include "common/protocols/UDSProtocolCommonSteps.hpp"

#include "common/protocols/UDSRequest.hpp"
#include "common/protocols/UDSError.hpp"
#include "common/protocols/UploadIntegrity.hpp"
#include "common/Util.hpp"

#include <easylogging++.h>

#include <algorithm>
#include <chrono>
#include <set>
#include <thread>
#include <utility>

namespace common {

	namespace {

		uint32_t generateKeyImpl(uint32_t hash, uint32_t input)
		{
			for (size_t i = 0; i < 32; ++i)
			{
				const bool is_bit_set = (hash ^ input) & 1;
				input >>= 1;
				hash >>= 1;
				if (is_bit_set)
					hash = (hash | 0x800000) ^ 0x109028;
			}
			return hash;
		}

        std::string pinToHexString(const std::array<uint8_t, 5>& pin)
        {
            return toHexString({pin[0], pin[1], pin[2], pin[3], pin[4]});
        }

        // Reads the channel for a short window and reports whether a positive DiagnosticSessionControl
        // response (50 02) arrived. The periodic broadcast already queued the responses; this just drains
        // the buffer and confirms the module actually came up. A timeout or empty read is "no confirmation",
        // not an error - the caller decides what that means.
        bool sawProgrammingSessionResponse(const j2534::J2534Channel& channel)
        {
            bool sawPositive = false;
            try {
                channel.readMsgs([&sawPositive](const uint8_t* data, size_t dataSize) {
                    // 4-byte CAN id header + payload; positive 50 02 for request 10 02.
                    if (dataSize >= 6 && data[4] == 0x50 && data[5] == 0x02) {
                        sawPositive = true;
                        return false;
                    }
                    return true;
                }, 300);
            }
            catch (const std::exception& ex) {
                LOG(DEBUG) << "sawProgrammingSessionResponse read ended: " << ex.what();
            }
            return sawPositive;
        }

        bool rangesOverlap(uint32_t firstAddr, uint32_t firstSize, uint32_t secondAddr, uint32_t secondSize)
        {
            if (firstSize == 0 || secondSize == 0) {
                return false;
            }
            const uint64_t firstEnd = static_cast<uint64_t>(firstAddr) + firstSize;
            const uint64_t secondEnd = static_cast<uint64_t>(secondAddr) + secondSize;
            return firstAddr < secondEnd && secondAddr < firstEnd;
        }

        bool rangeContains(uint32_t outerAddr, uint32_t outerSize, uint32_t innerAddr, uint32_t innerSize)
        {
            if (outerSize == 0 || innerSize == 0) {
                return false;
            }
            const uint64_t outerEnd = static_cast<uint64_t>(outerAddr) + outerSize;
            const uint64_t innerEnd = static_cast<uint64_t>(innerAddr) + innerSize;
            return outerAddr <= innerAddr && innerEnd <= outerEnd;
        }

        bool eraseRange(const j2534::J2534Channel& channel, uint32_t canId, uint32_t startAddr,
                        uint32_t eraseLength, size_t retryCount)
        {
            const auto eraseAddr = toVector(startAddr);
            const auto eraseSize = toVector(eraseLength);
            UDSRequest eraseRoutineRequest{ canId, { 0x31, 0x01, 0xff, 0x00,
                                                     eraseAddr[0], eraseAddr[1], eraseAddr[2], eraseAddr[3],
                                                     eraseSize[0], eraseSize[1], eraseSize[2], eraseSize[3] } };
            for(size_t i = 0; i < retryCount; ++i) {
                try {
                    (void)eraseRoutineRequest.process(channel, { 0x01, 0xff, 0x00, 0x00 }, 1, 60000);
                    return true;
                }
                catch (const std::exception& ex) {
                    LOG(WARNING) << "eraseRange attempt " << std::dec << (i + 1) << " failed, addr=0x"
                                 << std::hex << startAddr << " length=0x" << eraseLength
                                 << " ex=" << ex.what();
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }
            }
            return false;
        }

        bool eraseBlockTouchesData(const EraseBlock& eraseBlock, const std::vector<VBFChunk>& chunks)
        {
            for (const auto& chunk: chunks) {
                if (rangesOverlap(eraseBlock.startAddr, eraseBlock.length, chunk.writeOffset,
                                  static_cast<uint32_t>(chunk.data.size()))) {
                    return true;
                }
            }
            return false;
        }

        bool chunkCoveredByEraseBlock(const std::vector<EraseBlock>& eraseBlocks, const VBFChunk& chunk)
        {
            for (const auto& eraseBlock: eraseBlocks) {
                if (rangeContains(eraseBlock.startAddr, eraseBlock.length, chunk.writeOffset,
                                  static_cast<uint32_t>(chunk.data.size()))) {
                    return true;
                }
            }
            return false;
        }

	}

	bool UDSProtocolCommonSteps::broadcastProgrammingSession(const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels)
	{
        LOG(INFO) << "broadcastProgrammingSession enter";
        if (channels.empty()) {
            LOG(ERROR) << "broadcastProgrammingSession failed: no open channels";
            return false;
        }
        std::vector<std::vector<unsigned long>> msgIds(channels.size());
		for (size_t i = 0; i < channels.size(); ++i) {
            // 5 ms used to be used here, but SDA itself logs "StartPeriodicSend with a
            // TimeInterval<10! Not supported" on DiCE and other adapters may reject it too.
			const auto ids = channels[i]->startPeriodicMsgs(UDSMessage(0x7DF, { 0x10, 0x02 }), 20);
			if (ids.empty()) {
                LOG(ERROR) << "broadcastProgrammingSession failed to start periodic message on channel " << i;
				return false;
			}
			msgIds[i] = ids;
		}
		std::this_thread::sleep_for(std::chrono::seconds(2));
		for (size_t i = 0; i < channels.size(); ++i) {
			channels[i]->stopPeriodicMsg(msgIds[i]);
		}
        // A periodic-message start alone used to count as success. The ECU must actually have
        // answered 50 02: a sleeping/off module answers nothing, and reporting success then
        // left the real failure to surface later as a mysterious authorize/transfer timeout.
        for (size_t i = 0; i < channels.size(); ++i) {
            if (sawProgrammingSessionResponse(*channels[i])) {
                LOG(INFO) << "broadcastProgrammingSession confirmed programming session on channel " << i;
                return true;
            }
        }
        LOG(ERROR) << "broadcastProgrammingSession no module confirmed programming session (50 02)";
        return false;
	}

	std::vector<unsigned long> UDSProtocolCommonSteps::keepAlive(const j2534::J2534Channel& channel)
	{
		return channel.startPeriodicMsgs(UDSMessage(0x7DF, { 0x3E, 0x80 }), 1900);
	}

	bool UDSProtocolCommonSteps::broadcastProgrammingSessionSilent(
		const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels, unsigned long durationMs)
	{
        LOG(INFO) << "broadcastProgrammingSessionSilent enter";
        if (channels.empty()) {
            LOG(ERROR) << "broadcastProgrammingSessionSilent failed: no open channels";
            return false;
        }
        std::vector<std::vector<unsigned long>> msgIds(channels.size());
        bool success = true;
        for (size_t i = 0; i < channels.size(); ++i) {
            msgIds[i] = channels[i]->startPeriodicMsgs(UDSMessage(0x7DF, { 0x10, 0x82 }), 20);
            if (msgIds[i].empty()) {
                LOG(ERROR) << "broadcastProgrammingSessionSilent failed to start periodic message on channel " << i;
                success = false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));
        for (size_t i = 0; i < channels.size(); ++i) {
            if (!msgIds[i].empty()) {
                channels[i]->stopPeriodicMsg(msgIds[i]);
            }
        }
        LOG(INFO) << "broadcastProgrammingSessionSilent exit";
        return success;
	}

	bool UDSProtocolCommonSteps::broadcastProgrammingSessionPrelude(
		const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels)
	{
        if (!broadcastProgrammingSessionSilent(channels)) {
            return false;
        }
        return broadcastProgrammingSession(channels);
	}

	void UDSProtocolCommonSteps::broadcastEcuReset(const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels)
	{
        LOG(INFO) << "broadcastEcuReset enter";
        for(const auto& resetSubfunction: {0x11, 0x81}) {
            std::vector<std::vector<unsigned long>> msgIds(channels.size());
            for (size_t i = 0; i < channels.size(); ++i) {
                const auto ids = channels[i]->startPeriodicMsgs(UDSMessage(0x7DF, { 0x11, static_cast<uint8_t>(resetSubfunction) }), 20);
                if (ids.empty()) {
                    LOG(ERROR) << "broadcastEcuReset error, failed to start periodic messages on channel = " << i;
                    return;
                }
                msgIds[i] = ids;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            for (size_t i = 0; i < channels.size(); ++i) {
                channels[i]->stopPeriodicMsg(msgIds[i]);
            }
        }
        LOG(INFO) << "broadcastEcuReset exit";
        return;
	}

	uint32_t UDSProtocolCommonSteps::generateSecurityKey(const std::array<uint8_t, 5>& pin_array,
		const std::array<uint8_t, 3>& seed_array)
	{
		const uint32_t high_part = pin_array[4] << 24 | pin_array[3] << 16 | pin_array[2] << 8 | pin_array[1];
		const uint32_t low_part = pin_array[0] << 24 | seed_array[2] << 16 | seed_array[1] << 8 | seed_array[0];
		unsigned int hash = 0xC541A9;
		hash = generateKeyImpl(hash, low_part);
		hash = generateKeyImpl(hash, high_part);
		uint32_t result = ((hash & 0xF00000) >> 12) | hash & 0xF000 | (uint8_t)(16 * hash)
			| ((hash & 0xFF0) << 12) | ((hash & 0xF0000) >> 16);
		return result;
	}

	AuthResult UDSProtocolCommonSteps::authorize(const j2534::J2534Channel& channel, uint32_t canId,
		const std::array<uint8_t, 5>& pin)
	{
        LOG(INFO) << "authorize enter pin=" << pinToHexString(pin);
        UDSRequest seedRequest(canId, { 0x27, 0x01 });
        try {
            channel.clearRx();
            const auto seedResponse(seedRequest.process(channel));
            if (seedResponse.size() < 9) {
                LOG(ERROR) << "authorize short seed response (" << std::dec << seedResponse.size()
                           << " bytes), pin=" << pinToHexString(pin);
                return AuthResult::TransientError;
            }
            std::array<uint8_t, 3> seed = { seedResponse[6], seedResponse[7], seedResponse[8] };
            uint32_t key = generateSecurityKey(pin, seed);
            channel.clearRx();
            UDSRequest keyRequest(canId, { 0x27, 0x02, (key >> 16) & 0xFF, (key >> 8) & 0xFF, key & 0xFF });
            try {
                const auto keyResponse(keyRequest.process(channel));
                if (keyResponse.size() >= 6 && keyResponse[5] == 0x02) {
                    LOG(INFO) << "authorize success, pin=" << pinToHexString(pin);
                    return AuthResult::Unlocked;
                }
                LOG(ERROR) << "authorize unexpected key response (" << toHexString(keyResponse)
                           << "), pin=" << pinToHexString(pin);
                return AuthResult::TransientError;
            }
            catch(const UDSError& error) {
                if (error.getErrorCode() == UDSError::ErrorCode::InvalidKey) {
                    // Definitive verdict: the ECU understood the request and rejected the key.
                    LOG(ERROR) << "authorize invalid key (7F 27 35), pin=" << pinToHexString(pin);
                    return AuthResult::WrongKey;
                }
                LOG(ERROR) << "authorize error: " << error.what() << ", pin = "
                           << pinToHexString(pin);
                return AuthResult::TransientError;
            }
        }
        catch (const std::exception& ex) {
            LOG(ERROR) << "authorize transient failure: " << ex.what()
                       << ", pin=" << pinToHexString(pin);
            return AuthResult::TransientError;
        }
        catch (...) {
            LOG(ERROR) << "authorize transient failure, pin=" << pinToHexString(pin);
            return AuthResult::TransientError;
        }
	}

	bool UDSProtocolCommonSteps::authorizeWithRetry(const j2534::J2534Channel& channel, uint32_t canId,
		const std::array<uint8_t, 5>& pin)
	{
		// Known-PIN unlock backoff: a couple of quick retries for flaky buses. Seconds-long
		// sleeps must stay out of SecurityAccess paths - this function runs inside flash/read
		// sessions where a stall is just a stall.
		constexpr size_t kMaxAttempts = 3;
		for (size_t attempt = 1; attempt <= kMaxAttempts; ++attempt) {
			switch (authorize(channel, canId, pin)) {
			case AuthResult::Unlocked:
				return true;
			case AuthResult::WrongKey:
				// A wrong known PIN stays wrong; no amount of retrying fixes it.
				return false;
			case AuthResult::TransientError:
				if (attempt < kMaxAttempts) {
					std::this_thread::sleep_for(std::chrono::milliseconds(200));
				}
				break;
			}
		}
		LOG(INFO) << "authorization failed, pin=" << pinToHexString(pin);
		return false;
	}

    bool finishTransfer(const j2534::J2534Channel& channel, uint32_t canId, uint16_t expectedCrc)
    {
        UDSRequest transferExitRequest{ canId, { 0x37 } };
        const auto response = transferExitRequest.process(channel, 10000);
        if (response.size() < 5 || response[4] != 0x77) {
            LOG(ERROR) << "TransferExit got unexpected response: " << toHexString(response);
            return false;
        }
        // The download contract requires the block CRC: 77 <crc_hi> <crc_lo>. A bare 77 was
        // previously accepted silently, which meant the write was never verified on an ECU that
        // omits the CRC. Missing CRC where the VBF block CRC profile expects one is an error.
        if (response.size() < 7) {
            LOG(ERROR) << "TransferExit missing block CRC (bare 77), expected=0x" << std::hex
                       << expectedCrc << " response=" << toHexString(response);
            return false;
        }
        const uint16_t returnedCrc = (static_cast<uint16_t>(response[5]) << 8) | response[6];
        if (returnedCrc != expectedCrc) {
            LOG(ERROR) << "TransferExit CRC mismatch, expected=0x" << std::hex << expectedCrc
                       << " actual=0x" << returnedCrc;
            return false;
        }
        return true;
    }

    bool UDSProtocolCommonSteps::transferChunk(const j2534::J2534Channel& channel, uint32_t canId, const VBFChunk& chunk,
                                              const std::function<void(size_t)>& progressCallback)
	{
        LOG(INFO) << "transferChunk enter chunk: " << std::hex << chunk.writeOffset;
        try {
            const auto startAddr = chunk.writeOffset;
            const auto dataSize = chunk.data.size();
            UDSRequest requestDownloadRequest{ canId, { 0x34, 0x00, 0x44,
                (startAddr >> 24) & 0xFF, (startAddr >> 16) & 0xFF, (startAddr >> 8) & 0xFF, startAddr & 0xFF,
                (dataSize >> 24) & 0xFF, (dataSize >> 16) & 0xFF, (dataSize >> 8) & 0xFF, dataSize & 0xFF } };
            const auto downloadResponse{ requestDownloadRequest.process(channel, { 0x20 }, 10) };
            if (downloadResponse.size() < 2) {
                return false;
            }
            const uint32_t maxNumberOfBlockLength = encodeBigEndian(downloadResponse[0], downloadResponse[1]);
            if (maxNumberOfBlockLength <= 2) {
                LOG(ERROR) << "RequestDownload invalid max block size: 0x" << std::hex << maxNumberOfBlockLength;
                return false;
            }
            const size_t maxSizeToTransfer = maxNumberOfBlockLength - 2;
            // The TransferData counter is one byte and rolls over at 0xFF: a chunk that
            // needs more blocks cannot be transferred in this session shape.
            const size_t blockCount = (chunk.data.size() + maxSizeToTransfer - 1) / maxSizeToTransfer;
            if (blockCount > 255) {
                LOG(ERROR) << "transferChunk refused: chunk of 0x" << std::hex << chunk.data.size()
                           << " bytes needs " << std::dec << blockCount
                           << " TransferData blocks, the 1-byte counter caps at 255";
                return false;
            }
            uint8_t chunkIndex = 1;
            for (size_t i = 0; i < chunk.data.size(); i += maxSizeToTransfer, ++chunkIndex) {
                const auto chunkEnd{ std::min(i + maxSizeToTransfer, chunk.data.size()) };
                LOG(INFO) << "transferChunk write chunk: {" << std::hex << i << ", " << chunkEnd <<"}";
                std::vector<uint8_t> data{ 0x36, chunkIndex };
                data.insert(data.end(), chunk.data.cbegin() + i, chunk.data.cbegin() + chunkEnd);
                UDSRequest transferDataRequest{ canId, std::move(data) };
                transferDataRequest.process(channel, { chunkIndex }, 10, 60000);
                progressCallback(chunkEnd - i);
            }
            LOG(INFO) << "transferChunk finish transfer, crc: {" << std::hex
                      << ((chunk.crc >> 8) & 0xFF) << ", " << (chunk.crc & 0xFF) <<"}";
            if (!finishTransfer(channel, canId, static_cast<uint16_t>(chunk.crc & 0xFFFF))) {
                return false;
            }
		}
        catch(const std::exception& ex) {
            LOG(ERROR) << "transferChunk error, ex = " << ex.what() << ", offset = " << std::hex << chunk.writeOffset;
            return false;
        }
        catch (...) {
            LOG(ERROR) << "transferChunk error: offset = " << std::hex << chunk.writeOffset;
            return false;
        }
        LOG(INFO) << "transferChunk completed, offset = " << std::hex << chunk.writeOffset;
		return true;
	}

    bool UDSProtocolCommonSteps::transferData(const j2534::J2534Channel& channel, uint32_t canId, const VBF& data,
                                              const std::function<void(size_t)>& progressCallback)
    {
        LOG(INFO) << "transferData enter";
        // Last line of defence: we have no unpacker, and writing packed payload as-is would
        // brick the ECU. Refuse here so nothing can reach an ECU by any other path.
        if (isPackedDataFormat(data.header)) {
            LOG(ERROR) << "transferData refused: VBF has data_format_identifier=0x" << std::hex
                       << static_cast<int>(data.header.dataFormatIdentifier)
                       << ", payload is packed and unpacking is not implemented";
            return false;
        }
        try {
            for (const auto& chunk : data.chunks) {
                if (!transferChunk(channel, canId, chunk, progressCallback)) {
                    return false;
                }
            }
        }
        catch(const std::exception& ex) {
            LOG(ERROR) << "transferData error, ex = " << ex.what();
            return false;
        }
        catch (...) {
            LOG(ERROR) << "transferData error";
            return false;
        }
        LOG(INFO) << "transferData completed";

        return true;
    }

    std::vector<uint8_t> finishUpload(const j2534::J2534Channel& channel, uint32_t canId)
    {
        UDSRequest transferExitRequest{ canId, { 0x37 } };
        const auto response = transferExitRequest.process(channel, 10000);
        if (response.size() < 5 || response[4] != 0x77) {
            LOG(ERROR) << "Upload TransferExit got unexpected response: " << toHexString(response);
            throw std::runtime_error("Upload TransferExit got unexpected response");
        }
        return response;
    }

    // Sends the TransferExit and returns the raw response; exceptions are logged and swallowed so
    // cleanup never masks the original error. Only used on error paths, where the read already
    // failed and the close is best-effort.
    std::vector<uint8_t> finishUploadBestEffort(const j2534::J2534Channel& channel, uint32_t canId)
    {
        try {
            return finishUpload(channel, canId);
        }
        catch (const std::exception& ex) {
            LOG(WARNING) << "Upload TransferExit best-effort cleanup failed: " << ex.what();
        }
        catch (...) {
            LOG(WARNING) << "Upload TransferExit best-effort cleanup failed";
        }
        return {};
    }

    UploadReadResult UDSProtocolCommonSteps::readDataByUpload(const j2534::J2534Channel& channel, uint32_t canId,
                                                              uint32_t startAddr, uint32_t dataSize,
                                                              const std::function<void(size_t)>& progressCallback)
    {
        UploadReadResult result;
        result.startAddress = startAddr;
        result.dataSize = dataSize;
        LOG(INFO) << "readDataByUpload enter addr=0x" << std::hex << startAddr
                  << " size=0x" << dataSize;
        if (dataSize == 0) {
            result.success = true;
            result.integrityStatus = IntegrityStatus::NotChecked;
            LOG(INFO) << "readDataByUpload completed empty range";
            return result;
        }
        bool uploadStarted = false;
        UploadIntegrityAccumulator accumulator;
        try {
            result.payload.clear();
            result.payload.reserve(dataSize);

            const std::vector<uint8_t> requestUploadPayload = {
                0x35, 0x00, 0x44,
                static_cast<uint8_t>((startAddr >> 24) & 0xFF),
                static_cast<uint8_t>((startAddr >> 16) & 0xFF),
                static_cast<uint8_t>((startAddr >> 8) & 0xFF),
                static_cast<uint8_t>(startAddr & 0xFF),
                static_cast<uint8_t>((dataSize >> 24) & 0xFF),
                static_cast<uint8_t>((dataSize >> 16) & 0xFF),
                static_cast<uint8_t>((dataSize >> 8) & 0xFF),
                static_cast<uint8_t>(dataSize & 0xFF),
            };
            UDSRequest requestUploadRequest{ canId, requestUploadPayload };
            const auto uploadResponse = requestUploadRequest.process(channel, 10000);
            if (uploadResponse.size() < 8 || uploadResponse[4] != 0x75) {
                LOG(ERROR) << "RequestUpload got unexpected response: " << toHexString(uploadResponse);
                return result;
            }
            uploadStarted = true;

            if (uploadResponse[5] != 0x20) {
                LOG(ERROR) << "RequestUpload unsupported length format: 0x"
                           << std::hex << static_cast<int>(uploadResponse[5]);
                finishUploadBestEffort(channel, canId);
                return result;
            }

            const size_t maxBlockSize = (static_cast<size_t>(uploadResponse[6]) << 8) | uploadResponse[7];
            if (maxBlockSize <= 2) {
                LOG(ERROR) << "RequestUpload invalid max block size: 0x" << std::hex << maxBlockSize;
                finishUploadBestEffort(channel, canId);
                return result;
            }
            const size_t maxPayloadSize = maxBlockSize - 2;
            LOG(INFO) << "readDataByUpload maxBlockSize=0x" << std::hex << maxBlockSize
                      << " maxPayloadSize=0x" << maxPayloadSize;

            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            uint8_t blockIndex = 1;
            while (result.payload.size() < dataSize) {
                UDSRequest transferDataRequest{ canId, { 0x36, blockIndex } };
                LOG(DEBUG) << "TransferData upload request block=0x" << std::hex
                          << static_cast<int>(blockIndex)
                          << " outputOffset=0x" << result.payload.size()
                          << " remaining=0x" << (dataSize - result.payload.size());
                const auto transferResponse = transferDataRequest.process(channel, { blockIndex }, 10, 60000);
                const size_t remaining = dataSize - result.payload.size();
                LOG(DEBUG) << "TransferData upload accepted block=0x" << std::hex
                          << static_cast<int>(blockIndex)
                          << " payloadAfterStrip=" << toHexString(transferResponse)
                          << " payloadSize=0x" << transferResponse.size()
                          << " outputOffset=0x" << result.payload.size();
                if (transferResponse.empty()) {
                    LOG(ERROR) << "TransferData upload returned empty payload after positive 0x76 block=0x"
                               << std::hex << static_cast<int>(blockIndex)
                               << "; empty transferResponseParameterRecord is not treated as flash zeros";
                    finishUploadBestEffort(channel, canId);
                    return result;
                }
                if (transferResponse.size() > maxPayloadSize) {
                    LOG(ERROR) << "TransferData upload block too large, block=0x"
                               << std::hex << static_cast<int>(blockIndex)
                               << " size=0x" << transferResponse.size()
                               << " max=0x" << maxPayloadSize;
                    finishUploadBestEffort(channel, canId);
                    return result;
                }
                if (transferResponse.size() > remaining) {
                    LOG(ERROR) << "TransferData upload returned more data than requested, block=0x"
                               << std::hex << static_cast<int>(blockIndex)
                               << " size=0x" << transferResponse.size()
                               << " remaining=0x" << remaining;
                    finishUploadBestEffort(channel, canId);
                    return result;
                }
                // Feed the full positive 0x76 response (SID + counter + payload) to the integrity
                // accumulator before the SID/counter are stripped, so the SDA wire CRC sees the
                // exact flattened stream the ECU covered. Streaming stays - the wire CRC cannot be
                // recomputed from the assembled image afterwards.
                {
                    std::vector<uint8_t> fullResponse{ 0x76, blockIndex };
                    fullResponse.insert(fullResponse.end(), transferResponse.cbegin(), transferResponse.cend());
                    accumulator.addResponse(fullResponse);
                }
                LOG(DEBUG) << "TransferData upload copy block=0x" << std::hex
                          << static_cast<int>(blockIndex)
                          << " bytes=0x" << transferResponse.size()
                          << " outputOffset=0x" << result.payload.size();
                result.payload.insert(result.payload.end(), transferResponse.cbegin(), transferResponse.cend());
                if (progressCallback) {
                    progressCallback(transferResponse.size());
                }
                ++blockIndex;
            }

            if (result.payload.size() != dataSize) {
                LOG(ERROR) << "readDataByUpload size mismatch expected=0x" << std::hex << dataSize
                           << " actual=0x" << result.payload.size();
                finishUploadBestEffort(channel, canId);
                return result;
            }

            // The full volume was collected and its size matched - the read itself succeeded.
            // Nothing after this point may flip success: the integrity verdict is diagnostic.
            result.success = true;
            result.blockCount = accumulator.blockCount();

            // The verdict: try the TransferExit, but a missing 77 is not a read failure either.
            try {
                result.transferExitResponse = finishUpload(channel, canId);
            }
            catch (const std::exception& ex) {
                LOG(WARNING) << "readDataByUpload TransferExit did not complete: " << ex.what();
            }
            catch (...) {
                LOG(WARNING) << "readDataByUpload TransferExit did not complete";
            }
            result.integrityStatus = IntegrityStatus::NoTransferExit;
            if (!result.transferExitResponse.empty()) {
                const auto integrity = resolveTransferExit(result.transferExitResponse, accumulator);
                result.integrityStatus = integrity.status;
                result.crcPresent = integrity.crcPresent;
                result.integrityProfileUsed = integrity.profileUsed;
                result.returnedCrc = integrity.returnedCrc;
                result.computedImageCrc = integrity.computedImageCrc;
                result.computedSdaWireCrc = integrity.computedSdaWireCrc;
                switch (integrity.status) {
                case IntegrityStatus::Ok:
                    LOG(INFO) << "readDataByUpload integrity ok, algorithm="
                              << uploadIntegrityProfileName(integrity.profileUsed);
                    break;
                case IntegrityStatus::NotChecked:
                    LOG(INFO) << "readDataByUpload integrity not checked (SBL returned no CRC)";
                    break;
                case IntegrityStatus::Unverified:
                    LOG(WARNING) << "Upload CRC matched no known algorithm; dump is saved, verify it "
                                    "manually. ecu_crc=0x" << std::hex << integrity.returnedCrc
                                 << " image_crc16=0x" << integrity.computedImageCrc
                                 << " sda47_wire_crc16=0x" << integrity.computedSdaWireCrc
                                 << " blocks=" << std::dec << accumulator.blockCount()
                                 << " size=0x" << std::hex << result.payload.size();
                    break;
                case IntegrityStatus::NoTransferExit:
                    LOG(WARNING) << "Upload returned no 77 TransferExit response; dump is saved, "
                                    "verify it manually";
                    break;
                }
            }
            uploadStarted = false;
        }
        catch(const std::exception& ex) {
            LOG(ERROR) << "readDataByUpload error, ex = " << ex.what()
                       << ", addr = 0x" << std::hex << startAddr;
            if (uploadStarted) {
                finishUploadBestEffort(channel, canId);
            }
            return result;
        }
        catch (...) {
            LOG(ERROR) << "readDataByUpload error, addr = 0x" << std::hex << startAddr;
            if (uploadStarted) {
                finishUploadBestEffort(channel, canId);
            }
            return result;
        }
        LOG(INFO) << "readDataByUpload completed addr=0x" << std::hex << startAddr
                  << " size=0x" << dataSize;
        return result;
    }

    bool UDSProtocolCommonSteps::eraseFlash(const j2534::J2534Channel& channel, uint32_t canId, const VBF& data)
    {
        LOG(INFO) << "eraseFlash enter";
        std::set<std::pair<uint32_t, uint32_t>> erasedBlocks;
        for (const auto& eraseBlock : data.header.eraseBlocks) {
            if (eraseBlock.length == 0 || !eraseBlockTouchesData(eraseBlock, data.chunks)) {
                continue;
            }
            const auto blockKey = std::make_pair(eraseBlock.startAddr, eraseBlock.length);
            if (!erasedBlocks.insert(blockKey).second) {
                continue;
            }
            LOG(INFO) << "eraseFlash erase block: " << std::hex << eraseBlock.startAddr
                      << ", length = " << eraseBlock.length;
            if (!eraseRange(channel, canId, eraseBlock.startAddr, eraseBlock.length, 10)) {
                LOG(ERROR) << "Failed to erase block: " << std::hex << eraseBlock.startAddr;
                return false;
            }
        }
        for (const auto& chunk : data.chunks) {
            if (chunk.data.empty() || chunkCoveredByEraseBlock(data.header.eraseBlocks, chunk)) {
                continue;
            }
            if (!eraseRange(channel, canId, chunk.writeOffset, static_cast<uint32_t>(chunk.data.size()), 10)) {
                LOG(ERROR) << "Failed to erase chunk: " << std::hex << chunk.writeOffset;
                return false;
            }
        }
        LOG(INFO) << "eraseFlash completed";
        return true;
	}

    bool UDSProtocolCommonSteps::eraseChunk(const j2534::J2534Channel& channel, uint32_t canId, const VBFChunk& chunk)
    {
        LOG(INFO) << "eraseChunk enter chunk: " << std::hex << chunk.writeOffset;
        if (chunk.data.empty()) {
            LOG(INFO) << "eraseChunk skipped empty chunk, offset = " << std::hex << chunk.writeOffset;
            return true;
        }
        if (eraseRange(channel, canId, chunk.writeOffset, static_cast<uint32_t>(chunk.data.size()), 1)) {
            LOG(INFO) << "eraseChunk completed, offset = " << std::hex << chunk.writeOffset;
            return true;
        }
        LOG(ERROR) << "Failed to erase chunk: " << std::hex << chunk.writeOffset;
        return false;
    }

    bool UDSProtocolCommonSteps::startRoutine(const j2534::J2534Channel& channel, uint32_t canId, uint32_t addr)
    {
        LOG(INFO) << "startRoutine enter, addr = " << std::hex << addr;
        const auto callAddr = common::toVector(addr);
        try {
            UDSRequest startRoutineRequest{ canId,
                { 0x31, 0x01, 0x03, 0x01, callAddr[0], callAddr[1], callAddr[2], callAddr[3] } };
            startRoutineRequest.process(channel, { 0x01, 0x03, 0x01 }, 10, 10000);
        }
        catch (const std::exception& ex) {
            LOG(ERROR) << "startRoutine failed, addr = " << std::hex << addr << " ex=" << ex.what();
            return false;
        }
        LOG(INFO) << "startRoutine completed, addr = " << std::hex << addr;
        return true;
    }

    bool UDSProtocolCommonSteps::checkProgrammingDependencies(const j2534::J2534Channel& channel, uint32_t canId,
                                                              uint32_t startAddr, uint32_t length)
    {
        LOG(INFO) << "checkProgrammingDependencies enter, addr=0x" << std::hex << startAddr
                  << " length=0x" << length;
        const auto addr = toVector(startAddr);
        const auto size = toVector(length);
        UDSRequest request{ canId, { 0x31, 0x01, 0xff, 0x01,
                                     addr[0], addr[1], addr[2], addr[3],
                                     size[0], size[1], size[2], size[3] } };
        try {
            (void)request.process(channel, { 0x01, 0xff, 0x01, 0x00 }, 1, 60000);
        }
        catch (const UDSError& ex) {
            // The ECU rejected the request outright, so it just doesn't implement the routine.
            // Don't fail an otherwise good flash over an optional check.
            LOG(WARNING) << "checkProgrammingDependencies not supported by ECU (nrc=0x" << std::hex
                         << static_cast<int>(ex.getErrorCode()) << "), skipping: " << ex.what();
            return true;
        }
        catch (const std::exception& ex) {
            LOG(ERROR) << "checkProgrammingDependencies reported a fault: " << ex.what();
            return false;
        }
        LOG(INFO) << "checkProgrammingDependencies completed";
        return true;
    }

    bool UDSProtocolCommonSteps::checkValidApplication(const j2534::J2534Channel& channel, uint32_t canId)
    {
        LOG(INFO) << "checkValidApplication enter";
        UDSRequest checkValidApplicationRequest{ canId, { 0x31, 0x01, 0x03, 0x04 } };
        try {
            checkValidApplicationRequest.process(channel);
        }
        catch(const std::exception& ex) {
            LOG(ERROR) << "checkValidApplication error, ex = " << ex.what();
            return false;
        }
        LOG(INFO) << "checkValidApplication finshed";
        return true;
    }

} // namespace common
