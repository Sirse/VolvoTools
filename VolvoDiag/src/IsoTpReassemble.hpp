#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace volvodiag {

	// Outcome of feeding one received frame into a multi-frame reassembly in progress.
	enum class ConsecutiveFrameStatus {
		Appended,   // CF PCI matched the expected sequence number; its data was appended
		Ignored,    // not a consecutive frame (PCI high nibble != 2 or empty); frame skipped
		SequenceGap // CF sequence number mismatch; nothing was appended
	};

	// One ISO 15765-2 consecutive-frame step of response reassembly. `data` points at the
	// frame payload after the CAN-id header, so data[0] is the ONLY candidate PCI byte and
	// everything past it is verbatim data that is never inspected - a data byte that merely
	// looks like a PCI (e.g. ASCII space 0x20) must survive reassembly untouched.
	// nextSequence holds the rolling SN expected for this frame ((previous + 1) & 0x0F,
	// starting from the first frame's SN); it advances only on Appended.
	inline ConsecutiveFrameStatus appendConsecutiveFrame(std::vector<uint8_t>& payload,
		unsigned& nextSequence, const uint8_t* data, size_t size, size_t maxLength)
	{
		if (data == nullptr || size < 1) {
			return ConsecutiveFrameStatus::Ignored;
		}
		if ((data[0] & 0xF0) != 0x20) {
			return ConsecutiveFrameStatus::Ignored;
		}
		const auto sequence = data[0] & 0x0Fu;
		if (sequence != nextSequence) {
			return ConsecutiveFrameStatus::SequenceGap;
		}
		const size_t available = size - 1;
		const size_t room = payload.size() >= maxLength ? 0 : maxLength - payload.size();
		const size_t count = std::min(available, room);
		payload.insert(payload.end(), data + 1, data + 1 + count);
		nextSequence = static_cast<unsigned>((sequence + 1) & 0x0F);
		return ConsecutiveFrameStatus::Appended;
	}

} // namespace volvodiag
