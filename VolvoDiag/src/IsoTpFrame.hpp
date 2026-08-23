#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace volvodiag {

	// Builds one ISO 15765-2 single frame around a UDS payload (PCI byte = length).
	// padToEight controls the wire difference between the two historic call sites:
	// UDS-over-CAN requests are padded to a full DLC-8 frame, raw CAN probes send the
	// short frame as-is.
	inline std::vector<uint8_t> makeIsoTpSingleFrame(const std::vector<uint8_t>& udsPayload,
		bool padToEight)
	{
		if (udsPayload.empty() || udsPayload.size() > 7) {
			throw std::runtime_error("UDS single-frame payload must contain 1-7 bytes");
		}
		std::vector<uint8_t> frame;
		frame.reserve(8);
		frame.push_back(static_cast<uint8_t>(udsPayload.size()));
		frame.insert(frame.end(), udsPayload.cbegin(), udsPayload.cend());
		if (padToEight) {
			frame.resize(8, 0x00);
		}
		return frame;
	}

} // namespace volvodiag
