#pragma once

#include <cstdint>
#include <optional>

namespace common {

	// Online PIN search window per the P3Tool reference (method_54): the top two bytes stay
	// fixed at FFFF and only the low three bytes are scanned (2^24 candidates). Fixing the
	// prefix guarantees a universal collision inside the window and keeps the run bounded
	// (~hours) and resumable. The found value is a hex collision of the form FFFFxxxxxx, not
	// the "real" factory PIN from a CEM dump.
	constexpr uint64_t kCemPinWindowFloor = 0xFFFF000000ull;
	constexpr uint64_t kCemPinWindowCeil = 0xFFFFFFFFFFull;

	// Bounds of a PIN bruteforce scan. A unset bound means "unbounded" on that side, which
	// keeps the legacy free-run behaviour for expert manual scans.
	struct PinSearchWindow {
		std::optional<uint64_t> floorPin;
		std::optional<uint64_t> ceilPin;

		bool contains(uint64_t pin) const
		{
			return (!floorPin || pin >= *floorPin) && (!ceilPin || pin <= *ceilPin);
		}

		// Next candidate after `current` in the given direction; nullopt once the scan has
		// passed the window edge - that is the "everything searched, nothing found" point.
		// The edge value itself is still a valid candidate, so a downward scan visits the
		// floor and stops only when asked to step below it.
		std::optional<uint64_t> nextCandidate(uint64_t current, bool upward) const
		{
			if (upward) {
				if (ceilPin && current >= *ceilPin) {
					return {};
				}
				return current + 1;
			}
			if (floorPin && current <= *floorPin) {
				return {};
			}
			return current - 1;
		}
	};

} // namespace common
