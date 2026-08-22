#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace common {

	// Outcome of a single SecurityAccess (27 01 / 27 02) attempt, per the P3Tool reference:
	// 67 02 means unlocked, 7F 27 35 (InvalidKey) rejects the candidate, anything else
	// (timeout, TX failure, another NRC) is a bus-level problem - not a verdict on the key.
	enum class AuthResult {
		Unlocked,
		WrongKey,
		TransientError
	};

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

	// Decision core of the PIN bruteforcer: maps one attempt outcome onto the next action.
	// Hardware-free by design (attempts come from the caller), so the dispatcher - including
	// the wrong-key vs bus-failure handling that must never silently skip a candidate - is
	// unit-testable without a J2534 device.
	class PinSearchDispatcher {
	public:
		enum class Action {
			Continue,           // candidate rejected, advance to the next one
			RetrySameCandidate, // transient failure, reinit the session and repeat the same PIN
			Found,              // the current candidate unlocked the ECU
			Exhausted,          // window scanned through, nothing found
			GiveUp              // too many consecutive transient failures, the bus is dead
		};

		PinSearchDispatcher(PinSearchWindow window, bool upward, uint64_t startPin,
			size_t maxConsecutiveTransientErrors = 3)
			: _window{ window }
			, _upward{ upward }
			, _currentPin{ startPin }
			, _maxConsecutiveTransientErrors{ maxConsecutiveTransientErrors }
		{
		}

		uint64_t currentPin() const
		{
			return _currentPin;
		}

		Action step(AuthResult result)
		{
			switch (result) {
			case AuthResult::Unlocked:
				return Action::Found;
			case AuthResult::WrongKey: {
				_consecutiveTransientErrors = 0;
				const auto next{ _window.nextCandidate(_currentPin, _upward) };
				if (!next) {
					return Action::Exhausted;
				}
				_currentPin = *next;
				return Action::Continue;
			}
			case AuthResult::TransientError:
				++_consecutiveTransientErrors;
				if (_consecutiveTransientErrors >= _maxConsecutiveTransientErrors) {
					return Action::GiveUp;
				}
				return Action::RetrySameCandidate;
			}
			return Action::GiveUp;
		}

	private:
		PinSearchWindow _window;
		bool _upward;
		uint64_t _currentPin;
		size_t _maxConsecutiveTransientErrors;
		size_t _consecutiveTransientErrors{ 0 };
	};

} // namespace common
