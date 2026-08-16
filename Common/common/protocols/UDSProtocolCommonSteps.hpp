#pragma once

#include "common/VBF.hpp"

#include <j2534/J2534.hpp>
#include <j2534/J2534Channel.hpp>

#include <array>
#include <functional>
#include <memory>
#include <vector>

namespace common {

	class UDSProtocolCommonSteps {
	public:
		static std::vector<std::unique_ptr<j2534::J2534Channel>> openChannels(
			j2534::J2534& j2534, unsigned long baudrate, uint32_t canId);
		// These three used to be called fallAsleep / broadcastProgrammingMode / wakeUp. The old
		// names came from the D2 steps, where they are accurate (D2 really sends FF 86 "go to
		// sleep" and FF C8 "wake up"). Over UDS the wire content is something else entirely, and
		// reading "fallAsleep" as "the ECU is now asleep" has already produced a wrong diagnosis
		// and a proposed fix that would have reset the module mid-session. Named after the frame
		// they actually put on the bus.

		// Functional DiagnosticSessionControl -> programmingSession, 10 02 on 0x7DF, repeated for
		// 2 s. Positive responses are requested, so every module on the bus answers.
		static bool broadcastProgrammingSession(const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels);
		// Functional TesterPresent, 3E 80 on 0x7DF, periodic. Returns the periodic message ids.
		static std::vector<unsigned long> keepAlive(const j2534::J2534Channel& channel);
		// Same request as broadcastProgrammingSession but 10 82: suppressPositiveResponse is set,
		// so nothing answers. This is the form to use for a functional broadcast.
		static bool broadcastProgrammingSessionSilent(const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels,
			unsigned long durationMs = 180);
		// Functional ECUReset, 11 11 then 11 81 on 0x7DF. This is the teardown step that puts the
		// modules back to normal operation - it resets them, it does not wake anything up.
		// Subfunction 0x11 is not an ISO 14229 one; treat it as Volvo-specific and unverified.
		static void broadcastEcuReset(const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels);
		static bool authorize(const j2534::J2534Channel& channel, uint32_t canId, const std::array<uint8_t, 5>& pin);
        static bool transferData(const j2534::J2534Channel& channel, uint32_t canId, const VBF& data,
                                 const std::function<void(size_t)>& progressCallback);
        static bool transferChunk(const j2534::J2534Channel& channel, uint32_t canId, const VBFChunk& chunk,
                                 const std::function<void(size_t)>& progressCallback);
        static bool readDataByUpload(const j2534::J2534Channel& channel, uint32_t canId, uint32_t startAddr,
                                     uint32_t dataSize, std::vector<uint8_t>& output,
                                     const std::function<void(size_t)>& progressCallback);
        static bool eraseFlash(const j2534::J2534Channel& channel, uint32_t canId, const VBF& data);
        static bool eraseChunk(const j2534::J2534Channel& channel, uint32_t canId, const VBFChunk& chunk);
        static bool startRoutine(const j2534::J2534Channel& channel, uint32_t canId, uint32_t addr);
        // Routine 31 01 FF 01: asks the ECU whether the software it now holds is self-consistent.
        // Returns false only when the ECU ran the routine and reported a fault. An ECU that does
        // not implement it answers with an NRC, which counts as success - this is an extra
        // verification step and not every ECU supports it.
        static bool checkProgrammingDependencies(const j2534::J2534Channel& channel, uint32_t canId,
                                                 uint32_t startAddr, uint32_t length);
        static bool checkValidApplication(const j2534::J2534Channel& channel, uint32_t canId);
	};

} // namespace common
