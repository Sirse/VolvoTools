#pragma once

#include <common/CliSupport.hpp>
#include <common/CarPlatform.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace volvodiag {

// Stop flag set by the console handler.
inline std::atomic_bool& stopRequested = common::stopRequested;

enum class Command {
    None,
    ListDevices,
    Ident,
    Send,
    CanSend,
    CanRequest,
    CanPeriodic,
    CanReplay,
    Wake,
    Probe,
    Buses,
    Scan,
    Monitor,
    Session,
    TesterPresent,
    Reset,
    Routine,
    RoutineScan,
    DtcRead,
    DtcClear,
    DtcSnapshot,
    DtcExtended,
    DidRead,
    DidWrite,
    DidScan,
    ObdVin,
    ObdPid,
    ObdDtcRead,
    ObdDtcClear,
    Script,
    ReconCapture,
};

struct RawCanFilters {
    std::vector<uint32_t> ids;
    std::vector<std::pair<uint32_t, uint32_t>> ranges;
    std::vector<std::pair<uint32_t, uint32_t>> masks;
};

struct TxEchoFilter {
    bool enabled{false};
    std::vector<uint32_t> ids;
};

// Frame key: CAN ID only or full frame.
enum class BaselineKey {
    Id,
    Full,
};

struct ReplayFrame {
    uint64_t timeMs{0};
    uint32_t canId{0};
    std::vector<uint8_t> data;
};

struct RunOptions {
    Command command{Command::None};
    std::string deviceName;
    std::string tracePath;
    common::CarPlatform carPlatform{common::CarPlatform::P3};
    std::string platformName{"P3"};
    uint8_t ecuId{0x10};
    std::string busName;
    std::optional<uint32_t> baudrateOverride;
    std::vector<uint8_t> requestData;
    std::vector<uint8_t> expectedData;      // --expect prefix (uds-raw)
    std::optional<uint8_t> expectNrc;       // --expect-nrc (uds-raw)
    bool expectTimeout{false};              // --expect-timeout (uds-raw)
    uint32_t canId{0};
    size_t repeatCount{1};
    size_t timeoutMs{1000};
    size_t responseCount{1};
    size_t responseTimeoutMs{1000};
    size_t intervalMs{1000};
    size_t monitorDurationMs{0};
    bool preserveReplayTiming{true};
    std::string replayInputPath;
    std::string canOutputPath;
    uint32_t wakeCanId{0x7DF};
    std::vector<uint8_t> wakePayload{0x10, 0x82};
    std::vector<uint8_t> wakeHoldPayload{0x3E, 0x80};
    std::vector<uint8_t> wakeTeardownPayload{0x11, 0x81};
    size_t wakeBurstCount{10};
    size_t wakeGapMs{20};
    bool wakeHold{true};
    size_t wakeHoldMs{0}; // 0 means hold until Ctrl-C
    bool wakeTeardown{false};
    uint32_t probeFrom{0x700};
    uint32_t probeTo{0x7FF};
    uint32_t probeResponseOffset{0x8};
    bool probeUseResponseRange{false};
    std::pair<uint32_t, uint32_t> probeResponseRange{0, 0};
    size_t probeGapMs{0};
    bool probeReassemble{false};
    std::string probeOutputPath;
    std::string outputMode{"raw"};
    RawCanFilters rawFilters;
    TxEchoFilter txEchoFilter;
    size_t monitorCount{0};
    std::string monitorOutputPath;
    std::string baselineOutPath; // --baseline-record: record frame keys seen
    std::string baselinePath;    // --baseline-compare: print only frames absent from this file
    BaselineKey baselineKey{BaselineKey::Full};
    std::string scanOutputPath;
    std::string dtcOutputPath;
    std::string didOutputPath;
    std::string dtcClearOutputPath;
    bool allEcus{false};
    bool confirmDestructive{false}; // gate for --yes across reset/clear/write/routine
    uint8_t dtcStatusMask{0xFF};
    bool dtcConfirmedOnly{false};
    std::vector<uint32_t> dtcTargets; // raw 3-byte DTCs (low 24 bits) from --dtc
    bool dtcAllStored{false};         // query every DTC reported by 19 02 first
    uint8_t dtcRecordNumber{0xFF};
    std::string dtcRecordOutputPath;
    std::vector<uint16_t> didIds;
    uint16_t didWriteId{0};               // --id for did-write (UDS 2E)
    bool didWriteExtendedSession{false};  // enter extended session (10 03) before writing
    uint16_t didScanFrom{0x0000};
    uint16_t didScanTo{0xFFFF};
    size_t didScanGapMs{0};
    bool keepalive{false};
    std::string didScanOutputPath;
    uint8_t obdPid{0};
    std::string obdOutputPath;
    std::string identOutputPath;
    uint8_t sessionType{0x03}; // UDS DiagnosticSessionControl subfunction
    bool sessionSuppress{false};
    bool preludeWake{false};
    uint8_t preludeSessionType{0x00}; // 0 disables per-command DiagnosticSessionControl
    std::vector<uint8_t> preludeSecurityKey;
    bool testerPresentSuppress{false};
    size_t testerPresentIntervalMs{2000};
    size_t testerPresentCount{1}; // 0 means unlimited until Ctrl-C
    uint8_t resetType{0x01}; // UDS ECUReset subfunction (01 hard, 02 keyOffOn, 03 soft)
    bool resetFunctional{false}; // broadcast to functional 0x7DF instead of the ECU's id
    bool resetSuppress{false};   // set suppressPositiveResponse bit (0x80), don't wait for a reply
    uint16_t routineId{0};
    std::vector<uint8_t> routineData;
    bool routineSuppress{false};
    uint16_t routineScanFrom{0x0000};
    uint16_t routineScanTo{0xFFFF};
    uint8_t routineSubFunction{0x01}; // RoutineControl subfunction (01 start, 02 stop, 03 results)
    bool routineExtendedSession{false}; // enter + keep alive extended session (10 03) during scan
    size_t routineScanGapMs{0};
    std::string routineScanOutputPath;
    std::string scriptPath;
    std::string scriptOutputDir;
    std::vector<std::pair<std::string, std::string>> scriptVariables;
    bool scriptDryRun{false};
    std::optional<common::CarPlatform> scriptPlatformOverride;
    std::optional<uint8_t> scriptEcuOverride;
    uint32_t reconCanId{0};
    std::vector<uint8_t> reconMarker;
    uint32_t reconLoadAddress{0};
    uint32_t reconLoadLength{0};
    uint16_t reconRoutineId{0};
    std::vector<uint8_t> reconRoutineData;
    std::string reconStubPath;
    size_t reconCaptureCount{4};
    size_t reconCaptureTimeoutMs{1000};
    std::string reconOutputPath;
    std::string reconManifestPath;
    std::vector<uint8_t> reconReferenceBytes;
    uint32_t reconReferenceAddress{0};
    std::string reconStubSha256;
    std::vector<uint8_t> reconHealthSessionRequest;
    std::vector<uint8_t> reconHealthSessionExpect;
    std::vector<uint8_t> reconHealthDidRequest;
    std::vector<uint8_t> reconHealthDidExpect;
    uint32_t reconRequestCanId{0};
    uint32_t reconResponseCanId{0};
    uint8_t reconIsoTpPadding{0x00};
    bool reconIsoTpPadToEight{false};
    uint8_t reconIsoTpBlockSize{0};
    uint8_t reconIsoTpStmin{0};
    bool reconSmokeOnly{false};
    std::string reconRawProtocol{"can"};
};

} // namespace volvodiag
