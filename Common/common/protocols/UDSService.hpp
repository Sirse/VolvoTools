#pragma once

#include <cstdint>

namespace common {

namespace uds {

enum class ServiceId : uint8_t {
    DiagnosticSessionControl = 0x10,
    ECUReset = 0x11,
    ClearDiagnosticInformation = 0x14,
    ReadDTCInformation = 0x19,
    ReadDataByIdentifier = 0x22,
    WriteDataByIdentifier = 0x2E,
    RoutineControl = 0x31,
    TesterPresent = 0x3E,
};

enum class PositiveResponseId : uint8_t {
    DiagnosticSessionControl = 0x50,
    ECUReset = 0x51,
    ClearDiagnosticInformation = 0x54,
    ReadDTCInformation = 0x59,
    ReadDataByIdentifier = 0x62,
    WriteDataByIdentifier = 0x6E,
    RoutineControl = 0x71,
    TesterPresent = 0x7E,
};

enum class DiagnosticSessionType : uint8_t {
    Default = 0x01,
    Programming = 0x02,
    Extended = 0x03,
};

enum class EcuResetType : uint8_t {
    HardReset = 0x01,
    KeyOffOnReset = 0x02,
    SoftReset = 0x03,
};

enum class RoutineControlSubFunction : uint8_t {
    StartRoutine = 0x01,
    StopRoutine = 0x02,
    RequestRoutineResults = 0x03,
};

enum class ReadDTCSubFunction : uint8_t {
    ReportDTCByStatusMask = 0x02,
    ReportDTCSnapshotRecordByDTCNumber = 0x04,
    ReportDTCExtendedDataRecordByDTCNumber = 0x06,
};

enum DtcStatusMask : uint8_t {
    TestFailed = 0x01,
    TestFailedThisCycle = 0x02,
    Pending = 0x04,
    Confirmed = 0x08,
    TestNotCompletedSinceClear = 0x10,
    TestFailedSinceClear = 0x20,
    TestNotCompletedThisCycle = 0x40,
    WarningIndicator = 0x80,
};

} // namespace uds

namespace obd {

constexpr uint32_t FunctionalRequestCanId = 0x7DF;

enum class ServiceId : uint8_t {
    ShowCurrentData = 0x01,
    ShowStoredDTCs = 0x03,
    ClearDTCs = 0x04,
    RequestVehicleInformation = 0x09,
};

enum class PositiveResponseId : uint8_t {
    ShowCurrentData = 0x41,
    ShowStoredDTCs = 0x43,
    ClearDTCs = 0x44,
    RequestVehicleInformation = 0x49,
};

enum class VehicleInformationPid : uint8_t {
    Vin = 0x02,
};

} // namespace obd

} // namespace common
