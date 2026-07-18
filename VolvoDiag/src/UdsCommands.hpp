#pragma once

#include "VolvoDiagOptions.hpp"

#include <common/DeviceInfo.hpp>

#include <vector>

namespace volvodiag {

void runBuses(const RunOptions& options);
void runSend(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runIdent(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runDidRead(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runDidWrite(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runDidScan(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runScan(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runSession(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runTesterPresent(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runReset(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runRoutine(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runRoutineScan(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runDtcRead(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runDtcClear(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runDtcSnapshot(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runDtcExtended(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runObdVin(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runObdPid(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runObdDtcRead(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runObdDtcClear(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runScript(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);

} // namespace volvodiag
