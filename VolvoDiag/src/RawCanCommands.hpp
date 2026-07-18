#pragma once

#include "VolvoDiagOptions.hpp"

#include <common/DeviceInfo.hpp>

#include <vector>

namespace volvodiag {

void runCanSend(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runCanRequest(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runCanPeriodic(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runCanReplay(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runWake(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runProbe(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);
void runMonitor(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options);

} // namespace volvodiag
