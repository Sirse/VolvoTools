#pragma once

#include "VolvoDiagOptions.hpp"

#include <common/DeviceInfo.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <toml++/toml.h>

namespace volvodiag {

struct ScriptStep {
    std::string name;
    enum class Kind { Uds, DidRead, Session, TesterPresent, Delay } kind{Kind::Uds};
    std::vector<uint8_t> request;
    std::vector<uint16_t> dids;
    uint8_t sessionType{0x01};
    size_t delayMs{0};
    bool suppressResponse{false};
    bool expectPositive{false};
    std::vector<uint8_t> expectedPrefix;
    std::optional<uint8_t> expectNrc; // expect this negative response code
    bool expectTimeout{false};        // expect no response at all
};

struct ScriptScenario {
    int version{0};
    std::string name;
    common::CarPlatform platform{common::CarPlatform::P3};
    uint8_t ecuId{0x10};
    size_t timeoutMs{1000};
    bool wake{false};
    uint8_t preludeSession{0};
    std::string securityPinEnv;
    size_t testerPresentIntervalMs{0};
    bool testerPresentSuppress{true};
    std::vector<ScriptStep> steps;
};

ScriptScenario loadScriptScenario(const std::string& path,
                                  const RunOptions& options);
toml::table serializeScriptScenario(const ScriptScenario& scenario);
void runScriptScenario(const std::vector<j2534::DeviceInfo>& devices,
                       const RunOptions& options);

} // namespace volvodiag
