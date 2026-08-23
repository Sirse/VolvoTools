#include "ScriptModel.hpp"

#include "DiagContext.hpp"
#include "ExitCodes.hpp"
#include "OutputFormat.hpp"
#include "RawCanCommands.hpp"
#include "UdsCommands.hpp"

#include <common/Util.hpp>
#include <common/protocols/UDSRequest.hpp>
#include <common/protocols/UDSProtocolCommonSteps.hpp>
#include <common/J2534ChannelProvider.hpp>
#include <toml++/toml.h>

#include <chrono>
#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace volvodiag {
namespace {

using common::udsPayload;

using Table = toml::table;

[[noreturn]] void schemaError(const std::string& path, const std::string& message)
{
    throw std::runtime_error("Scenario " + path + ": " + message);
}

// Substitutes ${name} markers inside one string value and records any marker that no
// variable defines.
void substituteMarkersInString(std::string& text,
    const std::vector<std::pair<std::string, std::string>>& variables,
    std::vector<std::string>& unresolved, const std::string& path)
{
    for (const auto& [name, value] : variables) {
        const std::string marker = "${" + name + "}";
        size_t position = 0;
        while ((position = text.find(marker, position)) != std::string::npos) {
            text.replace(position, marker.size(), value);
            position += value.size();
        }
    }
    size_t position = text.find("${");
    while (position != std::string::npos) {
        const auto end = text.find('}', position);
        if (end == std::string::npos) {
            break;
        }
        unresolved.push_back(path + ": " + text.substr(position, end - position + 1));
        position = text.find("${", end);
    }
}

// Walks the parsed TOML tree and substitutes variables into string values only. Values
// can therefore never inject structure: whatever they contain stays a string. A marker
// that survives every substitution is reported instead of running with placeholder data.
void substituteScriptVariables(toml::node& node,
    const std::vector<std::pair<std::string, std::string>>& variables,
    std::vector<std::string>& unresolved, const std::string& path)
{
    node.visit([&](auto&& n) {
        using NodeType = std::decay_t<decltype(n)>;
        if constexpr (std::is_same_v<NodeType, toml::table>) {
            for (auto& [key, child] : n) {
                substituteScriptVariables(child, variables, unresolved, path + "." + std::string(key));
            }
        }
        else if constexpr (std::is_same_v<NodeType, toml::array>) {
            for (size_t i = 0; i < n.size(); ++i) {
                substituteScriptVariables(n[i], variables, unresolved,
                    path + "[" + std::to_string(i) + "]");
            }
        }
        else if constexpr (std::is_same_v<NodeType, toml::value<std::string>>) {
            substituteMarkersInString(n.get(), variables, unresolved, path);
        }
    });
}

void rejectUnknown(const Table& table, const std::string& path,
                   std::initializer_list<std::string_view> allowed)
{
    for (const auto& [key, _] : table) {
        bool known = false;
        for (const auto name : allowed) {
            known = known || key == name;
        }
        if (!known) {
            schemaError(path, "unknown field '" + std::string(key) + "'");
        }
    }
}

std::string requiredString(const Table& table, std::string_view key, const std::string& path)
{
    const auto value = table[key].value<std::string>();
    if (!value) schemaError(path, "field '" + std::string(key) + "' must be a string");
    return *value;
}

size_t optionalSize(const Table& table, std::string_view key, size_t fallback,
                    const std::string& path)
{
    if (!table.contains(key)) return fallback;
    const auto value = table[key].value<int64_t>();
    if (!value || *value < 0) schemaError(path, "field '" + std::string(key) + "' must be non-negative");
    return static_cast<size_t>(*value);
}

std::vector<uint8_t> parseBytes(const std::string& value, const std::string& path)
{
    try {
        const auto bytes = common::parseHexBytes(value);
        if (bytes.empty()) schemaError(path, "hex payload must not be empty");
        return bytes;
    }
    catch (const std::exception& ex) {
        schemaError(path, ex.what());
    }
}

uint8_t parseSession(const std::string& value, const std::string& path)
{
    const auto lower = common::toLower(value);
    if (lower == "default") return 0x01;
    if (lower == "programming") return 0x02;
    if (lower == "extended" || lower == "ext") return 0x03;
    const auto numeric = common::parseHexU32(value);
    if (numeric == 0 || numeric > 0x7F) schemaError(path, "invalid session type");
    return static_cast<uint8_t>(numeric);
}

ScriptStep parseStep(const toml::node& node, size_t index)
{
    const auto* table = node.as_table();
    if (!table) schemaError("steps[" + std::to_string(index) + "]", "must be a table");
    const auto path = "steps[" + std::to_string(index) + "]";
    rejectUnknown(*table, path, {"name", "uds", "did_read", "session", "tester_present", "delay_ms", "expect"});
    ScriptStep step;
    step.name = table->contains("name") ? requiredString(*table, "name", path) : path;
    size_t kinds = 0;
    if (table->contains("uds")) {
        step.kind = ScriptStep::Kind::Uds;
        step.request = parseBytes(requiredString(*table, "uds", path), path + ".uds");
        ++kinds;
    }
    if (table->contains("did_read")) {
        step.kind = ScriptStep::Kind::DidRead;
        const auto* array = table->get("did_read")->as_array();
        if (!array || array->empty()) schemaError(path, "did_read must be a non-empty array");
        for (const auto& item : *array) {
            const auto value = item.value<std::string>();
            if (!value) schemaError(path, "did_read entries must be strings");
            const auto did = common::parseHexU32(*value);
            if (did > 0xFFFF) schemaError(path, "DID must fit in two bytes");
            step.dids.push_back(static_cast<uint16_t>(did));
        }
        ++kinds;
    }
    if (table->contains("session")) {
        step.kind = ScriptStep::Kind::Session;
        step.sessionType = parseSession(requiredString(*table, "session", path), path + ".session");
        ++kinds;
    }
    if (table->contains("tester_present")) {
        step.kind = ScriptStep::Kind::TesterPresent;
        const auto* value = table->get("tester_present")->as_boolean();
        if (!value) schemaError(path, "tester_present must be boolean");
        step.suppressResponse = !value->get();
        ++kinds;
    }
    if (table->contains("delay_ms")) {
        step.kind = ScriptStep::Kind::Delay;
        step.delayMs = optionalSize(*table, "delay_ms", 0, path);
        ++kinds;
    }
    if (kinds != 1) schemaError(path, "exactly one step operation is required");
    if (const auto* expect = table->get("expect")) {
        if (step.kind != ScriptStep::Kind::Uds) {
            schemaError(path, "expect is only supported for uds steps");
        }
        const auto* expectTable = expect->as_table();
        if (!expectTable) schemaError(path + ".expect", "must be a table");
        rejectUnknown(*expectTable, path + ".expect", {"positive", "prefix", "nrc", "timeout"});
        if (const auto value = (*expectTable)["positive"].value<bool>()) step.expectPositive = *value;
        if (expectTable->contains("prefix")) {
            step.expectedPrefix = parseBytes(requiredString(*expectTable, "prefix", path + ".expect"), path + ".expect.prefix");
        }
        if (const auto value = (*expectTable)["timeout"].value<bool>()) step.expectTimeout = *value;
        if (expectTable->contains("nrc")) {
            const auto nrc = common::parseHexU32(requiredString(*expectTable, "nrc", path + ".expect"));
            if (nrc == 0 || nrc > 0xFF) schemaError(path + ".expect.nrc", "NRC must be a single byte (0x01-0xFF)");
            step.expectNrc = static_cast<uint8_t>(nrc);
        }
        // NRC and timeout cannot be combined with other expectations.
        if (step.expectTimeout && (step.expectNrc || step.expectPositive || !step.expectedPrefix.empty())) {
            schemaError(path + ".expect", "timeout cannot be combined with nrc/positive/prefix");
        }
        if (step.expectNrc && (step.expectPositive || !step.expectedPrefix.empty())) {
            schemaError(path + ".expect", "nrc cannot be combined with positive/prefix");
        }
    }
    return step;
}

const char* platformName(common::CarPlatform platform)
{
    switch (platform) {
    case common::CarPlatform::P3: return "P3";
    case common::CarPlatform::P3_Y413: return "P3_Y413";
    case common::CarPlatform::P3_Y283_IAM: return "P3_Y283_IAM";
    case common::CarPlatform::P3_Y283_ICM: return "P3_Y283_ICM";
    case common::CarPlatform::P3_P313_ICM: return "P3_P313_ICM";
    case common::CarPlatform::P3_P313_IAM: return "P3_P313_IAM";
    case common::CarPlatform::P3_Y555_IAM: return "P3_Y555_IAM";
    case common::CarPlatform::P3_Y555_ICM: return "P3_Y555_ICM";
    case common::CarPlatform::P3_Y312H_IAM: return "P3_Y312H_IAM";
    case common::CarPlatform::P3_Y312H_ICM: return "P3_Y312H_ICM";
    default: return "Undefined";
    }
}

std::ofstream openArtifact(const std::filesystem::path& path, const std::string& label)
{
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open()) {
        throw std::runtime_error("Failed to open " + label + ": " + path.string());
    }
    return output;
}

toml::table resolvedScenarioTable(const ScriptScenario& scenario)
{
    toml::table root;
    root.insert("version", scenario.version);
    root.insert("name", scenario.name);
    root.insert("platform", platformName(scenario.platform));
    root.insert("ecu", common::formatHexBytesLower({scenario.ecuId}));
    root.insert("timeout_ms", static_cast<int64_t>(scenario.timeoutMs));

    toml::table prelude;
    if (scenario.wake) prelude.insert("wake", true);
    if (scenario.preludeSession != 0) {
        prelude.insert("session", common::formatHexBytesLower({scenario.preludeSession}));
    }
    if (!scenario.securityPinEnv.empty()) {
        toml::table security;
        security.insert("pin_env", scenario.securityPinEnv);
        prelude.insert("security", std::move(security));
    }
    if (scenario.testerPresentIntervalMs > 0) {
        toml::table testerPresent;
        testerPresent.insert("interval_ms", static_cast<int64_t>(scenario.testerPresentIntervalMs));
        testerPresent.insert("suppress_response", scenario.testerPresentSuppress);
        prelude.insert("tester_present", std::move(testerPresent));
    }
    if (!prelude.empty()) root.insert("prelude", std::move(prelude));

    toml::array steps;
    for (const auto& step : scenario.steps) {
        toml::table item;
        item.insert("name", step.name);
        switch (step.kind) {
        case ScriptStep::Kind::Uds:
            item.insert("uds", common::formatHexBytesLower(step.request));
            break;
        case ScriptStep::Kind::DidRead: {
            toml::array dids;
            for (const auto did : step.dids) {
                std::ostringstream didText;
                didText << std::hex << std::setfill('0') << std::setw(4) << did;
                dids.push_back(didText.str());
            }
            item.insert("did_read", std::move(dids));
            break;
        }
        case ScriptStep::Kind::Session:
            item.insert("session", common::formatHexBytesLower({step.sessionType}));
            break;
        case ScriptStep::Kind::TesterPresent:
            item.insert("tester_present", !step.suppressResponse);
            break;
        case ScriptStep::Kind::Delay:
            item.insert("delay_ms", static_cast<int64_t>(step.delayMs));
            break;
        }
        if (step.kind == ScriptStep::Kind::Uds
            && (step.expectPositive || !step.expectedPrefix.empty()
                || step.expectNrc || step.expectTimeout)) {
            toml::table expect;
            if (step.expectPositive) expect.insert("positive", true);
            if (!step.expectedPrefix.empty()) {
                expect.insert("prefix", common::formatHexBytesLower(step.expectedPrefix));
            }
            if (step.expectNrc) {
                expect.insert("nrc", common::formatHexBytesLower({*step.expectNrc}));
            }
            if (step.expectTimeout) expect.insert("timeout", true);
            item.insert("expect", std::move(expect));
        }
        steps.push_back(std::move(item));
    }
    root.insert("steps", std::move(steps));
    return root;
}

void writeSummary(std::ostream& output, const ScriptScenario& scenario,
                  const std::string& status, size_t completed,
                  size_t failedStep, const std::string& message,
                  std::chrono::milliseconds duration)
{
    toml::table summary;
    summary.insert("name", scenario.name);
    summary.insert("status", status);
    summary.insert("platform", platformName(scenario.platform));
    summary.insert("ecu", common::formatHexBytesLower({scenario.ecuId}));
    summary.insert("completed_steps", static_cast<int64_t>(completed));
    summary.insert("duration_ms", static_cast<int64_t>(duration.count()));
    if (failedStep != 0) summary.insert("failed_step", static_cast<int64_t>(failedStep));
    if (!message.empty()) summary.insert("message", message);
    output << summary << '\n';
    if (!output) throw std::runtime_error("Failed to write scenario summary");
}

} // namespace

bool isDestructiveUdsService(const std::vector<uint8_t>& request)
{
    if (request.empty()) {
        return false;
    }
    switch (request[0]) {
    case 0x11: // ECUReset
    case 0x14: // ClearDiagnosticInformation
    case 0x2E: // WriteDataByIdentifier
    case 0x31: // RoutineControl
    case 0x34: // RequestDownload
    case 0x36: // TransferData (download direction)
        return true;
    default:
        return false;
    }
}

void ensureDestructiveStepsConfirmed(const ScriptScenario& scenario, bool confirmDestructive)
{
    if (confirmDestructive) {
        return;
    }
    for (size_t i = 0; i < scenario.steps.size(); ++i) {
        const auto& step = scenario.steps[i];
        if (step.kind != ScriptStep::Kind::Uds || !isDestructiveUdsService(step.request)) {
            continue;
        }
        throw DiagError(ExitCode::UsageError,
            "Scenario '" + scenario.name + "' step " + std::to_string(i + 1) + " ('" + step.name
            + "') uses destructive UDS service 0x" + common::formatHexBytesLower({step.request[0]})
            + "; re-run with --yes to confirm");
    }
}

toml::table serializeScriptScenario(const ScriptScenario& scenario)
{
    return resolvedScenarioTable(scenario);
}

ScriptScenario loadScriptScenario(const std::string& path, const RunOptions& options)
{
    Table root;
    try {
        std::ifstream input(path);
        if (!input) throw std::runtime_error("Failed to open scenario file");
        std::string source((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        root = toml::parse(source, path);
    }
    catch (const toml::parse_error& ex) {
        throw std::runtime_error("Failed to parse TOML scenario '" + path + "': " + std::string(ex.description()));
    }
    // Substitution happens on the parsed tree, not on the raw text: a variable value with
    // quotes or newlines cannot inject TOML structure, and an unresolved ${marker} is an
    // error instead of silently running the scenario with placeholder bytes.
    std::vector<std::string> unresolved;
    substituteScriptVariables(root, options.scriptVariables, unresolved, "root");
    if (!unresolved.empty()) {
        std::string joined;
        for (const auto& item : unresolved) {
            joined += "\n  - " + item;
        }
        throw std::runtime_error("Unresolved scenario variable marker(s):" + joined
            + "\nDefine them with --var name=value or remove the markers");
    }
    rejectUnknown(root, "root", {"version", "name", "platform", "ecu", "timeout_ms", "steps", "prelude"});
    ScriptScenario scenario;
    const auto version = root["version"].value<int64_t>();
    if (!version || *version != 1) schemaError("root", "version must be 1");
    scenario.version = 1;
    scenario.name = root.contains("name") ? requiredString(root, "name", "root") : std::filesystem::path(path).stem().string();
    if (root.contains("platform")) scenario.platform = common::parseCarPlatform(requiredString(root, "platform", "root"));
    if (root.contains("ecu")) {
        const auto value = root["ecu"].value<std::string>();
        if (value) {
            const auto parsed = common::parseHexU32(*value);
            if (parsed > 0xFF) schemaError("root.ecu", "must fit in one byte (0x00-0xFF)");
            scenario.ecuId = static_cast<uint8_t>(parsed);
        }
        else if (const auto numeric = root["ecu"].value<int64_t>()) {
            if (*numeric < 0 || *numeric > 0xFF) {
                schemaError("root.ecu", "must fit in one byte (0x00-0xFF)");
            }
            scenario.ecuId = static_cast<uint8_t>(*numeric);
        }
        else schemaError("root", "ecu must be a hex string or integer");
    }
    scenario.timeoutMs = optionalSize(root, "timeout_ms", 1000, "root");
    if (const auto* prelude = root["prelude"].as_table()) {
        rejectUnknown(*prelude, "root.prelude", {"wake", "session", "security", "tester_present"});
        if (const auto value = (*prelude)["wake"].value<bool>()) scenario.wake = *value;
        if (prelude->contains("session")) scenario.preludeSession = parseSession(requiredString(*prelude, "session", "root.prelude"), "root.prelude.session");
        if (const auto* security = prelude->get("security")) {
            const auto* securityTable = security->as_table();
            if (!securityTable) schemaError("root.prelude.security", "must be a table");
            rejectUnknown(*securityTable, "root.prelude.security", {"pin_env"});
            scenario.securityPinEnv = requiredString(*securityTable, "pin_env", "root.prelude.security");
        }
        if (const auto* tester = prelude->get("tester_present")) {
            const auto* testerTable = tester->as_table();
            if (!testerTable) schemaError("root.prelude.tester_present", "must be a table");
            rejectUnknown(*testerTable, "root.prelude.tester_present", {"interval_ms", "suppress_response"});
            scenario.testerPresentIntervalMs = optionalSize(*testerTable, "interval_ms", 2000, "root.prelude.tester_present");
            if (const auto value = (*testerTable)["suppress_response"].value<bool>()) scenario.testerPresentSuppress = *value;
        }
    }
    const auto* steps = root["steps"].as_array();
    if (!steps || steps->empty()) schemaError("root.steps", "must be a non-empty array");
    for (size_t i = 0; i < steps->size(); ++i) scenario.steps.push_back(parseStep((*steps)[i], i));
    if (options.scriptPlatformOverride) scenario.platform = *options.scriptPlatformOverride;
    if (options.scriptEcuOverride) scenario.ecuId = *options.scriptEcuOverride;
    return scenario;
}

void runScriptScenario(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    auto scenario = loadScriptScenario(options.scriptPath, options);
    if (options.scriptDryRun) {
        std::cout << "Scenario: " << scenario.name << " (" << scenario.steps.size() << " steps)\n";
        for (size_t i = 0; i < scenario.steps.size(); ++i) std::cout << i + 1 << ". " << scenario.steps[i].name << '\n';
        return;
    }
    // Fail fast on a missing --yes before touching the device or the bus: the gate covers
    // every step whose request targets a persistent-change UDS service.
    ensureDestructiveStepsConfirmed(scenario, options.confirmDestructive);
    const auto outputDir = options.scriptOutputDir.empty() ? std::filesystem::path(".") : std::filesystem::path(options.scriptOutputDir);
    std::filesystem::create_directories(outputDir);
    auto summary = openArtifact(outputDir / "summary.toml", "scenario summary");
    const auto started = std::chrono::steady_clock::now();
    size_t completed = 0;
    size_t currentStep = 0;
    try {
        auto resolved = openArtifact(outputDir / "scenario.resolved.toml", "resolved scenario");
        resolved << serializeScriptScenario(scenario) << '\n';
        if (!resolved) throw std::runtime_error("Failed to write resolved scenario");
        auto steps = openArtifact(outputDir / "steps.csv", "scenario step log");
        steps << "step,name,outcome,message\n";
        auto didOutput = openArtifact(outputDir / "did.csv", "scenario DID output");
        didOutput << "step,did,response\n";
        if (!steps || !didOutput) throw std::runtime_error("Failed to initialize scenario CSV artifacts");
        const auto device = common::selectSingleDevice(devices, options.deviceName);
        if (scenario.wake) {
            auto wakeOptions = options;
            wakeOptions.carPlatform = scenario.platform;
            wakeOptions.ecuId = scenario.ecuId;
            wakeOptions.preludeWake = true;
            wakeOptions.wakeHold = false;
            runWake({device}, wakeOptions);
        }
        auto j2534 = openDevice(device);
        common::J2534ChannelProvider provider{*j2534, scenario.platform, options.baudrateOverride};
        auto channel = provider.getChannelForEcu(scenario.ecuId);
        const auto canId = ecuCanId(scenario.platform, scenario.ecuId);
        if (scenario.preludeSession != 0) {
            ensurePayloadPrefix(udsPayload(processUds(*channel, canId,
                {0x10, scenario.preludeSession}, scenario.timeoutMs)),
                {0x50, scenario.preludeSession});
        }
        if (!scenario.securityPinEnv.empty()) {
            const auto* pin = std::getenv(scenario.securityPinEnv.c_str());
            if (!pin) throw std::runtime_error("Security PIN environment variable is not set: " + scenario.securityPinEnv);
            const auto bytes = common::parseHexBytes(pin);
            if (bytes.size() != 5) throw std::runtime_error("Security PIN must contain exactly 5 bytes");
            std::array<uint8_t, 5> pinArray{};
            std::copy(bytes.begin(), bytes.end(), pinArray.begin());
            if (!common::UDSProtocolCommonSteps::authorizeWithRetry(*channel, canId, pinArray)) throw std::runtime_error("SecurityAccess authorization failed");
        }
        auto sendTesterPresent = [&]() {
            if (scenario.testerPresentSuppress) {
                sendUdsNoWait(*channel, canId, {0x3E, 0x80}, scenario.timeoutMs);
            } else {
                ensurePayloadPrefix(udsPayload(processUds(*channel, canId,
                    {0x3E, 0x00}, scenario.timeoutMs)), {0x7E, 0x00});
            }
        };
        auto nextTesterPresent = std::chrono::steady_clock::now();
        if (scenario.testerPresentIntervalMs > 0) {
            sendTesterPresent();
            nextTesterPresent = std::chrono::steady_clock::now()
                + std::chrono::milliseconds(scenario.testerPresentIntervalMs);
        }
        auto serviceTesterPresentIfDue = [&]() {
            if (scenario.testerPresentIntervalMs == 0) return;
            if (std::chrono::steady_clock::now() >= nextTesterPresent) {
                try {
                    sendTesterPresent();
                }
                catch (const std::exception& ex) {
                    throw std::runtime_error("TesterPresent keep-alive failed: " + std::string(ex.what()));
                }
                nextTesterPresent = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(scenario.testerPresentIntervalMs);
            }
        };
        for (const auto& step : scenario.steps) {
            currentStep = completed + 1;
            if (stopRequested.load()) throw std::runtime_error("Scenario interrupted");
            try {
                serviceTesterPresentIfDue();
                if (step.kind == ScriptStep::Kind::Delay) {
                    auto remaining = std::chrono::milliseconds(step.delayMs);
                    while (remaining.count() > 0) {
                        if (stopRequested.load()) throw std::runtime_error("Scenario interrupted");
                        const auto slice = std::min(remaining, std::chrono::milliseconds(50));
                        std::this_thread::sleep_for(slice);
                        remaining -= slice;
                        serviceTesterPresentIfDue();
                    }
                } else if (step.kind == ScriptStep::Kind::Uds) {
                    UdsExpectation expect;
                    expect.prefix = step.expectedPrefix;
                    expect.positive = step.expectPositive;
                    expect.nrc = step.expectNrc;
                    expect.timeout = step.expectTimeout;
                    processUdsExpecting(*channel, canId, step.request, scenario.timeoutMs, expect);
                } else if (step.kind == ScriptStep::Kind::Session) {
                    const auto response = udsPayload(processUds(*channel, canId,
                        {0x10, step.sessionType}, scenario.timeoutMs));
                    ensurePayloadPrefix(response, {0x50, step.sessionType});
                } else if (step.kind == ScriptStep::Kind::TesterPresent) {
                    if (step.suppressResponse) sendUdsNoWait(*channel, canId, {0x3E, 0x80}, scenario.timeoutMs);
                    else ensurePayloadPrefix(udsPayload(processUds(*channel, canId,
                        {0x3E, 0x00}, scenario.timeoutMs)), {0x7E, 0x00});
                } else {
                    for (const auto did : step.dids) {
                        serviceTesterPresentIfDue();
                        const auto response = udsPayload(processUds(*channel, canId,
                            {0x22, static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did)},
                            scenario.timeoutMs));
                        didOutput << csvEscape(step.name) << ','
                                  << common::formatHexBytesLower({static_cast<uint8_t>(did >> 8), static_cast<uint8_t>(did)})
                                  << ',' << csvEscape(common::formatHexBytesLower(response)) << '\n';
                        if (!didOutput) throw std::runtime_error("Failed to write scenario DID output");
                    }
                }
                ++completed;
                steps << completed << ',' << csvEscape(step.name) << ",ok,\n";
                if (!steps) throw std::runtime_error("Failed to write scenario step log");
            }
            catch (const std::exception& ex) {
                steps << currentStep << ',' << csvEscape(step.name) << ",error," << csvEscape(ex.what()) << '\n';
                const std::string message = "Scenario step " + std::to_string(currentStep)
                    + " ('" + step.name + "') failed: " + ex.what();
                throw DiagError(static_cast<ExitCode>(classifyExitCode(ex)), message);
            }
        }
    }
    catch (const std::exception& ex) {
        writeSummary(summary, scenario, stopRequested.load() ? "interrupted" : "failed",
            completed, currentStep, ex.what(),
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started));
        throw;
    }
    writeSummary(summary, scenario, "passed", completed, 0, {},
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started));
}

void runScript(const std::vector<j2534::DeviceInfo>& devices, const RunOptions& options)
{
    runScriptScenario(devices, options);
}

} // namespace volvodiag
