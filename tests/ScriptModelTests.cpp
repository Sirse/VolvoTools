#include "ScriptModel.hpp"
#include "CliParser.hpp"
#include "CommandRisk.hpp"
#include "ReconCapture.hpp"
#include "DiagContext.hpp"
#include "ExitCodes.hpp"
#include "OutputFormat.hpp"

#include <common/Util.hpp>
#include <common/protocols/UDSError.hpp>
#include <common/protocols/UDSRequest.hpp>

#include <gtest/gtest.h>
#include <toml++/toml.h>

#include <filesystem>
#include <array>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <initializer_list>

namespace {

using volvodiag::RunOptions;

bool parseArgs(std::initializer_list<const char*> arguments, RunOptions& options)
{
    std::vector<const char*> argv{"VolvoDiag"};
    argv.insert(argv.end(), arguments.begin(), arguments.end());
    return volvodiag::parseOptions(static_cast<int>(argv.size()), argv.data(), options);
}

std::filesystem::path writeScenario(const std::string& name, const std::string& text)
{
    const auto path = std::filesystem::temp_directory_path() / ("volvodiag_" + name + ".toml");
    std::ofstream output(path, std::ios::trunc);
    EXPECT_TRUE(output.is_open());
    output << text;
    return path;
}

volvodiag::ScriptScenario load(const std::string& name, const std::string& text,
                               const RunOptions& options = {})
{
    const auto path = writeScenario(name, text);
    return volvodiag::loadScriptScenario(path.string(), options);
}

constexpr const char* kMinimal = R"(
version = 1
steps = [{ uds = "22 D1 00" }]
)";

} // namespace

TEST(ScriptModel, ParsesTomlAndAppliesDefaults)
{
    const auto scenario = load("defaults", kMinimal);
    EXPECT_EQ(scenario.version, 1);
    EXPECT_EQ(scenario.name, "volvodiag_defaults");
    EXPECT_EQ(scenario.platform, common::CarPlatform::P3);
    EXPECT_EQ(scenario.ecuId, 0x10);
    EXPECT_EQ(scenario.timeoutMs, 1000u);
    ASSERT_EQ(scenario.steps.size(), 1u);
    EXPECT_EQ(scenario.steps.front().request, (std::vector<uint8_t>{0x22, 0xD1, 0x00}));
}

TEST(ScriptModel, TomlValuesOverrideBuiltInDefaults)
{
    const auto scenario = load("toml_values", R"(
version = 1
platform = "P3"
ecu = "B2"
timeout_ms = 1500
steps = [{ uds = "3E 80" }]
)");
    EXPECT_EQ(scenario.platform, common::CarPlatform::P3);
    EXPECT_EQ(scenario.ecuId, 0xB2);
    EXPECT_EQ(scenario.timeoutMs, 1500u);
}

TEST(ScriptModel, CliOverridesTomlValues)
{
    RunOptions options;
    options.scriptPlatformOverride = common::CarPlatform::P3_Y555_ICM;
    options.scriptEcuOverride = 0xB2;
    const auto scenario = load("cli_values", R"(
version = 1
platform = "P3"
ecu = "10"
steps = [{ uds = "22 D1 00" }]
)", options);
    EXPECT_EQ(scenario.platform, common::CarPlatform::P3_Y555_ICM);
    EXPECT_EQ(scenario.ecuId, 0xB2);
}

TEST(ScriptModel, VariableSubstitutionReachesStringValues)
{
    RunOptions options;
    options.scriptVariables.emplace_back("ECU", "B2");
    const auto scenario = load("variables", R"(
version = 1
ecu = "${ECU}"
steps = [{ uds = "22 D1 00" }]
)", options);
    EXPECT_EQ(scenario.ecuId, 0xB2);
}

TEST(ScriptModel, VariableValueCannotInjectTomlStructure)
{
    // The value contains quotes and a TOML assignment. Substituted into the raw text
    // this would create a real 'evil' field; substituted into the parsed string value it
    // must stay opaque payload text.
    RunOptions options;
    options.scriptVariables.emplace_back("N", "x\" evil = true\nb = \"");
    const auto scenario = load("injection", R"(
version = 1
steps = [{ name = "${N}", uds = "22 D1 00" }]
)", options);
    ASSERT_EQ(scenario.steps.size(), 1u);
    EXPECT_EQ(scenario.steps[0].name, "x\" evil = true\nb = \"");
}

TEST(ScriptModel, UnresolvedVariableMarkerFailsTheLoad)
{
    RunOptions options;
    const auto path = writeScenario("unresolved", R"(
version = 1
steps = [{ uds = "${TYP0}" }]
)");
    try {
        volvodiag::loadScriptScenario(path.string(), options);
        FAIL() << "expected an unresolved-marker error";
    }
    catch (const std::exception& ex) {
        const auto message = std::string(ex.what());
        EXPECT_NE(message.find("${TYP0}"), std::string::npos);
        EXPECT_NE(message.find("--var"), std::string::npos);
    }
}

TEST(ScriptModel, RejectsUnknownFieldsAndMultipleOperations)
{
    EXPECT_THROW(load("unknown", "version = 1\nwat = true\nsteps = [{ uds = \"3E 00\" }]\n"), std::exception);
    EXPECT_THROW(load("multiple", "version = 1\nsteps = [{ uds = \"3E 00\", delay_ms = 1 }]\n"), std::exception);
}

TEST(ScriptModel, RejectsOutOfRangeIdsAndInvalidSession)
{
    EXPECT_THROW(load("ecu_range", "version = 1\necu = \"100\"\nsteps = [{ uds = \"3E 00\" }]\n"), std::exception);
    EXPECT_THROW(load("did_range", "version = 1\nsteps = [{ did_read = [\"10000\"] }]\n"), std::exception);
    EXPECT_THROW(load("session_range", "version = 1\nprelude = { session = \"00\" }\nsteps = [{ uds = \"3E 00\" }]\n"), std::exception);
}

TEST(ScriptModel, RejectsMutuallyExclusiveExpectations)
{
    EXPECT_THROW(load("expect_timeout", R"(
version = 1
steps = [{ uds = "22 D1 00", expect = { timeout = true, positive = true } }]
)") , std::exception);
    EXPECT_THROW(load("expect_nrc", R"(
version = 1
steps = [{ uds = "22 D1 00", expect = { nrc = "13", prefix = "62" } }]
)") , std::exception);
}

TEST(ScriptModel, ResolvedScenarioRoundTripsWithoutLosingFields)
{
    const auto original = load("roundtrip_source", R"(
version = 1
name = "roundtrip"
platform = "P3"
ecu = "B2"
timeout_ms = 1500
[prelude]
wake = true
session = "programming"
[prelude.tester_present]
interval_ms = 1000
suppress_response = true
[prelude.security]
pin_env = "PAM_TEST_PIN"
[[steps]]
name = "read"
uds = "22 D1 00"
[steps.expect]
positive = true
prefix = "62 D1 00"
[[steps]]
name = "identity"
did_read = ["F180", "F18C"]
[[steps]]
name = "pause"
delay_ms = 2500
[[steps]]
name = "session"
session = "extended"
[[steps]]
name = "tester"
tester_present = false
[[steps]]
name = "expected-nrc"
uds = "22 F1 90"
[steps.expect]
nrc = "13"
[[steps]]
name = "expected-timeout"
uds = "22 F1 91"
[steps.expect]
timeout = true
)");

    std::ostringstream serialized;
    serialized << volvodiag::serializeScriptScenario(original);
    const auto path = writeScenario("roundtrip_resolved", serialized.str());
    const auto restored = volvodiag::loadScriptScenario(path.string(), {});

    EXPECT_EQ(restored.version, original.version);
    EXPECT_EQ(restored.name, original.name);
    EXPECT_EQ(restored.platform, original.platform);
    EXPECT_EQ(restored.ecuId, original.ecuId);
    EXPECT_EQ(restored.timeoutMs, original.timeoutMs);
    EXPECT_EQ(restored.wake, original.wake);
    EXPECT_EQ(restored.preludeSession, original.preludeSession);
    EXPECT_EQ(restored.testerPresentIntervalMs, original.testerPresentIntervalMs);
    EXPECT_EQ(restored.testerPresentSuppress, original.testerPresentSuppress);
    EXPECT_EQ(restored.securityPinEnv, original.securityPinEnv);
    ASSERT_EQ(restored.steps.size(), original.steps.size());
    for (size_t i = 0; i < original.steps.size(); ++i) {
        EXPECT_EQ(restored.steps[i].name, original.steps[i].name);
        EXPECT_EQ(restored.steps[i].kind, original.steps[i].kind);
        EXPECT_EQ(restored.steps[i].request, original.steps[i].request);
        EXPECT_EQ(restored.steps[i].dids, original.steps[i].dids);
        EXPECT_EQ(restored.steps[i].sessionType, original.steps[i].sessionType);
        EXPECT_EQ(restored.steps[i].delayMs, original.steps[i].delayMs);
        EXPECT_EQ(restored.steps[i].suppressResponse, original.steps[i].suppressResponse);
        EXPECT_EQ(restored.steps[i].expectPositive, original.steps[i].expectPositive);
        EXPECT_EQ(restored.steps[i].expectedPrefix, original.steps[i].expectedPrefix);
        EXPECT_EQ(restored.steps[i].expectNrc, original.steps[i].expectNrc);
        EXPECT_EQ(restored.steps[i].expectTimeout, original.steps[i].expectTimeout);
    }
}

TEST(ExitCodes, ClassifiesKnownExceptionFamilies)
{
    volvodiag::DiagError validation(volvodiag::ExitCode::ValidationError, "validation");
    common::UDSError nrc(0x13);
    common::UDSRequestRxTimeout timeout("timeout");
    common::UDSRequestTxError tx(1, 0, "tx");
    std::runtime_error generic("generic");
    EXPECT_EQ(volvodiag::classifyExitCode(validation), 6);
    EXPECT_EQ(volvodiag::classifyExitCode(nrc), 5);
    EXPECT_EQ(volvodiag::classifyExitCode(timeout), 4);
    EXPECT_EQ(volvodiag::classifyExitCode(tx), 3);
    EXPECT_EQ(volvodiag::classifyExitCode(generic), 1);
}

TEST(OutputFormat, EscapesCsvAndFormatsHex)
{
    EXPECT_EQ(volvodiag::csvEscape("plain"), "plain");
    EXPECT_EQ(volvodiag::csvEscape("a,b\"c\nd"), "\"a,b\"\"c\nd\"");
    EXPECT_EQ(volvodiag::hexNumber(0xA, 2), "0x0a");
    EXPECT_EQ(volvodiag::hexNumber(0x736, 3), "0x736");
}

TEST(DiagContext, ValidatesRealPamPayloadPrefix)
{
    const std::vector<uint8_t> response{
        0x62, 0xF1, 0x80, 0x01, 0x43, 0x47, 0x53, 0x2D, 0x53, 0x31, 0x32};
    EXPECT_NO_THROW(volvodiag::ensurePayloadPrefix(response, {0x62, 0xF1, 0x80}));
    EXPECT_NO_THROW(volvodiag::ensurePayloadPrefix({}, {}));
    EXPECT_THROW(volvodiag::ensurePayloadPrefix(response, {0x62, 0xF1, 0x81}), std::exception);
    EXPECT_THROW(volvodiag::ensurePayloadPrefix(response, {0x62, 0xF1, 0x80, 0x01, 0x43, 0x47, 0x53, 0x2D, 0x53, 0x31, 0x32, 0x00}), std::exception);
}

TEST(CommandRisk, WriteEraseIsExactlyTheYesGatedCommandSet)
{
    const std::array<std::pair<volvodiag::Command, volvodiag::CommandRisk>, 13> expected{{
        {volvodiag::Command::DidWrite, volvodiag::CommandRisk::WriteErase},
        {volvodiag::Command::Reset, volvodiag::CommandRisk::WriteErase},
        {volvodiag::Command::Routine, volvodiag::CommandRisk::WriteErase},
        {volvodiag::Command::RoutineScan, volvodiag::CommandRisk::WriteErase},
        {volvodiag::Command::DtcClear, volvodiag::CommandRisk::WriteErase},
        {volvodiag::Command::ObdDtcClear, volvodiag::CommandRisk::WriteErase},
        {volvodiag::Command::DidRead, volvodiag::CommandRisk::ReadOnly},
        {volvodiag::Command::DtcRead, volvodiag::CommandRisk::ReadOnly},
        {volvodiag::Command::ListDevices, volvodiag::CommandRisk::ReadOnly},
        {volvodiag::Command::Send, volvodiag::CommandRisk::StateChanging},
        {volvodiag::Command::Session, volvodiag::CommandRisk::StateChanging},
        {volvodiag::Command::Script, volvodiag::CommandRisk::StateChanging},
        {volvodiag::Command::ReconCapture, volvodiag::CommandRisk::StateChanging},
    }};
    for (const auto [command, risk] : expected) {
        EXPECT_EQ(volvodiag::commandRisk(command), risk);
    }

    RunOptions options;
    ASSERT_TRUE(parseArgs({"did", "write", "-f", "P3", "--id", "F1A0", "--data", "01"}, options));
    EXPECT_FALSE(options.confirmDestructive);
    ASSERT_TRUE(parseArgs({"did", "write", "-f", "P3", "--id", "F1A0", "--data", "01", "--yes"}, options));
    EXPECT_TRUE(options.confirmDestructive);
}

TEST(ReconCapture, ReconstructsOutOfOrderA4FramesAndRejectsGaps)
{
    const std::vector<uint8_t> signature{0xA4};
    const std::vector<volvodiag::ReconFrame> frames{
        {20, 0x73E, {0xA4, 0x02, 0xFF, 0xF8, 0x40, 0x0F, 0xF2, 0x73}},
        {10, 0x73E, {0xA4, 0x00, 0xFF, 0xF0, 0x40, 0x23, 0x40, 0x1E}},
        {30, 0x73E, {0xA4, 0x03, 0xFF, 0xFC, 0x40, 0x05, 0xE0, 0xFF}},
        {15, 0x73E, {0xA4, 0x01, 0xFF, 0xF4, 0x40, 0x19, 0x40, 0x14}},
        {40, 0x700, {0xA4, 0x04, 0x00, 0x00, 0, 0, 0, 0}},
    };
    const auto image = volvodiag::reconstructReconImage(frames, 0x73E, signature, 4);
    EXPECT_EQ(image.address, 0xFFF0u);
    EXPECT_EQ(image.bytes, (std::vector<uint8_t>{0x40, 0x23, 0x40, 0x1E,
        0x40, 0x19, 0x40, 0x14, 0x40, 0x0F, 0xF2, 0x73,
        0x40, 0x05, 0xE0, 0xFF}));
    EXPECT_THROW(volvodiag::reconstructReconImage(
        {frames[0], frames[1], frames[2]}, 0x73E, signature, 4), std::runtime_error);
}

// ---- destructive-step confirmation gate (--yes) ----------------------------

TEST(ScriptModel, DestructiveServiceClassification)
{
    using volvodiag::isDestructiveUdsService;
    // Persistent-change services: reset, DTC clear, write, routine, download/transfer.
    EXPECT_TRUE(isDestructiveUdsService({0x11, 0x01}));
    EXPECT_TRUE(isDestructiveUdsService({0x14, 0xFF, 0xFF, 0xFF}));
    EXPECT_TRUE(isDestructiveUdsService({0x2E, 0xD1, 0x00, 0x01}));
    EXPECT_TRUE(isDestructiveUdsService({0x31, 0x01, 0x03, 0x01}));
    EXPECT_TRUE(isDestructiveUdsService({0x34, 0x00, 0x44}));
    EXPECT_TRUE(isDestructiveUdsService({0x36, 0x01, 0xAA}));
    // Read/state-changing-but-not-persistent services stay ungated.
    EXPECT_FALSE(isDestructiveUdsService({}));
    EXPECT_FALSE(isDestructiveUdsService({0x22, 0xF1, 0x90}));
    EXPECT_FALSE(isDestructiveUdsService({0x10, 0x03}));
    EXPECT_FALSE(isDestructiveUdsService({0x3E, 0x00}));
    EXPECT_FALSE(isDestructiveUdsService({0x27, 0x01}));
}

TEST(ScriptModel, DestructiveStepsRequireConfirmation)
{
    const auto scenario = load("destructive", R"(
version = 1
steps = [
    { name = "read", uds = "22 D1 00" },
    { name = "write", uds = "2E D1 00 01 02" },
]
)");
    EXPECT_NO_THROW(volvodiag::ensureDestructiveStepsConfirmed(scenario, true));
    try {
        volvodiag::ensureDestructiveStepsConfirmed(scenario, false);
        FAIL() << "expected DiagError for an unconfirmed destructive step";
    }
    catch (const volvodiag::DiagError& ex) {
        EXPECT_EQ(ex.code(), volvodiag::ExitCode::UsageError);
        const auto message = std::string(ex.what());
        EXPECT_NE(message.find("'write'"), std::string::npos);
        EXPECT_NE(message.find("0x2e"), std::string::npos);
        EXPECT_NE(message.find("--yes"), std::string::npos);
    }
}

TEST(ScriptModel, ReadOnlyScenarioNeedsNoConfirmation)
{
    const auto scenario = load("gate_readonly", R"(
version = 1
steps = [
    { uds = "22 D1 00" },
    { did_read = ["D100"] },
    { session = "extended" },
    { tester_present = true },
    { delay_ms = 5 },
]
)");
    EXPECT_NO_THROW(volvodiag::ensureDestructiveStepsConfirmed(scenario, false));
}
