#include "CliParser.hpp"
#include "ScriptModel.hpp"
#include "VolvoDiagOptions.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <filesystem>
#include <iostream>
#include <streambuf>
#include <utility>
#include <vector>

using namespace volvodiag;

namespace {

// Parses an argv (without the leading program name) into options. parseOptions prints
// usage to stderr on failure, so we silence cerr for the duration.
bool parse(std::vector<const char*> args, RunOptions& options)
{
    args.insert(args.begin(), "VolvoDiag");
    std::streambuf* saved = std::cerr.rdbuf(nullptr);
    const bool ok = parseOptions(static_cast<int>(args.size()), args.data(), options);
    std::cerr.rdbuf(saved);
    return ok;
}

} // namespace

// ---- command mapping & defaults -------------------------------------------

TEST(Cli, DidReadMapsAndDefaults)
{
    RunOptions o;
    ASSERT_TRUE(parse({"did", "read", "-f", "P3", "--id", "F190", "--id", "F18C", "-o", "did.csv", "--trace", "uds.csv"}, o));
    EXPECT_EQ(o.command, Command::DidRead);
    EXPECT_EQ(o.ecuId, 0x10);                 // default ECU
    EXPECT_EQ(o.tracePath, "uds.csv");
    EXPECT_EQ(o.didOutputPath, "did.csv");
    ASSERT_EQ(o.didIds.size(), 2u);
    EXPECT_EQ(o.didIds[0], 0xF190);
    EXPECT_EQ(o.didIds[1], 0xF18C);
}

TEST(Cli, DidReadUsesOutputForTableAndTraceForFixtures)
{
    RunOptions o;
    ASSERT_TRUE(parse({"did", "read", "-f", "P3", "--id", "F190", "-o", "did.csv"}, o));
    EXPECT_EQ(o.command, Command::DidRead);
    EXPECT_EQ(o.didOutputPath, "did.csv");
    EXPECT_TRUE(o.tracePath.empty());
}

TEST(Cli, ReadOnlyUdsCommandsAcceptPrelude)
{
    RunOptions o;
    ASSERT_TRUE(parse({"dtc", "read", "-f", "P3", "--wake", "--session", "programming",
                       "--security", "DCD5447AE6"}, o));
    EXPECT_EQ(o.command, Command::DtcRead);
    EXPECT_TRUE(o.preludeWake);
    EXPECT_EQ(o.preludeSessionType, 0x02);
    EXPECT_EQ(o.preludeSecurityKey,
              (std::vector<uint8_t>{0xDC, 0xD5, 0x44, 0x7A, 0xE6}));

    ASSERT_TRUE(parse({"routine", "results", "-f", "P3", "--id", "FF00", "--wake",
                       "--session", "programming", "--security", "DC D5 44 7A E6"}, o));
    EXPECT_EQ(o.command, Command::Routine);
    EXPECT_TRUE(o.preludeWake);
    EXPECT_EQ(o.preludeSessionType, 0x02);
    EXPECT_EQ(o.preludeSecurityKey,
              (std::vector<uint8_t>{0xDC, 0xD5, 0x44, 0x7A, 0xE6}));
}

TEST(Cli, KeepaliveFlagAppliesToLongReadOnlyScans)
{
    RunOptions o;
    ASSERT_TRUE(parse({"did", "scan", "-f", "P3", "--keepalive"}, o));
    EXPECT_EQ(o.command, Command::DidScan);
    EXPECT_TRUE(o.keepalive);

    ASSERT_TRUE(parse({"dtc", "snapshot", "-f", "P3", "--dtc", "012345", "--keepalive"}, o));
    EXPECT_EQ(o.command, Command::DtcSnapshot);
    EXPECT_TRUE(o.keepalive);

    ASSERT_TRUE(parse({"routine-scan", "-f", "P3", "--keepalive", "--yes"}, o));
    EXPECT_EQ(o.command, Command::RoutineScan);
    EXPECT_TRUE(o.keepalive);
}

TEST(Cli, DidReadLoadsIdsFromFile)
{
    const auto dir = std::filesystem::temp_directory_path() / "volvotools-tests";
    std::filesystem::create_directories(dir);
    const auto path = (dir / "did_ids.txt").string();
    {
        std::ofstream file(path, std::ios::trunc);
        ASSERT_TRUE(file.is_open());
        file << "# comment\n";
        file << "F190\n";
        file << "\n";
        file << " F18C ; ignored trailing comment\n";
    }

    RunOptions o;
    ASSERT_TRUE(parse({"did", "read", "-f", "P3", "--ids-from", path.c_str(), "-o", "did.csv"}, o));
    EXPECT_EQ(o.command, Command::DidRead);
    ASSERT_EQ(o.didIds.size(), 2u);
    EXPECT_EQ(o.didIds[0], 0xF190);
    EXPECT_EQ(o.didIds[1], 0xF18C);
}

TEST(Cli, GroupedCanRequestResolves)
{
    RunOptions o;
    ASSERT_TRUE(parse({"can", "request", "-f", "P3", "--can-id", "7E0", "--data", "02 10 03"}, o));
    EXPECT_EQ(o.command, Command::CanRequest);
    EXPECT_EQ(o.canId, 0x7E0u);
}

TEST(Cli, CanSendAcceptsDropTxEchoAsNoOp)
{
    RunOptions o;
    ASSERT_TRUE(parse({"can", "send", "-f", "P3", "--can-id", "7E0", "--data", "01 02", "--drop-tx-echo"}, o));
    EXPECT_EQ(o.command, Command::CanSend);
    EXPECT_TRUE(o.txEchoFilter.enabled);
}

TEST(Cli, EcuScanRenamed)
{
    RunOptions o;
    EXPECT_TRUE(parse({"ecu-scan", "-f", "P3"}, o));
    EXPECT_EQ(o.command, Command::Scan);
}

// ---- named enum values ----------------------------------------------------

TEST(Cli, ResetTypeNames)
{
    RunOptions o;
    ASSERT_TRUE(parse({"reset", "-f", "P3", "--type", "soft", "--yes"}, o));
    EXPECT_EQ(o.resetType, 0x03);
    ASSERT_TRUE(parse({"reset", "-f", "P3", "--type", "02", "--yes"}, o)); // hex still works
    EXPECT_EQ(o.resetType, 0x02);
}

TEST(Cli, RoutineSubAndSession)
{
    RunOptions o;
    ASSERT_TRUE(parse({"routine-scan", "-f", "P3", "--sub", "results",
                       "--session", "ext", "--yes"}, o));
    EXPECT_EQ(o.routineSubFunction, 0x03);
    EXPECT_TRUE(o.routineExtendedSession);
}

TEST(Cli, SessionCommand)
{
    RunOptions o;
    ASSERT_TRUE(parse({"session", "-f", "P3", "-e", "10", "--type", "programming", "--suppress"}, o));
    EXPECT_EQ(o.command, Command::Session);
    EXPECT_EQ(o.sessionType, 0x02);
    EXPECT_TRUE(o.sessionSuppress);

    ASSERT_TRUE(parse({"session", "-f", "P3", "--type", "03"}, o));
    EXPECT_EQ(o.sessionType, 0x03);
}

TEST(Cli, TesterPresentCommand)
{
    RunOptions o;
    ASSERT_TRUE(parse({"tester-present", "-f", "P3", "--count", "0", "--interval-ms", "1500", "--suppress"}, o));
    EXPECT_EQ(o.command, Command::TesterPresent);
    EXPECT_EQ(o.testerPresentCount, 0u);
    EXPECT_EQ(o.testerPresentIntervalMs, 1500u);
    EXPECT_TRUE(o.testerPresentSuppress);
}

TEST(Cli, RoutineCommand)
{
    RunOptions o;
    ASSERT_TRUE(parse({"routine", "start", "-f", "P3", "--id", "FF00", "--data", "01 02",
                       "--session", "ext", "--suppress", "--yes"}, o));
    EXPECT_EQ(o.command, Command::Routine);
    EXPECT_EQ(o.routineSubFunction, 0x01);
    EXPECT_EQ(o.routineId, 0xFF00);
    EXPECT_TRUE(o.routineExtendedSession);
    EXPECT_TRUE(o.routineSuppress);
    ASSERT_EQ(o.routineData.size(), 2u);
    EXPECT_EQ(o.routineData[0], 0x01);
    EXPECT_EQ(o.routineData[1], 0x02);

    ASSERT_TRUE(parse({"routine", "results", "-f", "P3", "--id", "FF00"}, o));
    EXPECT_EQ(o.command, Command::Routine);
    EXPECT_EQ(o.routineSubFunction, 0x03);
    EXPECT_TRUE(o.confirmDestructive);
}

// "extended" is accepted as a synonym of "ext" on the default/ext --session flag, matching
// the standalone `session --type` vocabulary.
TEST(Cli, AcceptsExtendedSessionSynonym)
{
    RunOptions o;
    ASSERT_TRUE(parse({"routine", "results", "-f", "P3", "--id", "FF00", "--session", "extended"}, o));
    EXPECT_TRUE(o.preludeSessionType == 0x03);

    ASSERT_TRUE(parse({"did", "write", "-f", "P3", "--id", "F1A0", "--data", "01",
                       "--session", "extended", "--yes"}, o));
    EXPECT_TRUE(o.didWriteExtendedSession);
}

// ---- confirmation gates ---------------------------------------------------

// The --yes gate is enforced by the command layer (run* throws), not the parser. At the
// parse layer the contract is that confirmDestructive mirrors --yes; the gate itself is exercised
// by the on-bench / command tests.
TEST(Cli, ConfirmFlagMirrorsYes)
{
    RunOptions o;
    ASSERT_TRUE(parse({"did", "write", "-f", "P3", "--id", "F1A0", "--data", "01"}, o));
    EXPECT_EQ(o.command, Command::DidWrite);
    EXPECT_EQ(o.didWriteId, 0xF1A0);
    EXPECT_FALSE(o.confirmDestructive);

    ASSERT_TRUE(parse({"did", "write", "-f", "P3", "--id", "F1A0", "--data", "01", "--yes"}, o));
    EXPECT_TRUE(o.confirmDestructive);

    ASSERT_TRUE(parse({"reset", "-f", "P3", "--yes"}, o));
    EXPECT_TRUE(o.confirmDestructive);
}

// ---- validation rejections ------------------------------------------------

TEST(Cli, RejectsBusWithoutFunctional)
{
    RunOptions o;
    EXPECT_FALSE(parse({"reset", "-f", "P3", "--bus", "CAN HS", "--yes"}, o));
}

TEST(Cli, AcceptsFunctionalReset)
{
    RunOptions o;
    ASSERT_TRUE(parse({"reset", "-f", "P3", "--functional", "--suppress", "--yes"}, o));
    EXPECT_TRUE(o.resetFunctional);
    EXPECT_TRUE(o.resetSuppress);
}

TEST(Cli, RejectsBaselineRecordAndCompareTogether)
{
    RunOptions o;
    EXPECT_FALSE(parse({"monitor", "-f", "P3", "--baseline-record", "a.txt",
                        "--baseline-compare", "b.txt"}, o));
}

TEST(Cli, MonitorDurationFlag)
{
    RunOptions o;
    ASSERT_TRUE(parse({"monitor", "-f", "P3", "--duration-ms", "5000"}, o));
    EXPECT_EQ(o.command, Command::Monitor);
    EXPECT_EQ(o.monitorDurationMs, 5000u);

    ASSERT_TRUE(parse({"monitor", "-f", "P3", "--timeout-ms", "2500"}, o));
    EXPECT_EQ(o.monitorDurationMs, 2500u);
}

TEST(Cli, RejectsBadSessionAndType)
{
    RunOptions o;
    EXPECT_FALSE(parse({"routine-scan", "-f", "P3", "--session", "bogus", "--yes"}, o));
    EXPECT_FALSE(parse({"reset", "-f", "P3", "--type", "bogus", "--yes"}, o));
    EXPECT_FALSE(parse({"session", "-f", "P3", "--type", "bogus"}, o));
    EXPECT_FALSE(parse({"tester-present", "-f", "P3", "--interval-ms", "0"}, o));
    EXPECT_FALSE(parse({"routine", "start", "-f", "P3", "--id", "10000", "--yes"}, o));
    EXPECT_FALSE(parse({"routine", "results", "-f", "P3", "--id", "FF00", "--session", "bogus"}, o));
}

TEST(Cli, RejectsBadHexAndOutOfRange)
{
    RunOptions o;
    EXPECT_FALSE(parse({"can", "send", "-f", "P3", "--can-id", "7E0", "--data", "ZZ"}, o));
    EXPECT_FALSE(parse({"did", "scan", "-f", "P3", "--from", "0010", "--to", "0001"}, o));
    EXPECT_FALSE(parse({"ident", "-f", "P3", "-e", "1FF"}, o)); // ecu > 1 byte
}

TEST(Script, RejectsOutOfRangeEcuIds)
{
    const auto dir = std::filesystem::temp_directory_path() / "volvotools-tests";
    std::filesystem::create_directories(dir);
    RunOptions options;

    for (const auto& [name, ecu] : std::vector<std::pair<std::string, std::string>>{
             {"large_hex", "\"0x17A\""}, {"large_integer", "378"}, {"negative", "-1"}}) {
        const auto path = dir / ("scenario_" + name + ".toml");
        {
            std::ofstream file(path, std::ios::trunc);
            ASSERT_TRUE(file.is_open());
            file << "version = 1\necu = " << ecu
                 << "\nsteps = [{ uds = \"3E 00\" }]\n";
        }
        EXPECT_THROW(loadScriptScenario(path.string(), options), std::exception) << name;
    }
}

TEST(Cli, ScriptCommandUsesArgparseAndTracksExplicitOverrides)
{
    RunOptions options;
    ASSERT_TRUE(parse({"script", "--file", "scenario.toml", "--output-dir", "artifacts",
                       "--trace", "trace.csv", "-d", "Mongoose", "-f", "P3_Y555_IAM",
                       "-e", "B2", "-b", "125000", "--var", "A=one", "--var", "B=two",
                       "--dry-run"}, options));

    EXPECT_EQ(options.command, Command::Script);
    EXPECT_EQ(options.scriptPath, "scenario.toml");
    EXPECT_EQ(options.scriptOutputDir, "artifacts");
    EXPECT_EQ(options.tracePath, "trace.csv");
    EXPECT_EQ(options.deviceName, "Mongoose");
    ASSERT_TRUE(options.scriptPlatformOverride.has_value());
    EXPECT_EQ(*options.scriptPlatformOverride, common::CarPlatform::P3_Y555_IAM);
    ASSERT_TRUE(options.scriptEcuOverride.has_value());
    EXPECT_EQ(*options.scriptEcuOverride, 0xB2);
    ASSERT_TRUE(options.baudrateOverride.has_value());
    EXPECT_EQ(*options.baudrateOverride, 125000u);
    ASSERT_EQ(options.scriptVariables.size(), 2u);
    EXPECT_TRUE(options.scriptDryRun);
}

TEST(Script, CliOverridesTomlPlatformAndEcu)
{
    const auto dir = std::filesystem::temp_directory_path() / "volvotools-tests";
    std::filesystem::create_directories(dir);
    const auto path = dir / "scenario_overrides.toml";
    {
        std::ofstream file(path, std::ios::trunc);
        ASSERT_TRUE(file.is_open());
        file << "version = 1\nplatform = \"P3\"\necu = \"10\"\n"
                "steps = [{ uds = \"3E 00\" }]\n";
    }

    RunOptions options;
    options.scriptPlatformOverride = common::CarPlatform::P3_Y413;
    options.scriptEcuOverride = 0xB2;
    const auto scenario = loadScriptScenario(path.string(), options);
    EXPECT_EQ(scenario.platform, common::CarPlatform::P3_Y413);
    EXPECT_EQ(scenario.ecuId, 0xB2);
}

TEST(Script, TomlPlatformAndEcuOverrideBuiltInDefaults)
{
    const auto dir = std::filesystem::temp_directory_path() / "volvotools-tests";
    std::filesystem::create_directories(dir);
    const auto path = dir / "scenario_toml_values.toml";
    {
        std::ofstream file(path, std::ios::trunc);
        ASSERT_TRUE(file.is_open());
        file << "version = 1\nplatform = \"P3_Y413\"\necu = \"B2\"\n"
                "steps = [{ uds = \"3E 00\" }]\n";
    }

    const RunOptions options;
    const auto scenario = loadScriptScenario(path.string(), options);
    EXPECT_EQ(scenario.platform, common::CarPlatform::P3_Y413);
    EXPECT_EQ(scenario.ecuId, 0xB2);
}

// ---- send --format (renamed from --output) --------------------------------

TEST(Cli, SendFormatFlag)
{
    RunOptions o;
    ASSERT_TRUE(parse({"send", "-f", "P3", "--data", "22 F1 90", "--format", "payload"}, o));
    EXPECT_EQ(o.outputMode, "payload");
    EXPECT_FALSE(parse({"send", "-f", "P3", "--data", "22 F1 90", "--format", "bogus"}, o));
}

TEST(Cli, ProbeReassembleFlag)
{
    RunOptions o;
    ASSERT_TRUE(parse({"probe", "-f", "P3", "--bus", "CAN MS", "--reassemble"}, o));
    EXPECT_EQ(o.command, Command::Probe);
    EXPECT_TRUE(o.probeReassemble);
}

TEST(Cli, ProbeUsesOutputForCsv)
{
    RunOptions o;
    ASSERT_TRUE(parse({"probe", "-f", "P3", "--bus", "CAN MS", "-o", "probe.csv"}, o));
    EXPECT_EQ(o.command, Command::Probe);
    EXPECT_EQ(o.probeOutputPath, "probe.csv");
    EXPECT_TRUE(o.tracePath.empty());
}

TEST(Cli, DidReadRequiresIdsSource)
{
    RunOptions o;
    EXPECT_FALSE(parse({"did", "read", "-f", "P3", "-o", "did.csv"}, o));
}

TEST(Cli, ReconCaptureAllowsOmittedReferenceBytes)
{
    RunOptions o;
    ASSERT_TRUE(parse({"recon-capture", "--capture-can-id", "73E",
        "--req-id", "736", "--rsp-id", "73E",
        "--capture-marker", "A4", "--trigger-routine", "0301:00001400",
        "--out", "recon.csv", "--health-session", "10 02",
        "--health-session-expect", "50 02", "--health-did", "22 D1 00",
        "--health-did-expect", "62 D1 00 02"}, o));
    EXPECT_EQ(o.command, Command::ReconCapture);
    EXPECT_TRUE(o.reconReferenceBytes.empty());
}
