#include <common/CliSupport.hpp>
#include <common/RuntimeDiagnostics.hpp>
#include <common/Util.hpp>

#include "CliParser.hpp"
#include "CommandRisk.hpp"
#include "DiagContext.hpp"
#include "ExitCodes.hpp"
#include "OutputFormat.hpp"
#include "RawCanCommands.hpp"
#include "ReconCapture.hpp"
#include "UdsCommands.hpp"
#include "VolvoDiagOptions.hpp"

#include <easylogging++.h>

#include <iostream>

INITIALIZE_EASYLOGGINGPP

int main(int argc, const char* argv[])
{
    using namespace volvodiag;
    common::initLogger("VolvoDiag.log", common::isDebugLoggingRequested(argc, argv));
    common::printRuntimeDiagnostics("VolvoDiag");
    common::installConsoleCtrlHandler();

    // Mark each log block with the full command line.
    {
        std::string invocation;
        for (int i = 0; i < argc; ++i) {
            if (i > 0) {
                invocation += ' ';
            }
            invocation += argv[i];
        }
        LOG(INFO) << "===== invocation: " << invocation << " =====";
    }

    RunOptions options;
    if (!parseOptions(argc, argv, options)) {
        return static_cast<int>(ExitCode::UsageError);
    }

    // Print risk to stderr; list commands stay quiet.
    if (options.command != Command::None && options.command != Command::ListDevices
        && options.command != Command::Buses) {
        const auto risk = commandRisk(options.command);
        LOG(INFO) << "risk: " << riskLabel(risk);
        std::cerr << "[risk: " << riskLabel(risk) << "]" << std::endl;
    }

    try {
        // Dry-run only validates and expands the script; no adapter is needed.
        if (options.command == Command::Script && options.scriptDryRun) {
            runScript({}, options);
            return 0;
        }
        setUdsTracePath(options.tracePath);
        const auto devices = common::getAvailableDevices();
        switch (options.command) {
        case Command::ListDevices:
            common::printAvailableDevices(std::cout, devices);
            return 0;
        case Command::Buses:
            runBuses(options);
            return 0;
        case Command::Send:
            runSend(devices, options);
            return 0;
        case Command::CanSend:
            runCanSend(devices, options);
            return 0;
        case Command::CanRequest:
            runCanRequest(devices, options);
            return 0;
        case Command::CanPeriodic:
            runCanPeriodic(devices, options);
            return 0;
        case Command::CanReplay:
            runCanReplay(devices, options);
            return 0;
        case Command::Wake:
            runWake(devices, options);
            return 0;
        case Command::Probe:
            runProbe(devices, options);
            return 0;
        case Command::Ident:
            runIdent(devices, options);
            return 0;
        case Command::DidRead:
            runDidRead(devices, options);
            return 0;
        case Command::DidWrite:
            runDidWrite(devices, options);
            return 0;
        case Command::DidScan:
            runDidScan(devices, options);
            return 0;
        case Command::Scan:
            runScan(devices, options);
            return 0;
        case Command::Monitor:
            runMonitor(devices, options);
            return 0;
        case Command::ReconCapture:
            runReconCapture(devices, options);
            return 0;
        case Command::Session:
            runSession(devices, options);
            return 0;
        case Command::TesterPresent:
            runTesterPresent(devices, options);
            return 0;
        case Command::Reset:
            runReset(devices, options);
            return 0;
        case Command::Routine:
            runRoutine(devices, options);
            return 0;
        case Command::RoutineScan:
            runRoutineScan(devices, options);
            return 0;
        case Command::DtcRead:
            runDtcRead(devices, options);
            return 0;
        case Command::DtcClear:
            runDtcClear(devices, options);
            return 0;
        case Command::DtcSnapshot:
            runDtcSnapshot(devices, options);
            return 0;
        case Command::DtcExtended:
            runDtcExtended(devices, options);
            return 0;
        case Command::ObdVin:
            runObdVin(devices, options);
            return 0;
        case Command::ObdPid:
            runObdPid(devices, options);
            return 0;
        case Command::ObdDtcRead:
            runObdDtcRead(devices, options);
            return 0;
        case Command::ObdDtcClear:
            runObdDtcClear(devices, options);
            return 0;
        case Command::Script:
            runScript(devices, options);
            return 0;
        case Command::None:
            return static_cast<int>(ExitCode::UsageError);
        }
    }
    catch (const std::exception& ex) {
        const int code = classifyExitCode(ex);
        LOG(ERROR) << "VolvoDiag failed (exit " << code << "): " << ex.what();
        std::cerr << ex.what() << std::endl;
        return code;
    }
    return static_cast<int>(ExitCode::Success);
}
