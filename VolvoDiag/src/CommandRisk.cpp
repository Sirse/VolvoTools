#include "CommandRisk.hpp"

namespace volvodiag {

CommandRisk commandRisk(Command command)
{
    switch (command) {
    // Persistent changes need --yes.
    case Command::DidWrite:
    case Command::Reset:
    case Command::Routine:
    case Command::RoutineScan:
    case Command::DtcClear:
    case Command::ObdDtcClear:
        return CommandRisk::WriteErase;

    // Commands that may change ECU or bus state.
    case Command::Send:
    case Command::CanSend:
    case Command::CanRequest:
    case Command::CanPeriodic:
    case Command::CanReplay:
    case Command::Wake:
    case Command::Session:
    case Command::TesterPresent:
    case Command::ReconCapture:
    // Script risk depends on its steps.
    case Command::Script:
        return CommandRisk::StateChanging;

    // Read-only and local commands.
    case Command::None:
    case Command::ListDevices:
    case Command::Buses:
    case Command::Ident:
    case Command::Probe:
    case Command::Scan:
    case Command::Monitor:
    case Command::DidRead:
    case Command::DidScan:
    case Command::DtcRead:
    case Command::DtcSnapshot:
    case Command::DtcExtended:
    case Command::ObdVin:
    case Command::ObdPid:
    case Command::ObdDtcRead:
        return CommandRisk::ReadOnly;
    }
    return CommandRisk::ReadOnly;
}

const char* riskLabel(CommandRisk risk)
{
    switch (risk) {
    case CommandRisk::ReadOnly:      return "read-only";
    case CommandRisk::StateChanging: return "state-changing";
    case CommandRisk::WriteErase:    return "write-erase";
    }
    return "read-only";
}

} // namespace volvodiag
