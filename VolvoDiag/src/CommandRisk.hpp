#pragma once

#include "VolvoDiagOptions.hpp"

namespace volvodiag {

// Command risk shown before execution. Write/erase needs --yes.
enum class CommandRisk {
    ReadOnly,       // only reads state (or is purely local/meta)
    StateChanging,  // changes transient ECU/bus state (session, reset flows, raw traffic)
    WriteErase,     // writes or erases persistent ECU state; requires --yes
};

// Maps a command to its risk class.
CommandRisk commandRisk(Command command);

// Short risk label for output.
const char* riskLabel(CommandRisk risk);

} // namespace volvodiag
