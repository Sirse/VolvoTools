#pragma once

#include <common/DeviceInfo.hpp>

#include <j2534/J2534_v0404.h>

#include <chrono>
#include <cstdint>
#include <fstream>
#include <functional>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace volvodiag {

// One compact line summarising a single request/response operation, e.g.
//   OK   send  ecu=0x10  tx=[22 F1 90]  rx=[62 F1 90 01 02]  (12.3 ms)
//   FAIL session  ecu=0x10  tx=[10 02]  nrc=0x33 Security access denied  (8.1 ms)
struct ResultLine {
    std::string command;            // command name, e.g. "send"
    std::string ecu;                // formatted ecu label, e.g. "0x10" (may be empty)
    std::vector<uint8_t> request;   // tx bytes to show (may be empty)
    std::vector<uint8_t> response;  // rx bytes to show on success (may be empty)
    std::optional<uint8_t> nrc;     // set when the ECU returned a negative response
    std::string detail;             // freeform note ("suppressed", counter, error text)
    double durationMs{0.0};
    bool ok{true};
};

// Prints a compact OK/FAIL result line.
void printResultLine(const ResultLine& line);

// Runs and times an action, prints its result, then rethrows failures.
void runReported(const std::string& command, const std::string& ecuLabel,
                 const std::vector<uint8_t>& request,
                 const std::function<std::vector<uint8_t>()>& action,
                 const std::function<std::string(const std::vector<uint8_t>&)>& describe = {});

// Prints space-separated hex bytes followed by a newline to std::cout.
void printHexBytes(const std::vector<uint8_t>& data);

// Formats a value as "0x" + zero-padded hex of the given width.
std::string hexNumber(uint32_t value, size_t width);

// Formats CAN IDs as 3-digit or 8-digit hexadecimal.
std::string formatCanId(uint32_t canId);

// Formats space-separated hex bytes into a string (no trailing newline).
std::string formatBytes(const std::vector<uint8_t>& bytes);

// Escapes CSV fields when needed.
std::string csvEscape(const std::string& value);

// Writes one CAN frame as a "time_ms,can_id,dlc,data" CSV row.
void writeFrame(std::ostream& output, std::chrono::milliseconds elapsed, const PASSTHRU_MSG& msg);

// Splits a CSV line into trimmed cells.
std::vector<std::string> splitCsvLine(const std::string& line);

// Parses a strict decimal unsigned value, throwing on trailing garbage.
uint64_t parseDecimalU64(const std::string& input, const std::string& fieldName);

// Writes CSV to stdout and optionally to a file.
class DiagOutput {
public:
    // Opens path when given; otherwise writes only to stdout.
    DiagOutput(const std::string& path, const std::string& label);

    // Runs fn for every active stream.
    void each(const std::function<void(std::ostream&)>& fn);

    // Runs fn only for the file stream.
    void fileOnly(const std::function<void(std::ostream&)>& fn);

    // True when the output file is open.
    bool hasFile() const;

    void line(const std::string& text);

private:
    std::ofstream _file;
};

} // namespace volvodiag
