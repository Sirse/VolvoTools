#include "OutputFormat.hpp"

#include <common/RuntimeDiagnostics.hpp>
#include <common/Util.hpp>
#include <common/protocols/UDSError.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace volvodiag {

void printHexBytes(const std::vector<uint8_t>& data)
{
    std::cout << common::formatHexBytesLower(data) << std::endl;
}

std::string hexNumber(uint32_t value, size_t width)
{
    std::stringstream ss;
    ss << "0x" << std::hex << std::setw(static_cast<int>(width)) << std::setfill('0') << value;
    return ss.str();
}

std::string formatCanId(uint32_t canId)
{
    return hexNumber(canId, canId > 0x7FF ? 8 : 3);
}

std::string formatBytes(const std::vector<uint8_t>& bytes)
{
    return common::formatHexBytesLower(bytes);
}

std::string csvEscape(const std::string& value)
{
    if (value.find_first_of(",\"\r\n") == std::string::npos) {
        return value;
    }

    std::string result{"\""};
    for (const auto ch : value) {
        if (ch == '"') {
            result += "\"\"";
        } else {
            result.push_back(ch);
        }
    }
    result.push_back('"');
    return result;
}

void printResultLine(const ResultLine& line)
{
    std::ostringstream out;
    out << (line.ok ? "OK  " : "FAIL") << ' ' << line.command;
    if (!line.ecu.empty()) {
        out << "  ecu=" << line.ecu;
    }
    if (!line.request.empty()) {
        out << "  tx=[" << formatBytes(line.request) << ']';
    }
    if (line.nrc) {
        out << "  nrc=" << hexNumber(*line.nrc, 2);
        if (!line.detail.empty()) {
            out << ' ' << line.detail;
        }
    } else {
        if (!line.response.empty()) {
            out << "  rx=[" << formatBytes(line.response) << ']';
        }
        if (!line.detail.empty()) {
            out << "  " << line.detail;
        }
    }
    out << "  (" << std::fixed << std::setprecision(1) << line.durationMs << " ms)";
    std::cout << out.str() << std::endl;
}

void runReported(const std::string& command, const std::string& ecuLabel,
                 const std::vector<uint8_t>& request,
                 const std::function<std::vector<uint8_t>()>& action,
                 const std::function<std::string(const std::vector<uint8_t>&)>& describe)
{
    const auto start = std::chrono::steady_clock::now();
    const auto elapsedMs = [&]() {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - start).count();
    };
    ResultLine line;
    line.command = command;
    line.ecu = ecuLabel;
    line.request = request;
    try {
        line.response = action();
        if (describe) {
            line.detail = describe(line.response);
        }
        line.durationMs = elapsedMs();
        printResultLine(line);
    }
    catch (const common::UDSError& ex) {
        // Include the NRC code and description.
        line.ok = false;
        line.nrc = ex.getErrorCode();
        line.detail = ex.what();
        line.durationMs = elapsedMs();
        printResultLine(line);
        throw;
    }
    catch (const std::exception& ex) {
        line.ok = false;
        line.detail = ex.what();
        line.durationMs = elapsedMs();
        printResultLine(line);
        throw;
    }
}

void writeFrame(std::ostream& output, std::chrono::milliseconds elapsed, const PASSTHRU_MSG& msg)
{
    const auto canId = common::canIdFromFrame(msg);
    output << elapsed.count() << ",0x" << std::hex << canId << std::dec << "," << (msg.DataSize - 4) << ",";
    output << std::hex << std::setfill('0');
    for (size_t i = 4; i < msg.DataSize; ++i) {
        if (i > 4) {
            output << ' ';
        }
        output << std::setw(2) << static_cast<unsigned>(msg.Data[i]);
    }
    output << std::dec << std::setfill(' ') << std::endl;
}

std::vector<std::string> splitCsvLine(const std::string& line)
{
    std::vector<std::string> result;
    std::stringstream ss{line};
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        result.push_back(common::trim(cell));
    }
    if (!line.empty() && line.back() == ',') {
        result.push_back(std::string{});
    }
    return result;
}

uint64_t parseDecimalU64(const std::string& input, const std::string& fieldName)
{
    const auto trimmed = common::trim(input);
    // std::stoull accepts a leading '-' and wraps around: a typo like time_ms=-1 must be
    // rejected, not become a 584-million-year sleep.
    if (trimmed.empty() || trimmed.front() == '-' || trimmed.front() == '+') {
        throw std::runtime_error("Invalid decimal " + fieldName + ": " + input);
    }
    size_t processedChars = 0;
    const auto value = std::stoull(trimmed, &processedChars, 10);
    if (processedChars != trimmed.size()) {
        throw std::runtime_error("Invalid decimal " + fieldName + ": " + input);
    }
    return value;
}

DiagOutput::DiagOutput(const std::string& path, const std::string& label)
{
    if (!path.empty()) {
        _file.open(path);
        if (!_file) {
            throw std::runtime_error("Failed to open " + label + ": " + path);
        }
    }
}

void DiagOutput::each(const std::function<void(std::ostream&)>& fn)
{
    fn(std::cout);
    if (_file.is_open()) {
        fn(_file);
    }
}

void DiagOutput::fileOnly(const std::function<void(std::ostream&)>& fn)
{
    if (_file.is_open()) {
        fn(_file);
    }
}

bool DiagOutput::hasFile() const
{
    return _file.is_open();
}

void DiagOutput::line(const std::string& text)
{
    each([&text](std::ostream& os) { os << text << std::endl; });
}

} // namespace volvodiag
