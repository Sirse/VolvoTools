#include "CliParser.hpp"

#include "DiagContext.hpp"

#include <common/Util.hpp>

#include <argparse/argparse.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace volvodiag {

namespace {

uint32_t parseHexId(const std::string& input)
{
    return common::parseHexU32(input);
}

std::pair<uint32_t, uint32_t> parseHexPair(const std::string& input, char delimiter,
                                           const std::string& description)
{
    const auto separator = input.find(delimiter);
    if (separator == std::string::npos) {
        throw std::runtime_error("Invalid " + description + ": " + input);
    }
    const auto first = parseHexId(input.substr(0, separator));
    const auto second = parseHexId(input.substr(separator + 1));
    return {first, second};
}

std::pair<uint32_t, uint32_t> parseCanIdRange(const std::string& input)
{
    auto range = parseHexPair(input, '-', "CAN id range");
    if (range.first > range.second) {
        throw std::runtime_error("Invalid CAN id range, start is greater than end: " + input);
    }
    if (range.second > 0x1FFFFFFF) {
        throw std::runtime_error("CAN id range must fit in 29 bits: " + input);
    }
    return range;
}

std::pair<uint32_t, uint32_t> parseCanIdMask(const std::string& input)
{
    auto mask = parseHexPair(input, ':', "CAN id mask");
    if (mask.first > 0x1FFFFFFF || mask.second > 0x1FFFFFFF) {
        throw std::runtime_error("CAN id mask values must fit in 29 bits: " + input);
    }
    if ((mask.second & ~mask.first) != 0) {
        throw std::runtime_error("CAN id mask pattern has bits outside the mask (never matches): " + input);
    }
    return mask;
}

// Allow --debug after the subcommand; Common reads it before parsing.
void addDebugArgument(argparse::ArgumentParser& command)
{
    command.add_argument("--debug").default_value(false).implicit_value(true).nargs(0)
        .help("Enable verbose debug logging");
}

void addCommonConnectionArguments(argparse::ArgumentParser& command)
{
    command.add_argument("-d", "--device").default_value(std::string{}).help("J2534 device name substring");
    command.add_argument("-f", "--platform").default_value(std::string{"P3"}).help("Car platform, e.g. P3");
    command.add_argument("-e", "--ecu").scan<'x', unsigned>().default_value(0x10u).help("ECU id");
    command.add_argument("-b", "--baudrate").scan<'u', unsigned>().help("Override configured CAN bus speed");
    addDebugArgument(command);
}

void addPlatformConnectionArguments(argparse::ArgumentParser& command)
{
    command.add_argument("-d", "--device").default_value(std::string{}).help("J2534 device name substring");
    command.add_argument("-f", "--platform").default_value(std::string{"P3"}).help("Car platform, e.g. P3");
    command.add_argument("-b", "--baudrate").scan<'u', unsigned>().help("Override configured CAN bus speed");
    addDebugArgument(command);
}

void addUdsTraceArgument(argparse::ArgumentParser& command)
{
    command.add_argument("--trace").default_value(std::string{}).help("Optional UDS trace CSV path");
}

void addOutputArgument(argparse::ArgumentParser& command)
{
    command.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");
}

void addDidIdsFromArgument(argparse::ArgumentParser& command)
{
    command.add_argument("--ids-from").default_value(std::string{})
        .help("Read DID list from a text file (one hex DID per line)");
}

void addUdsConnectionArguments(argparse::ArgumentParser& command)
{
    addCommonConnectionArguments(command);
    addUdsTraceArgument(command);
}

void addUdsPreludeArguments(argparse::ArgumentParser& command)
{
    command.add_argument("--wake").default_value(false).implicit_value(true).nargs(0)
        .help("Send functional raw CAN wake burst on the selected ECU bus before opening UDS");
    command.add_argument("--session").default_value(std::string{"none"})
        .help("Enter diagnostic session before the command: none|default|programming|ext|extended (or hex 01-7F)");
    command.add_argument("--security").default_value(std::string{})
        .help("Unlock SecurityAccess level 1 before the command using a 5-byte Y2/PIN hex value");
}

void addKeepaliveArgument(argparse::ArgumentParser& command)
{
    command.add_argument("--keepalive").default_value(false).implicit_value(true).nargs(0)
        .help("Send TesterPresent between long requests to keep the session alive");
}

void addUdsPlatformConnectionArguments(argparse::ArgumentParser& command)
{
    addPlatformConnectionArguments(command);
    addUdsTraceArgument(command);
}

void addPlatformArgument(argparse::ArgumentParser& command)
{
    command.add_argument("-f", "--platform").default_value(std::string{"P3"}).help("Car platform, e.g. P3");
    addDebugArgument(command);
}

void addDtcArguments(argparse::ArgumentParser& command)
{
    addUdsConnectionArguments(command);
    command.add_argument("--all").default_value(false).implicit_value(true).nargs(0)
        .help("Process all configured UDS ECUs on the platform");
}

void addDtcReadOnlyArguments(argparse::ArgumentParser& command)
{
    addDtcArguments(command);
    addUdsPreludeArguments(command);
}

uint32_t parseDtcNumber(const std::string& input)
{
    const auto value = parseHexId(input);
    if (value > 0xFFFFFF) {
        throw std::runtime_error("DTC must fit in three bytes: " + input);
    }
    return value;
}

uint8_t parseSessionType(const std::string& input)
{
    const auto value = common::toLower(input);
    if (value == "default") {
        return 0x01;
    }
    if (value == "programming") {
        return 0x02;
    }
    if (value == "ext" || value == "extended") {
        return 0x03;
    }
    const auto numeric = parseHexId(value);
    if (numeric == 0 || numeric > 0x7F) {
        throw std::runtime_error("--type must be default|programming|ext|extended or a hex byte 01-7F");
    }
    return static_cast<uint8_t>(numeric);
}

uint8_t parseRoutineSubFunction(const std::string& input)
{
    const auto value = common::toLower(input);
    if (value == "start") {
        return 0x01;
    }
    if (value == "stop") {
        return 0x02;
    }
    if (value == "results") {
        return 0x03;
    }
    const auto numeric = parseHexId(value);
    if (numeric == 0 || numeric > 0x7F) {
        throw std::runtime_error("--sub must be start|stop|results or a hex byte 01-7F");
    }
    return static_cast<uint8_t>(numeric);
}

// Session option limited to default and extended (10 03); "extended" aliases "ext".
bool parseSessionFlag(const std::string& input)
{
    const auto value = common::toLower(input);
    if (value == "default") {
        return false;
    }
    if (value == "ext" || value == "extended") {
        return true;
    }
    throw std::runtime_error("--session must be \"default\" or \"ext\"");
}

uint8_t parseResetType(const std::string& input)
{
    const auto value = common::toLower(input);
    if (value == "hard") {
        return 0x01;
    }
    if (value == "keyoffon") {
        return 0x02;
    }
    if (value == "soft") {
        return 0x03;
    }
    unsigned type = 0;
    try {
        type = parseHexId(value);
    } catch (const std::exception&) {
        throw std::runtime_error("--type must be hard|keyoffon|soft or a single hex byte");
    }
    if (type == 0 || type > 0xFF) {
        throw std::runtime_error("--type must be hard|keyoffon|soft or a single hex byte");
    }
    return static_cast<uint8_t>(type);
}

void addDtcRecordArguments(argparse::ArgumentParser& command)
{
    addDtcReadOnlyArguments(command);
    addKeepaliveArgument(command);
    command.add_argument("--dtc").append()
        .help("Target DTC as raw 3-byte hex, e.g. 012345 (repeatable)");
    command.add_argument("--all-dtcs").default_value(false).implicit_value(true).nargs(0)
        .help("Query every DTC reported by 19 02 first");
    command.add_argument("--record").scan<'x', unsigned>().default_value(0xFFu)
        .help("Record number (default FF = all records)");
    command.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");
}

void addRawFilterArguments(argparse::ArgumentParser& command)
{
    command.add_argument("--filter-id").append().help("Raw CAN id RX filter, repeatable hex value, e.g. 7E8");
    command.add_argument("--filter-range").append().help("Raw CAN id range RX filter, repeatable, e.g. 700-7FF");
    command.add_argument("--filter-mask").append().help("Raw CAN mask RX filter, repeatable MASK:PATTERN, e.g. 7F0:7E0");
}

void addTxEchoFilterArguments(argparse::ArgumentParser& command, bool allowExplicitIds)
{
    command.add_argument("--drop-tx-echo").default_value(false).implicit_value(true).nargs(0)
        .help("Drop frames whose CAN id matches transmitted raw CAN frame ids");
    if (allowExplicitIds) {
        command.add_argument("--tx-id").append()
            .help("Transmitted CAN id to drop when --drop-tx-echo is set, repeatable hex value");
    }
}

void readCommonConnectionArguments(const argparse::ArgumentParser& command, RunOptions& options)
{
    options.deviceName = command.get<std::string>("-d");
    options.platformName = command.get<std::string>("-f");
    options.carPlatform = common::parseCarPlatform(options.platformName);
    const auto ecuId = command.get<unsigned>("-e");
    if (ecuId > 0xFF) {
        throw std::runtime_error("ECU id must fit in one byte");
    }
    options.ecuId = static_cast<uint8_t>(ecuId);
    options.baudrateOverride = command.is_used("-b")
        ? std::optional<uint32_t>{command.get<unsigned>("-b")}
        : std::nullopt;
}

void readPlatformConnectionArguments(const argparse::ArgumentParser& command, RunOptions& options)
{
    options.deviceName = command.get<std::string>("-d");
    options.platformName = command.get<std::string>("-f");
    options.carPlatform = common::parseCarPlatform(options.platformName);
    options.baudrateOverride = command.is_used("-b")
        ? std::optional<uint32_t>{command.get<unsigned>("-b")}
        : std::nullopt;
}

void readUdsConnectionArguments(const argparse::ArgumentParser& command, RunOptions& options)
{
    readCommonConnectionArguments(command, options);
    options.tracePath = command.get<std::string>("--trace");
}

void readUdsPreludeArguments(const argparse::ArgumentParser& command, RunOptions& options)
{
    options.preludeWake = command.get<bool>("--wake");
    const auto session = common::toLower(command.get<std::string>("--session"));
    if (session == "none") {
        options.preludeSessionType = 0x00;
    } else {
        options.preludeSessionType = parseSessionType(session);
    }
    const auto security = command.get<std::string>("--security");
    if (!security.empty()) {
        options.preludeSecurityKey = common::parseHexBytes(security);
        if (options.preludeSecurityKey.size() != 5) {
            throw std::runtime_error("--security must contain exactly 5 bytes");
        }
    }
}

void readUdsPlatformConnectionArguments(const argparse::ArgumentParser& command, RunOptions& options)
{
    readPlatformConnectionArguments(command, options);
    options.tracePath = command.get<std::string>("--trace");
}

void readBusSelectionArgument(const argparse::ArgumentParser& command, RunOptions& options)
{
    if (command.is_used("--bus")) {
        options.busName = command.get<std::string>("--bus");
    }
}

void readDtcArguments(const argparse::ArgumentParser& command, RunOptions& options)
{
    readUdsConnectionArguments(command, options);
    options.allEcus = command.get<bool>("--all");
}

void readDtcReadOnlyArguments(const argparse::ArgumentParser& command, RunOptions& options)
{
    readDtcArguments(command, options);
    readUdsPreludeArguments(command, options);
}

void loadDidIdsFromFile(const std::string& path, std::vector<uint16_t>& didIds)
{
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open DID list file: " + path);
    }

    std::string line;
    while (std::getline(input, line)) {
        const auto comment = line.find_first_of("#;");
        if (comment != std::string::npos) {
            line = line.substr(0, comment);
        }
        line.erase(std::remove_if(line.begin(), line.end(),
            [](unsigned char ch) { return std::isspace(ch) != 0; }), line.end());
        if (line.empty()) {
            continue;
        }
        const auto value = parseHexId(line);
        if (value > 0xFFFF) {
            throw std::runtime_error("DID must fit in two bytes in " + path + ": " + line);
        }
        didIds.push_back(static_cast<uint16_t>(value));
    }
}

void readDtcRecordArguments(const argparse::ArgumentParser& command, RunOptions& options)
{
    readDtcReadOnlyArguments(command, options);
    if (command.is_used("--dtc")) {
        for (const auto& dtc : command.get<std::vector<std::string>>("--dtc")) {
            options.dtcTargets.push_back(parseDtcNumber(dtc));
        }
    }
    options.dtcAllStored = command.get<bool>("--all-dtcs");
    const auto record = command.get<unsigned>("--record");
    if (record > 0xFF) {
        throw std::runtime_error("--record must fit in one byte");
    }
    options.dtcRecordNumber = static_cast<uint8_t>(record);
    options.dtcRecordOutputPath = command.get<std::string>("-o");
    if (options.dtcTargets.empty() && !options.dtcAllStored) {
        throw std::runtime_error("Specify --dtc <hex> or --all-dtcs");
    }
}

void readRawFilterArguments(const argparse::ArgumentParser& command, RunOptions& options)
{
    if (command.is_used("--filter-id")) {
        for (const auto& id : command.get<std::vector<std::string>>("--filter-id")) {
            const auto canId = parseHexId(id);
            ensureCanIdFits(canId, "--filter-id");
            options.rawFilters.ids.push_back(canId);
        }
    }
    if (command.is_used("--filter-range")) {
        for (const auto& range : command.get<std::vector<std::string>>("--filter-range")) {
            options.rawFilters.ranges.push_back(parseCanIdRange(range));
        }
    }
    if (command.is_used("--filter-mask")) {
        for (const auto& mask : command.get<std::vector<std::string>>("--filter-mask")) {
            options.rawFilters.masks.push_back(parseCanIdMask(mask));
        }
    }
}

void readTxEchoFilterArguments(const argparse::ArgumentParser& command, RunOptions& options, bool allowExplicitIds)
{
    options.txEchoFilter.enabled = command.get<bool>("--drop-tx-echo");
    if (allowExplicitIds && command.is_used("--tx-id")) {
        for (const auto& id : command.get<std::vector<std::string>>("--tx-id")) {
            const auto canId = parseHexId(id);
            ensureCanIdFits(canId, "--tx-id");
            options.txEchoFilter.ids.push_back(canId);
        }
    }
}

void readRoutineCommandArguments(const argparse::ArgumentParser& command, RunOptions& options,
                                 uint8_t subFunction, bool requiresConfirmation)
{
    readUdsConnectionArguments(command, options);
    if (subFunction == 0x03) {
        readUdsPreludeArguments(command, options);
    }
    const auto routineId = parseHexId(command.get<std::string>("--id"));
    if (routineId > 0xFFFF) {
        throw std::runtime_error("Routine id must fit in two bytes");
    }
    options.routineId = static_cast<uint16_t>(routineId);
    options.routineSubFunction = subFunction;
    const auto data = command.get<std::string>("--data");
    if (!data.empty()) {
        options.routineData = common::parseHexBytes(data);
    }
    if (subFunction != 0x03) {
        options.routineExtendedSession = parseSessionFlag(command.get<std::string>("--session"));
    }
    if (subFunction != 0x03) {
        options.routineSuppress = command.get<bool>("--suppress");
    }
    options.timeoutMs = command.get<size_t>("--timeout-ms");
    options.confirmDestructive = requiresConfirmation ? command.get<bool>("--yes") : true;
}

} // namespace

bool parseOptions(int argc, const char* argv[], RunOptions& options)
{
    argparse::ArgumentParser program("VolvoDiag");
    program.add_description("Volvo J2534 diagnostic utility");
    program.add_epilog(
        "Examples:\n"
        "  Read-only:\n"
        "    VolvoDiag list-devices\n"
        "    VolvoDiag ident --platform P3 --ecu 0x10\n"
        "    VolvoDiag did read --platform P3 --ecu 0x10 --id F190\n"
        "    VolvoDiag send --platform P3 --ecu 0x10 --data \"22 F1 90\" --expect \"62 F1 90\"\n"
        "  State-changing (no --yes needed):\n"
        "    VolvoDiag session --platform P3 --ecu 0x10 --type 03\n"
        "    VolvoDiag send --platform P3 --ecu 0x10 --data \"10 03\" --expect-nrc 0x33\n"
        "    VolvoDiag send --platform P3 --ecu 0x10 --data \"10 02\" --expect-timeout\n"
        "  Write/erase (require --yes):\n"
        "    VolvoDiag reset --platform P3 --ecu 0x10 --type 01 --yes\n"
        "    VolvoDiag dtc clear --platform P3 --ecu 0x10 --yes\n"
        "  Raw CAN:\n"
        "    VolvoDiag can request --platform P3 --can-id 7E0 --data \"02 10 03\"\n"
        "    VolvoDiag monitor --platform P3 --bus \"CAN HS\"\n"
        "\n"
        "Exit codes:\n"
        "  0  success (all assertions passed)\n"
        "  1  generic failure\n"
        "  2  invalid CLI usage\n"
        "  3  J2534 device/channel error\n"
        "  4  transport timeout (no response)\n"
        "  5  ECU negative response (UDS NRC)\n"
        "  6  validation/expect failure");
    program.add_argument("--debug").default_value(false).implicit_value(true).nargs(0)
        .help("Enable verbose debug logging");

    argparse::ArgumentParser listDevicesCommand("list-devices");
    listDevicesCommand.add_description("List available J2534 devices");
    addDebugArgument(listDevicesCommand);

    argparse::ArgumentParser busesCommand("buses");
    busesCommand.add_description("List configured CAN buses and ECUs for a platform");
    addPlatformArgument(busesCommand);

    argparse::ArgumentParser identCommand("ident");
    identCommand.add_description("Read basic ECU identification data");
    addUdsConnectionArguments(identCommand);
    addUdsPreludeArguments(identCommand);
    addOutputArgument(identCommand);

    argparse::ArgumentParser sendCommand("send");
    sendCommand.add_description("Send raw UDS request bytes to ECU and print response");
    addUdsConnectionArguments(sendCommand);
    addUdsPreludeArguments(sendCommand);
    sendCommand.add_argument("--data").required().help("Hex bytes, e.g. \"22 F1 90\"");
    sendCommand.add_argument("--expect").default_value(std::string{}).help("Expected UDS payload prefix, e.g. \"62 F1 90\"");
    sendCommand.add_argument("--expect-nrc").default_value(std::string{}).help("Expect this negative response code, e.g. 0x33");
    sendCommand.add_argument("--expect-timeout").default_value(false).implicit_value(true).nargs(0).help("Expect no response (RX timeout)");
    sendCommand.add_argument("--repeat").scan<'u', size_t>().default_value(size_t{1}).help("Send request N times");
    sendCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000}).help("UDS response timeout");
    sendCommand.add_argument("--format").default_value(std::string{"raw"}).help("Response output format: raw or payload");

    argparse::ArgumentParser canCommand("can");
    canCommand.add_description("Raw CAN frame send / request / periodic / replay");

    argparse::ArgumentParser canSendCommand("send");
    canSendCommand.add_description("Send one raw CAN frame on selected bus");
    addCommonConnectionArguments(canSendCommand);
    canSendCommand.add_argument("--bus").help("Bus name override, e.g. \"CAN HS\" or \"CAN MS\"");
    canSendCommand.add_argument("--can-id").required().help("CAN id (hex), e.g. 7E0");
    canSendCommand.add_argument("--data").required().help("CAN data bytes, e.g. \"01 02 03 04\"");
    canSendCommand.add_argument("--repeat").scan<'u', size_t>().default_value(size_t{1}).help("Send frame N times");
    canSendCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000}).help("J2534 write timeout");
    addTxEchoFilterArguments(canSendCommand, false);

    argparse::ArgumentParser canRequestCommand("request");
    canRequestCommand.add_description("Send one raw CAN frame and read matching raw CAN responses");
    addCommonConnectionArguments(canRequestCommand);
    canRequestCommand.add_argument("--bus").help("Bus name override, e.g. \"CAN HS\" or \"CAN MS\"");
    canRequestCommand.add_argument("--can-id").required().help("Transmit CAN id (hex), e.g. 7E0");
    canRequestCommand.add_argument("--data").required().help("CAN data bytes, e.g. \"02 10 03\"");
    canRequestCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000}).help("J2534 write timeout");
    canRequestCommand.add_argument("--response-timeout-ms").scan<'u', size_t>().default_value(size_t{1000})
        .help("Total response read timeout after transmit");
    canRequestCommand.add_argument("--count").scan<'u', size_t>().default_value(size_t{1})
        .help("Stop after N matching response frames");
    addRawFilterArguments(canRequestCommand);
    addTxEchoFilterArguments(canRequestCommand, false);
    canRequestCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");

    argparse::ArgumentParser canPeriodicCommand("periodic");
    canPeriodicCommand.add_description("Send one raw CAN frame periodically");
    addCommonConnectionArguments(canPeriodicCommand);
    canPeriodicCommand.add_argument("--bus").help("Bus name override, e.g. \"CAN HS\" or \"CAN MS\"");
    canPeriodicCommand.add_argument("--can-id").required().help("CAN id (hex), e.g. 7E0");
    canPeriodicCommand.add_argument("--data").required().help("CAN data bytes, e.g. \"01 02 03 04\"");
    canPeriodicCommand.add_argument("--interval-ms").scan<'u', size_t>().default_value(size_t{1000}).help("Send interval");
    canPeriodicCommand.add_argument("--count").scan<'u', size_t>().default_value(size_t{0}).help("Stop after N frames, 0 means unlimited");
    canPeriodicCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000}).help("J2534 write timeout");

    argparse::ArgumentParser canReplayCommand("replay");
    canReplayCommand.add_description("Replay raw CAN frames from monitor CSV");
    addCommonConnectionArguments(canReplayCommand);
    canReplayCommand.add_argument("--bus").help("Bus name override, e.g. \"CAN HS\" or \"CAN MS\"");
    canReplayCommand.add_argument("-i", "--input").required().help("CSV input path with time_ms,can_id,dlc,data");
    canReplayCommand.add_argument("--no-timing").default_value(false).implicit_value(true).nargs(0)
        .help("Replay frames without preserving time_ms deltas");
    canReplayCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000}).help("J2534 write timeout");

    canCommand.add_subparser(canSendCommand);
    canCommand.add_subparser(canRequestCommand);
    canCommand.add_subparser(canPeriodicCommand);
    canCommand.add_subparser(canReplayCommand);

    argparse::ArgumentParser probeCommand("probe");
    probeCommand.add_description("Probe raw CAN request identifiers with a single-frame UDS request");
    addCommonConnectionArguments(probeCommand);
    probeCommand.add_argument("--bus").help("Bus name override, e.g. \"CAN HS\" or \"CAN MS\"");
    probeCommand.add_argument("--from").scan<'x', unsigned>().default_value(0x700u).help("First request CAN id");
    probeCommand.add_argument("--to").scan<'x', unsigned>().default_value(0x7FFu).help("Last request CAN id");
    probeCommand.add_argument("--data").default_value(std::string{"22 F1 90"})
        .help("UDS payload bytes to wrap as an ISO-TP single frame");
    probeCommand.add_argument("--response-offset").scan<'x', unsigned>().default_value(0x8u)
        .help("Expected response id offset from request id");
    probeCommand.add_argument("--response-range").help("Response CAN id range override, e.g. 700-7FF");
    probeCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{100})
        .help("Response timeout per request id");
    probeCommand.add_argument("--gap-ms").scan<'u', size_t>().default_value(size_t{0})
        .help("Delay between probes");
    probeCommand.add_argument("--reassemble").default_value(false).implicit_value(true).nargs(0)
        .help("Reassemble ISO-TP responses before writing output");
    addTxEchoFilterArguments(probeCommand, false);
    probeCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");

    argparse::ArgumentParser wakeCommand("wake");
    wakeCommand.add_description("Wake Volvo modules with a functional raw CAN diagnostic broadcast");
    addCommonConnectionArguments(wakeCommand);
    wakeCommand.add_argument("--bus").help("Bus name override, e.g. \"CAN HS\" or \"CAN MS\"");
    wakeCommand.add_argument("--can-id").default_value(std::string{"7DF"})
        .help("Functional wake CAN id (hex)");
    wakeCommand.add_argument("--data").default_value(std::string{"10 82"})
        .help("UDS wake payload to wrap as ISO-TP single frame");
    wakeCommand.add_argument("--count").scan<'u', size_t>().default_value(size_t{10})
        .help("Wake burst frame count");
    wakeCommand.add_argument("--gap-ms").scan<'u', size_t>().default_value(size_t{20})
        .help("Delay between wake burst frames");
    wakeCommand.add_argument("--no-hold").default_value(false).implicit_value(true).nargs(0)
        .help("Do not start periodic hold frame after the wake burst");
    wakeCommand.add_argument("--hold-data").default_value(std::string{"3E 80"})
        .help("UDS hold payload to wrap as ISO-TP single frame");
    wakeCommand.add_argument("--interval-ms").scan<'u', size_t>().default_value(size_t{1000})
        .help("Hold frame interval");
    wakeCommand.add_argument("--hold-ms").scan<'u', size_t>().default_value(size_t{0})
        .help("Hold duration, 0 means until Ctrl-C");
    wakeCommand.add_argument("--teardown").default_value(false).implicit_value(true).nargs(0)
        .help("Send teardown frame after hold exits");
    wakeCommand.add_argument("--teardown-data").default_value(std::string{"11 81"})
        .help("UDS teardown payload to wrap as ISO-TP single frame");
    wakeCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000})
        .help("J2534 write timeout");

    argparse::ArgumentParser didCommand("did");
    didCommand.add_description("Read, write or scan DataByIdentifier values");

    argparse::ArgumentParser didReadCommand("read");
    didReadCommand.add_description("Read and decode DataByIdentifier values (UDS 22)");
    addUdsConnectionArguments(didReadCommand);
    addUdsPreludeArguments(didReadCommand);
    didReadCommand.add_argument("--id").append()
        .help("DID to read, repeatable hex value, e.g. F190");
    addDidIdsFromArgument(didReadCommand);
    addOutputArgument(didReadCommand);

    argparse::ArgumentParser didWriteCommand("write");
    didWriteCommand.add_description("Write a DataByIdentifier value (UDS 2E)");
    addUdsConnectionArguments(didWriteCommand);
    didWriteCommand.add_argument("--id").required().help("DID to write (hex), e.g. F1A0");
    didWriteCommand.add_argument("--data").required().help("Hex bytes to write, e.g. \"01 02 03\"");
    didWriteCommand.add_argument("--session").default_value(std::string{"default"})
        .help("Diagnostic session: default|ext|extended (enter 10 03 before writing)");
    didWriteCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000})
        .help("UDS response timeout");
    didWriteCommand.add_argument("--yes").default_value(false).implicit_value(true).nargs(0)
        .help("Confirm the write (modifies ECU memory)");

    argparse::ArgumentParser didScanCommand("scan");
    didScanCommand.add_description("Scan a DID range with UDS ReadDataByIdentifier (22)");
    addUdsConnectionArguments(didScanCommand);
    addUdsPreludeArguments(didScanCommand);
    addKeepaliveArgument(didScanCommand);
    didScanCommand.add_argument("--from").scan<'x', unsigned>().default_value(0x0000u)
        .help("First DID to scan");
    didScanCommand.add_argument("--to").scan<'x', unsigned>().default_value(0xFFFFu)
        .help("Last DID to scan");
    didScanCommand.add_argument("--gap-ms").scan<'u', size_t>().default_value(size_t{0})
        .help("Delay between DID requests");
    didScanCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{100})
        .help("Response timeout per DID");
    didScanCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");

    didCommand.add_subparser(didReadCommand);
    didCommand.add_subparser(didWriteCommand);
    didCommand.add_subparser(didScanCommand);

    argparse::ArgumentParser scanCommand("ecu-scan");
    scanCommand.add_description("Probe configured ECUs on a UDS platform");
    addUdsPlatformConnectionArguments(scanCommand);
    scanCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");

    argparse::ArgumentParser sessionCommand("session");
    sessionCommand.add_description("Send UDS DiagnosticSessionControl (10); session lasts only while the ECU S3 timer allows");
    addUdsConnectionArguments(sessionCommand);
    sessionCommand.add_argument("--type").default_value(std::string{"ext"})
        .help("Session type: default|programming|ext|extended (or hex 01-7F)");
    sessionCommand.add_argument("--suppress").default_value(false).implicit_value(true).nargs(0)
        .help("Set suppressPositiveResponse bit (0x80) and do not wait for a reply");
    sessionCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000})
        .help("UDS response timeout");

    argparse::ArgumentParser testerPresentCommand("tester-present");
    testerPresentCommand.add_description("Send UDS TesterPresent (3E), optionally keeping the current session alive while this process runs");
    addUdsConnectionArguments(testerPresentCommand);
    testerPresentCommand.add_argument("--suppress").default_value(false).implicit_value(true).nargs(0)
        .help("Set suppressPositiveResponse bit (0x80) and do not wait for a reply");
    testerPresentCommand.add_argument("--interval-ms").scan<'u', size_t>().default_value(size_t{2000})
        .help("Delay between TesterPresent frames when --count is not 1");
    testerPresentCommand.add_argument("--count").scan<'u', size_t>().default_value(size_t{1})
        .help("Send N frames, 0 means keep sending until Ctrl-C");
    testerPresentCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000})
        .help("UDS response timeout");

    argparse::ArgumentParser resetCommand("reset");
    resetCommand.add_description("Reset an ECU using UDS ECUReset (11)");
    addUdsConnectionArguments(resetCommand);
    resetCommand.add_argument("--type").default_value(std::string{"hard"})
        .help("Reset type: hard|keyoffon|soft (or hex 01/02/03)");
    resetCommand.add_argument("--functional").default_value(false).implicit_value(true).nargs(0)
        .help("Broadcast to functional 0x7DF (reset all modules on the bus) instead of one ECU");
    resetCommand.add_argument("--bus")
        .help("Restrict functional reset to one bus by name, e.g. \"CAN HS\" (default: all UDS buses)");
    resetCommand.add_argument("--suppress").default_value(false).implicit_value(true).nargs(0)
        .help("Set suppressPositiveResponse bit (0x80) and do not wait for a reply");
    resetCommand.add_argument("--yes").default_value(false).implicit_value(true).nargs(0)
        .help("Confirm ECU reset");

    argparse::ArgumentParser routineCommand("routine");
    routineCommand.add_description("Run one UDS RoutineControl request (31)");

    argparse::ArgumentParser routineStartCommand("start");
    routineStartCommand.add_description("Start one routine by id (31 01)");
    addUdsConnectionArguments(routineStartCommand);
    routineStartCommand.add_argument("--id").required().help("Routine id (hex), e.g. FF00");
    routineStartCommand.add_argument("--data").default_value(std::string{})
        .help("Optional routine control option bytes, e.g. \"01 02\"");
    routineStartCommand.add_argument("--session").default_value(std::string{"default"})
        .help("Diagnostic session: default|ext|extended (enter 10 03 before routine)");
    routineStartCommand.add_argument("--suppress").default_value(false).implicit_value(true).nargs(0)
        .help("Set suppressPositiveResponse bit (0x80) and do not wait for a reply");
    routineStartCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000})
        .help("UDS response timeout");
    routineStartCommand.add_argument("--yes").default_value(false).implicit_value(true).nargs(0)
        .help("Confirm starting a routine on the ECU");

    argparse::ArgumentParser routineStopCommand("stop");
    routineStopCommand.add_description("Stop one routine by id (31 02)");
    addUdsConnectionArguments(routineStopCommand);
    routineStopCommand.add_argument("--id").required().help("Routine id (hex), e.g. FF00");
    routineStopCommand.add_argument("--data").default_value(std::string{})
        .help("Optional routine control option bytes, e.g. \"01 02\"");
    routineStopCommand.add_argument("--session").default_value(std::string{"default"})
        .help("Diagnostic session: default|ext|extended (enter 10 03 before routine)");
    routineStopCommand.add_argument("--suppress").default_value(false).implicit_value(true).nargs(0)
        .help("Set suppressPositiveResponse bit (0x80) and do not wait for a reply");
    routineStopCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000})
        .help("UDS response timeout");
    routineStopCommand.add_argument("--yes").default_value(false).implicit_value(true).nargs(0)
        .help("Confirm stopping a routine on the ECU");

    argparse::ArgumentParser routineResultsCommand("results");
    routineResultsCommand.add_description("Request one routine result by id (31 03)");
    addUdsConnectionArguments(routineResultsCommand);
    addUdsPreludeArguments(routineResultsCommand);
    routineResultsCommand.add_argument("--id").required().help("Routine id (hex), e.g. FF00");
    routineResultsCommand.add_argument("--data").default_value(std::string{})
        .help("Optional routine control option bytes, e.g. \"01 02\"");
    routineResultsCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000})
        .help("UDS response timeout");

    routineCommand.add_subparser(routineStartCommand);
    routineCommand.add_subparser(routineStopCommand);
    routineCommand.add_subparser(routineResultsCommand);

    argparse::ArgumentParser routineScanCommand("routine-scan");
    routineScanCommand.add_description("Scan a routine-id range with UDS RoutineControl (31)");
    addUdsConnectionArguments(routineScanCommand);
    routineScanCommand.add_argument("--from").scan<'x', unsigned>().default_value(0x0000u)
        .help("First routine id");
    routineScanCommand.add_argument("--to").scan<'x', unsigned>().default_value(0xFFFFu)
        .help("Last routine id");
    routineScanCommand.add_argument("--sub").default_value(std::string{"start"})
        .help("Subfunction: start|stop|results (or hex 01/02/03)");
    routineScanCommand.add_argument("--session").default_value(std::string{"default"})
        .help("Diagnostic session: default|ext|extended (enter + keep alive 10 03 during scan)");
    addKeepaliveArgument(routineScanCommand);
    routineScanCommand.add_argument("--gap-ms").scan<'u', size_t>().default_value(size_t{0})
        .help("Delay between routine requests");
    routineScanCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{100})
        .help("Response timeout per routine");
    routineScanCommand.add_argument("--yes").default_value(false).implicit_value(true).nargs(0)
        .help("Confirm scanning starts routines on the ECU");
    routineScanCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");

    argparse::ArgumentParser monitorCommand("monitor");
    monitorCommand.add_description("Monitor raw CAN frames on selected bus");
    addCommonConnectionArguments(monitorCommand);
    monitorCommand.add_argument("--bus").help("Bus name override, e.g. \"CAN HS\" or \"CAN MS\"");
    addRawFilterArguments(monitorCommand);
    addTxEchoFilterArguments(monitorCommand, true);
    monitorCommand.add_argument("--count").scan<'u', size_t>().default_value(size_t{0}).help("Stop after N matching frames, 0 means unlimited");
    monitorCommand.add_argument("--duration-ms").scan<'u', size_t>().default_value(size_t{0})
        .help("Stop after N milliseconds, 0 means unlimited");
    monitorCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{0})
        .help("Alias for --duration-ms");
    monitorCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");
    monitorCommand.add_argument("--baseline-record").default_value(std::string{}).help("Record seen frame keys to this file (snapshot)");
    monitorCommand.add_argument("--baseline-compare").default_value(std::string{}).help("Print only frames whose key is absent from this baseline file");
    monitorCommand.add_argument("--baseline-key").default_value(std::string{"full"}).help("Frame identity for baseline: \"id\" (CAN ID) or \"full\" (CAN ID + data)");

    argparse::ArgumentParser reconCaptureCommand("recon-capture");
    reconCaptureCommand.add_description("Capture parameterised raw frames while running a UDS RAM stub");
    addCommonConnectionArguments(reconCaptureCommand);
    reconCaptureCommand.add_argument("--bus").default_value(std::string{}).help("Raw CAN bus override");
    reconCaptureCommand.add_argument("--raw-protocol").default_value(std::string{"can"})
        .help("Raw channel protocol: can|can-ps|auto");
    reconCaptureCommand.add_argument("--req-id").required().help("ISO-TP request CAN id");
    reconCaptureCommand.add_argument("--rsp-id").required().help("ISO-TP response CAN id");
    reconCaptureCommand.add_argument("--isotp-padding").default_value(std::string{"00"}).help("ISO-TP TX padding byte");
    reconCaptureCommand.add_argument("--isotp-pad-to-8").default_value(false).implicit_value(true).nargs(0)
        .help("Pad ISO-TP TX frames to DLC 8 (disabled by default for Mongoose raw CAN)");
    reconCaptureCommand.add_argument("--isotp-bs").scan<'x', unsigned>().default_value(0u).help("ISO-TP receive Flow Control block size");
    reconCaptureCommand.add_argument("--isotp-stmin").scan<'x', unsigned>().default_value(0u).help("ISO-TP receive Flow Control STmin");
    reconCaptureCommand.add_argument("--capture-can-id").required().help("Raw CAN capture frame id");
    reconCaptureCommand.add_argument("--capture-marker").required().help("Raw frame marker/signature bytes");
    reconCaptureCommand.add_argument("--trigger-routine").required().help("Routine RID and data, e.g. 0301:00001400");
    reconCaptureCommand.add_argument("--load-stub-bin").default_value(std::string{}).help("Raw stub binary; omit when already resident");
    reconCaptureCommand.add_argument("--stub-sha256").default_value(std::string{}).help("Expected SHA-256 for loaded stub");
    reconCaptureCommand.add_argument("--load-address").scan<'x', unsigned>().default_value(0x1400u).help("RAM load address");
    reconCaptureCommand.add_argument("--capture-count").scan<'u', size_t>().default_value(size_t{4}).help("Expected captured frames");
    reconCaptureCommand.add_argument("--capture-timeout-ms").scan<'u', size_t>().default_value(size_t{1000}).help("Capture timeout");
    reconCaptureCommand.add_argument("--reference-bytes").default_value(std::string{}).help("Expected reconstructed image bytes");
    reconCaptureCommand.add_argument("--reference-address").scan<'x', unsigned>().default_value(0xFFF0u).help("Reference image start address");
    reconCaptureCommand.add_argument("-o", "--out").required().help("Capture CSV output path");
    reconCaptureCommand.add_argument("--manifest").default_value(std::string{}).help("Run manifest JSON path");
    reconCaptureCommand.add_argument("--health-session").required().help("Health session request bytes");
    reconCaptureCommand.add_argument("--health-session-expect").required().help("Health session expected prefix");
    reconCaptureCommand.add_argument("--health-did").required().help("Health DID request bytes");
    reconCaptureCommand.add_argument("--health-did-expect").required().help("Health DID expected bytes/prefix");
    reconCaptureCommand.add_argument("--timeout-ms").scan<'u', size_t>().default_value(size_t{1000}).help("UDS timeout");
    reconCaptureCommand.add_argument("--smoke-only").default_value(false).implicit_value(true).nargs(0)
        .help("Open one raw channel and run only the configured ISO-TP health-session probe");

    argparse::ArgumentParser dtcCommand("dtc");
    dtcCommand.add_description("Read or clear diagnostic trouble codes");

    argparse::ArgumentParser dtcReadCommand("read");
    dtcReadCommand.add_description("Read DTCs using UDS reportDTCByStatusMask (19 02)");
    addDtcReadOnlyArguments(dtcReadCommand);
    addKeepaliveArgument(dtcReadCommand);
    dtcReadCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");
    dtcReadCommand.add_argument("--status-mask").scan<'x', unsigned>().default_value(0xFFu)
        .help("UDS status mask byte for 19 02 (default FF = all stored DTCs)");
    dtcReadCommand.add_argument("--confirmed").default_value(false).implicit_value(true).nargs(0)
        .help("Show only confirmed DTCs (status bit 3)");

    argparse::ArgumentParser dtcClearCommand("clear");
    dtcClearCommand.add_description("Clear DTCs using UDS 14 FF FF FF");
    addDtcArguments(dtcClearCommand);
    dtcClearCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");
    dtcClearCommand.add_argument("--yes").default_value(false).implicit_value(true).nargs(0)
        .help("Confirm DTC clearing");

    argparse::ArgumentParser dtcSnapshotCommand("snapshot");
    dtcSnapshotCommand.add_description("Read DTC snapshot/freeze-frame records (UDS 19 04)");
    addDtcRecordArguments(dtcSnapshotCommand);
    addKeepaliveArgument(dtcSnapshotCommand);

    argparse::ArgumentParser dtcExtendedCommand("extended");
    dtcExtendedCommand.add_description("Read DTC extended data records (UDS 19 06)");
    addDtcRecordArguments(dtcExtendedCommand);
    addKeepaliveArgument(dtcExtendedCommand);

    dtcCommand.add_subparser(dtcReadCommand);
    dtcCommand.add_subparser(dtcClearCommand);
    dtcCommand.add_subparser(dtcSnapshotCommand);
    dtcCommand.add_subparser(dtcExtendedCommand);

    argparse::ArgumentParser obdCommand("obd");
    obdCommand.add_description("Generic OBD-II services over CAN");

    argparse::ArgumentParser obdVinCommand("vin");
    obdVinCommand.add_description("Read VIN using OBD-II mode 09 PID 02");
    addUdsPlatformConnectionArguments(obdVinCommand);
    obdVinCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");

    argparse::ArgumentParser obdPidCommand("pid");
    obdPidCommand.add_description("Read one OBD-II mode 01 PID");
    addUdsPlatformConnectionArguments(obdPidCommand);
    obdPidCommand.add_argument("--id").required().help("PID byte (hex), e.g. 0C");
    obdPidCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");

    argparse::ArgumentParser obdDtcReadCommand("dtc-read");
    obdDtcReadCommand.add_description("Read OBD-II DTCs using mode 03");
    addUdsPlatformConnectionArguments(obdDtcReadCommand);
    obdDtcReadCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");

    argparse::ArgumentParser obdDtcClearCommand("dtc-clear");
    obdDtcClearCommand.add_description("Clear OBD-II DTCs using mode 04");
    addUdsPlatformConnectionArguments(obdDtcClearCommand);
    obdDtcClearCommand.add_argument("--yes").default_value(false).implicit_value(true).nargs(0)
        .help("Confirm OBD-II DTC clearing");
    obdDtcClearCommand.add_argument("-o", "--output").default_value(std::string{}).help("Optional CSV output path");

    obdCommand.add_subparser(obdVinCommand);
    obdCommand.add_subparser(obdPidCommand);
    obdCommand.add_subparser(obdDtcReadCommand);
    obdCommand.add_subparser(obdDtcClearCommand);

    argparse::ArgumentParser scriptCommand("script");
    scriptCommand.add_description("Run a TOML diagnostic scenario in one persistent session");
    scriptCommand.add_argument("--file").required().help("TOML scenario path");
    scriptCommand.add_argument("--output-dir").default_value(std::string{})
        .help("Directory for scenario artifacts");
    scriptCommand.add_argument("--trace").default_value(std::string{})
        .help("Optional UDS trace CSV path");
    scriptCommand.add_argument("-d", "--device").default_value(std::string{})
        .help("J2534 device name substring");
    scriptCommand.add_argument("-f", "--platform")
        .help("Override the platform declared by the scenario");
    scriptCommand.add_argument("-e", "--ecu").scan<'x', unsigned>()
        .help("Override the ECU id declared by the scenario");
    scriptCommand.add_argument("-b", "--baudrate").scan<'u', unsigned>()
        .help("Override configured CAN bus speed");
    scriptCommand.add_argument("--var").append()
        .help("Scenario variable override as NAME=VALUE (repeatable)");
    scriptCommand.add_argument("--dry-run").default_value(false).implicit_value(true).nargs(0)
        .help("Validate and print the scenario without opening a device");
    scriptCommand.add_argument("--yes").default_value(false).implicit_value(true).nargs(0)
        .help("Confirm scenario operations that require explicit approval");
    addDebugArgument(scriptCommand);

    program.add_subparser(listDevicesCommand);
    program.add_subparser(busesCommand);
    program.add_subparser(identCommand);
    program.add_subparser(sendCommand);
    program.add_subparser(canCommand);
    program.add_subparser(probeCommand);
    program.add_subparser(wakeCommand);
    program.add_subparser(didCommand);
    program.add_subparser(scanCommand);
    program.add_subparser(sessionCommand);
    program.add_subparser(testerPresentCommand);
    program.add_subparser(resetCommand);
    program.add_subparser(routineCommand);
    program.add_subparser(routineScanCommand);
    program.add_subparser(monitorCommand);
    program.add_subparser(reconCaptureCommand);
    program.add_subparser(dtcCommand);
    program.add_subparser(obdCommand);
    program.add_subparser(scriptCommand);

    try {
        program.parse_args(argc, argv);
        if (program.is_subcommand_used(listDevicesCommand)) {
            options.command = Command::ListDevices;
            return true;
        }
        if (program.is_subcommand_used(busesCommand)) {
            options.command = Command::Buses;
            options.platformName = busesCommand.get<std::string>("-f");
            options.carPlatform = common::parseCarPlatform(options.platformName);
            return true;
        }
        if (program.is_subcommand_used(identCommand)) {
            options.command = Command::Ident;
            readUdsConnectionArguments(identCommand, options);
            readUdsPreludeArguments(identCommand, options);
            options.identOutputPath = identCommand.get<std::string>("-o");
            return true;
        }
        if (program.is_subcommand_used(sendCommand)) {
            options.command = Command::Send;
            readUdsConnectionArguments(sendCommand, options);
            readUdsPreludeArguments(sendCommand, options);
            options.requestData = common::parseHexBytes(sendCommand.get<std::string>("--data"));
            if (!sendCommand.get<std::string>("--expect").empty()) {
                options.expectedData = common::parseHexBytes(sendCommand.get<std::string>("--expect"));
            }
            options.expectTimeout = sendCommand.get<bool>("--expect-timeout");
            if (!sendCommand.get<std::string>("--expect-nrc").empty()) {
                const auto nrc = common::parseHexU32(sendCommand.get<std::string>("--expect-nrc"));
                if (nrc == 0 || nrc > 0xFF) {
                    throw std::runtime_error("--expect-nrc must be a single byte (0x01-0xFF)");
                }
                options.expectNrc = static_cast<uint8_t>(nrc);
            }
            // Expectations are mutually exclusive.
            if (options.expectTimeout && (options.expectNrc || !options.expectedData.empty())) {
                throw std::runtime_error("--expect-timeout cannot be combined with --expect-nrc/--expect");
            }
            if (options.expectNrc && !options.expectedData.empty()) {
                throw std::runtime_error("--expect-nrc cannot be combined with --expect");
            }
            options.repeatCount = sendCommand.get<size_t>("--repeat");
            options.timeoutMs = sendCommand.get<size_t>("--timeout-ms");
            options.outputMode = common::toLower(sendCommand.get<std::string>("--format"));
            if (options.repeatCount == 0) {
                throw std::runtime_error("--repeat must be greater than zero");
            }
            if (options.outputMode != "raw" && options.outputMode != "payload") {
                throw std::runtime_error("--format must be raw or payload");
            }
            return true;
        }
        if (program.is_subcommand_used(canCommand)) {
          if (canCommand.is_subcommand_used(canSendCommand)) {
            options.command = Command::CanSend;
            readCommonConnectionArguments(canSendCommand, options);
            readBusSelectionArgument(canSendCommand, options);
            options.canId = parseHexId(canSendCommand.get<std::string>("--can-id"));
            ensureCanIdFits(options.canId, "--can-id");
            options.requestData = common::parseHexBytes(canSendCommand.get<std::string>("--data"));
            if (options.requestData.size() > 8) {
                throw std::runtime_error("Raw CAN data must be at most 8 bytes");
            }
            options.repeatCount = canSendCommand.get<size_t>("--repeat");
            options.timeoutMs = canSendCommand.get<size_t>("--timeout-ms");
            options.txEchoFilter.enabled = canSendCommand.get<bool>("--drop-tx-echo");
            if (options.repeatCount == 0) {
                throw std::runtime_error("--repeat must be greater than zero");
            }
            return true;
        }
          if (canCommand.is_subcommand_used(canRequestCommand)) {
            options.command = Command::CanRequest;
            readCommonConnectionArguments(canRequestCommand, options);
            readBusSelectionArgument(canRequestCommand, options);
            options.canId = parseHexId(canRequestCommand.get<std::string>("--can-id"));
            ensureCanIdFits(options.canId, "--can-id");
            options.requestData = common::parseHexBytes(canRequestCommand.get<std::string>("--data"));
            if (options.requestData.size() > 8) {
                throw std::runtime_error("Raw CAN data must be at most 8 bytes");
            }
            options.timeoutMs = canRequestCommand.get<size_t>("--timeout-ms");
            options.responseTimeoutMs = canRequestCommand.get<size_t>("--response-timeout-ms");
            options.responseCount = canRequestCommand.get<size_t>("--count");
            options.canOutputPath = canRequestCommand.get<std::string>("-o");
            readRawFilterArguments(canRequestCommand, options);
            readTxEchoFilterArguments(canRequestCommand, options, false);
            if (options.txEchoFilter.enabled) {
                options.txEchoFilter.ids.push_back(options.canId);
            }
            if (options.responseCount == 0) {
                throw std::runtime_error("--count must be greater than zero");
            }
            return true;
        }
          if (canCommand.is_subcommand_used(canPeriodicCommand)) {
            options.command = Command::CanPeriodic;
            readCommonConnectionArguments(canPeriodicCommand, options);
            readBusSelectionArgument(canPeriodicCommand, options);
            options.canId = parseHexId(canPeriodicCommand.get<std::string>("--can-id"));
            ensureCanIdFits(options.canId, "--can-id");
            options.requestData = common::parseHexBytes(canPeriodicCommand.get<std::string>("--data"));
            if (options.requestData.size() > 8) {
                throw std::runtime_error("Raw CAN data must be at most 8 bytes");
            }
            options.intervalMs = canPeriodicCommand.get<size_t>("--interval-ms");
            options.repeatCount = canPeriodicCommand.get<size_t>("--count");
            options.timeoutMs = canPeriodicCommand.get<size_t>("--timeout-ms");
            if (options.intervalMs == 0) {
                throw std::runtime_error("--interval-ms must be greater than zero");
            }
            return true;
        }
          if (canCommand.is_subcommand_used(canReplayCommand)) {
            options.command = Command::CanReplay;
            readCommonConnectionArguments(canReplayCommand, options);
            readBusSelectionArgument(canReplayCommand, options);
            options.replayInputPath = canReplayCommand.get<std::string>("-i");
            options.preserveReplayTiming = !canReplayCommand.get<bool>("--no-timing");
            options.timeoutMs = canReplayCommand.get<size_t>("--timeout-ms");
            return true;
          }
          std::cerr << canCommand;
          return false;
        }
        if (program.is_subcommand_used(probeCommand)) {
            options.command = Command::Probe;
            readCommonConnectionArguments(probeCommand, options);
            readBusSelectionArgument(probeCommand, options);
            options.probeFrom = probeCommand.get<unsigned>("--from");
            options.probeTo = probeCommand.get<unsigned>("--to");
            ensureCanIdFits(options.probeFrom, "--from");
            ensureCanIdFits(options.probeTo, "--to");
            if (options.probeFrom > options.probeTo) {
                throw std::runtime_error("--from must be less than or equal to --to");
            }
            options.requestData = common::parseHexBytes(probeCommand.get<std::string>("--data"));
            if (options.requestData.size() > 7) {
                throw std::runtime_error("--data must fit in one ISO-TP single frame payload (max 7 bytes)");
            }
            options.probeResponseOffset = probeCommand.get<unsigned>("--response-offset");
            ensureCanIdFits(options.probeResponseOffset, "--response-offset");
            if (probeCommand.is_used("--response-range")) {
                options.probeResponseRange = parseCanIdRange(probeCommand.get<std::string>("--response-range"));
                options.probeUseResponseRange = true;
            }
            options.timeoutMs = probeCommand.get<size_t>("--timeout-ms");
            options.probeGapMs = probeCommand.get<size_t>("--gap-ms");
            options.probeReassemble = probeCommand.get<bool>("--reassemble");
            options.probeOutputPath = probeCommand.get<std::string>("-o");
            readTxEchoFilterArguments(probeCommand, options, false);
            return true;
        }
        if (program.is_subcommand_used(wakeCommand)) {
            options.command = Command::Wake;
            readCommonConnectionArguments(wakeCommand, options);
            readBusSelectionArgument(wakeCommand, options);
            options.wakeCanId = parseHexId(wakeCommand.get<std::string>("--can-id"));
            ensureCanIdFits(options.wakeCanId, "--can-id");
            options.wakePayload = common::parseHexBytes(wakeCommand.get<std::string>("--data"));
            options.wakeHoldPayload = common::parseHexBytes(wakeCommand.get<std::string>("--hold-data"));
            options.wakeTeardownPayload = common::parseHexBytes(wakeCommand.get<std::string>("--teardown-data"));
            if (options.wakePayload.empty() || options.wakePayload.size() > 7) {
                throw std::runtime_error("--data must contain 1-7 UDS bytes");
            }
            if (options.wakeHoldPayload.empty() || options.wakeHoldPayload.size() > 7) {
                throw std::runtime_error("--hold-data must contain 1-7 UDS bytes");
            }
            if (options.wakeTeardownPayload.empty() || options.wakeTeardownPayload.size() > 7) {
                throw std::runtime_error("--teardown-data must contain 1-7 UDS bytes");
            }
            options.wakeBurstCount = wakeCommand.get<size_t>("--count");
            options.wakeGapMs = wakeCommand.get<size_t>("--gap-ms");
            options.wakeHold = !wakeCommand.get<bool>("--no-hold");
            options.intervalMs = wakeCommand.get<size_t>("--interval-ms");
            options.wakeHoldMs = wakeCommand.get<size_t>("--hold-ms");
            options.wakeTeardown = wakeCommand.get<bool>("--teardown");
            options.timeoutMs = wakeCommand.get<size_t>("--timeout-ms");
            if (options.wakeBurstCount == 0) {
                throw std::runtime_error("--count must be greater than zero");
            }
            if (options.intervalMs == 0) {
                throw std::runtime_error("--interval-ms must be greater than zero");
            }
            return true;
        }
        if (program.is_subcommand_used(didCommand)) {
          if (didCommand.is_subcommand_used(didReadCommand)) {
            options.command = Command::DidRead;
            readUdsConnectionArguments(didReadCommand, options);
            readUdsPreludeArguments(didReadCommand, options);
            options.didOutputPath = didReadCommand.get<std::string>("-o");
            if (didReadCommand.is_used("--id")) {
                for (const auto& id : didReadCommand.get<std::vector<std::string>>("--id")) {
                    const auto value = parseHexId(id);
                    if (value > 0xFFFF) {
                        throw std::runtime_error("DID must fit in two bytes: " + id);
                    }
                    options.didIds.push_back(static_cast<uint16_t>(value));
                }
            }
            if (didReadCommand.is_used("--ids-from")) {
                loadDidIdsFromFile(didReadCommand.get<std::string>("--ids-from"), options.didIds);
            }
            if (options.didIds.empty()) {
                throw std::runtime_error("Specify at least one DID with --id or --ids-from");
            }
            return true;
          }
          if (didCommand.is_subcommand_used(didWriteCommand)) {
            options.command = Command::DidWrite;
            readUdsConnectionArguments(didWriteCommand, options);
            const auto did = parseHexId(didWriteCommand.get<std::string>("--id"));
            if (did > 0xFFFF) {
                throw std::runtime_error("DID must fit in two bytes");
            }
            options.didWriteId = static_cast<uint16_t>(did);
            options.requestData = common::parseHexBytes(didWriteCommand.get<std::string>("--data"));
            if (options.requestData.empty()) {
                throw std::runtime_error("--data must contain at least one byte");
            }
            options.didWriteExtendedSession = parseSessionFlag(didWriteCommand.get<std::string>("--session"));
            options.timeoutMs = didWriteCommand.get<size_t>("--timeout-ms");
            options.confirmDestructive = didWriteCommand.get<bool>("--yes");
            return true;
          }
          if (didCommand.is_subcommand_used(didScanCommand)) {
            options.command = Command::DidScan;
            readUdsConnectionArguments(didScanCommand, options);
            readUdsPreludeArguments(didScanCommand, options);
            const auto from = didScanCommand.get<unsigned>("--from");
            const auto to = didScanCommand.get<unsigned>("--to");
            if (from > 0xFFFF || to > 0xFFFF) {
                throw std::runtime_error("DID scan bounds must fit in two bytes");
            }
            if (from > to) {
                throw std::runtime_error("--from must be less than or equal to --to");
            }
            options.didScanFrom = static_cast<uint16_t>(from);
            options.didScanTo = static_cast<uint16_t>(to);
            options.didScanGapMs = didScanCommand.get<size_t>("--gap-ms");
            options.keepalive = didScanCommand.get<bool>("--keepalive");
            options.timeoutMs = didScanCommand.get<size_t>("--timeout-ms");
            options.didScanOutputPath = didScanCommand.get<std::string>("-o");
            return true;
          }
          std::cerr << didCommand;
          return false;
        }
        if (program.is_subcommand_used(scanCommand)) {
            options.command = Command::Scan;
            readUdsPlatformConnectionArguments(scanCommand, options);
            options.scanOutputPath = scanCommand.get<std::string>("-o");
            return true;
        }
        if (program.is_subcommand_used(sessionCommand)) {
            options.command = Command::Session;
            readUdsConnectionArguments(sessionCommand, options);
            options.sessionType = parseSessionType(sessionCommand.get<std::string>("--type"));
            options.sessionSuppress = sessionCommand.get<bool>("--suppress");
            options.timeoutMs = sessionCommand.get<size_t>("--timeout-ms");
            return true;
        }
        if (program.is_subcommand_used(testerPresentCommand)) {
            options.command = Command::TesterPresent;
            readUdsConnectionArguments(testerPresentCommand, options);
            options.testerPresentSuppress = testerPresentCommand.get<bool>("--suppress");
            options.testerPresentIntervalMs = testerPresentCommand.get<size_t>("--interval-ms");
            options.testerPresentCount = testerPresentCommand.get<size_t>("--count");
            options.timeoutMs = testerPresentCommand.get<size_t>("--timeout-ms");
            if (options.testerPresentIntervalMs == 0) {
                throw std::runtime_error("--interval-ms must be greater than zero");
            }
            return true;
        }
        if (program.is_subcommand_used(resetCommand)) {
            options.command = Command::Reset;
            readUdsConnectionArguments(resetCommand, options);
            options.resetType = parseResetType(resetCommand.get<std::string>("--type"));
            options.resetFunctional = resetCommand.get<bool>("--functional");
            options.resetSuppress = resetCommand.get<bool>("--suppress");
            readBusSelectionArgument(resetCommand, options);
            if (!options.busName.empty() && !options.resetFunctional) {
                throw std::runtime_error("--bus only applies to --functional reset; "
                                         "an addressed reset uses the selected ECU's bus");
            }
            options.confirmDestructive = resetCommand.get<bool>("--yes");
            return true;
        }
        if (program.is_subcommand_used(routineCommand)) {
            options.command = Command::Routine;
            if (routineCommand.is_subcommand_used(routineStartCommand)) {
                readRoutineCommandArguments(routineStartCommand, options, 0x01, true);
                return true;
            }
            if (routineCommand.is_subcommand_used(routineStopCommand)) {
                readRoutineCommandArguments(routineStopCommand, options, 0x02, true);
                return true;
            }
            if (routineCommand.is_subcommand_used(routineResultsCommand)) {
                readRoutineCommandArguments(routineResultsCommand, options, 0x03, false);
                return true;
            }
            std::cerr << routineCommand;
            return false;
        }
        if (program.is_subcommand_used(routineScanCommand)) {
            options.command = Command::RoutineScan;
            readUdsConnectionArguments(routineScanCommand, options);
            const auto from = routineScanCommand.get<unsigned>("--from");
            const auto to = routineScanCommand.get<unsigned>("--to");
            if (from > 0xFFFF || to > 0xFFFF) {
                throw std::runtime_error("routine-scan bounds must fit in two bytes");
            }
            if (from > to) {
                throw std::runtime_error("--from must be less than or equal to --to");
            }
            options.routineSubFunction = parseRoutineSubFunction(routineScanCommand.get<std::string>("--sub"));
            options.routineScanFrom = static_cast<uint16_t>(from);
            options.routineScanTo = static_cast<uint16_t>(to);
            options.routineExtendedSession = parseSessionFlag(routineScanCommand.get<std::string>("--session"));
            options.keepalive = routineScanCommand.get<bool>("--keepalive");
            options.routineScanGapMs = routineScanCommand.get<size_t>("--gap-ms");
            options.timeoutMs = routineScanCommand.get<size_t>("--timeout-ms");
            options.routineScanOutputPath = routineScanCommand.get<std::string>("-o");
            options.confirmDestructive = routineScanCommand.get<bool>("--yes");
            return true;
        }
        if (program.is_subcommand_used(monitorCommand)) {
            options.command = Command::Monitor;
            readCommonConnectionArguments(monitorCommand, options);
            readBusSelectionArgument(monitorCommand, options);
            readRawFilterArguments(monitorCommand, options);
            readTxEchoFilterArguments(monitorCommand, options, true);
            if (options.txEchoFilter.enabled && options.txEchoFilter.ids.empty()) {
                throw std::runtime_error("monitor --drop-tx-echo requires at least one --tx-id");
            }
            options.monitorCount = monitorCommand.get<size_t>("--count");
            options.monitorDurationMs = monitorCommand.is_used("--duration-ms")
                ? monitorCommand.get<size_t>("--duration-ms")
                : monitorCommand.get<size_t>("--timeout-ms");
            if (options.monitorDurationMs == 0 && options.monitorCount > 0) {
                options.monitorDurationMs = 1000;
            }
            options.monitorOutputPath = monitorCommand.get<std::string>("-o");
            options.baselineOutPath = monitorCommand.get<std::string>("--baseline-record");
            options.baselinePath = monitorCommand.get<std::string>("--baseline-compare");
            if (!options.baselineOutPath.empty() && !options.baselinePath.empty()) {
                throw std::runtime_error("monitor: use --baseline-record or --baseline-compare, not both");
            }
            const auto baselineKey = monitorCommand.get<std::string>("--baseline-key");
            if (baselineKey == "id") {
                options.baselineKey = BaselineKey::Id;
            } else if (baselineKey == "full") {
                options.baselineKey = BaselineKey::Full;
            } else {
                throw std::runtime_error("monitor --baseline-key must be \"id\" or \"full\"");
            }
            return true;
        }
        if (program.is_subcommand_used(reconCaptureCommand)) {
            options.command = Command::ReconCapture;
            readCommonConnectionArguments(reconCaptureCommand, options);
            options.busName = reconCaptureCommand.get<std::string>("--bus");
            options.reconRawProtocol = common::toLower(reconCaptureCommand.get<std::string>("--raw-protocol"));
            if (options.reconRawProtocol != "can" && options.reconRawProtocol != "can-ps"
                && options.reconRawProtocol != "auto")
                throw std::runtime_error("--raw-protocol must be can, can-ps, or auto");
            options.reconRequestCanId = parseHexId(reconCaptureCommand.get<std::string>("--req-id"));
            options.reconResponseCanId = parseHexId(reconCaptureCommand.get<std::string>("--rsp-id"));
            const auto padding = parseHexId(reconCaptureCommand.get<std::string>("--isotp-padding"));
            if (padding > 0xFF) throw std::runtime_error("--isotp-padding must fit in one byte");
            options.reconIsoTpPadding = static_cast<uint8_t>(padding);
            options.reconIsoTpPadToEight = reconCaptureCommand.get<bool>("--isotp-pad-to-8");
            const auto bs = reconCaptureCommand.get<unsigned>("--isotp-bs");
            const auto stmin = reconCaptureCommand.get<unsigned>("--isotp-stmin");
            if (bs > 0xFF || stmin > 0xFF) throw std::runtime_error("ISO-TP BS/STmin must fit in one byte");
            options.reconIsoTpBlockSize = static_cast<uint8_t>(bs);
            options.reconIsoTpStmin = static_cast<uint8_t>(stmin);
            options.reconCanId = parseHexId(reconCaptureCommand.get<std::string>("--capture-can-id"));
            options.reconMarker = common::parseHexBytes(reconCaptureCommand.get<std::string>("--capture-marker"));
            const auto routine = reconCaptureCommand.get<std::string>("--trigger-routine");
            const auto separator = routine.find(':');
            if (separator == std::string::npos) throw std::runtime_error("--trigger-routine must be RID:DATA");
            const auto rid = parseHexId(routine.substr(0, separator));
            if (rid > 0xFFFF) throw std::runtime_error("routine RID must fit in two bytes");
            options.reconRoutineId = static_cast<uint16_t>(rid);
            options.reconRoutineData = common::parseHexBytes(routine.substr(separator + 1));
            options.reconStubPath = reconCaptureCommand.get<std::string>("--load-stub-bin");
            options.reconStubSha256 = common::toLower(reconCaptureCommand.get<std::string>("--stub-sha256"));
            options.reconLoadAddress = reconCaptureCommand.get<unsigned>("--load-address");
            options.reconLoadLength = options.reconStubPath.empty() ? 0 : static_cast<uint32_t>(std::filesystem::file_size(options.reconStubPath));
            options.reconCaptureCount = reconCaptureCommand.get<size_t>("--capture-count");
            options.reconCaptureTimeoutMs = reconCaptureCommand.get<size_t>("--capture-timeout-ms");
            const auto referenceBytes = reconCaptureCommand.get<std::string>("--reference-bytes");
            if (!referenceBytes.empty()) {
                options.reconReferenceBytes = common::parseHexBytes(referenceBytes);
            }
            options.reconReferenceAddress = reconCaptureCommand.get<unsigned>("--reference-address");
            options.reconOutputPath = reconCaptureCommand.get<std::string>("--out");
            options.reconManifestPath = reconCaptureCommand.get<std::string>("--manifest");
            options.reconHealthSessionRequest = common::parseHexBytes(reconCaptureCommand.get<std::string>("--health-session"));
            options.reconHealthSessionExpect = common::parseHexBytes(reconCaptureCommand.get<std::string>("--health-session-expect"));
            options.reconHealthDidRequest = common::parseHexBytes(reconCaptureCommand.get<std::string>("--health-did"));
            options.reconHealthDidExpect = common::parseHexBytes(reconCaptureCommand.get<std::string>("--health-did-expect"));
            options.timeoutMs = reconCaptureCommand.get<size_t>("--timeout-ms");
            options.reconSmokeOnly = reconCaptureCommand.get<bool>("--smoke-only");
            if (options.reconCaptureCount == 0 || options.reconCaptureTimeoutMs == 0)
                throw std::runtime_error("recon capture count and timeout must be greater than zero");
            return true;
        }
        if (program.is_subcommand_used(dtcCommand)) {
            if (dtcCommand.is_subcommand_used(dtcReadCommand)) {
                options.command = Command::DtcRead;
                readDtcReadOnlyArguments(dtcReadCommand, options);
                options.keepalive = dtcReadCommand.get<bool>("--keepalive");
                options.dtcOutputPath = dtcReadCommand.get<std::string>("-o");
                const auto statusMask = dtcReadCommand.get<unsigned>("--status-mask");
                if (statusMask > 0xFF) {
                    throw std::runtime_error("--status-mask must fit in one byte");
                }
                options.dtcStatusMask = static_cast<uint8_t>(statusMask);
                options.dtcConfirmedOnly = dtcReadCommand.get<bool>("--confirmed");
                return true;
            }
            if (dtcCommand.is_subcommand_used(dtcClearCommand)) {
                options.command = Command::DtcClear;
                readDtcArguments(dtcClearCommand, options);
                options.dtcClearOutputPath = dtcClearCommand.get<std::string>("-o");
                options.confirmDestructive = dtcClearCommand.get<bool>("--yes");
                return true;
            }
            if (dtcCommand.is_subcommand_used(dtcSnapshotCommand)) {
                options.command = Command::DtcSnapshot;
                readDtcRecordArguments(dtcSnapshotCommand, options);
                options.keepalive = dtcSnapshotCommand.get<bool>("--keepalive");
                return true;
            }
            if (dtcCommand.is_subcommand_used(dtcExtendedCommand)) {
                options.command = Command::DtcExtended;
                readDtcRecordArguments(dtcExtendedCommand, options);
                options.keepalive = dtcExtendedCommand.get<bool>("--keepalive");
                return true;
            }
            std::cerr << dtcCommand;
            return false;
        }
        if (program.is_subcommand_used(obdCommand)) {
            if (obdCommand.is_subcommand_used(obdVinCommand)) {
                options.command = Command::ObdVin;
                readUdsPlatformConnectionArguments(obdVinCommand, options);
                options.obdOutputPath = obdVinCommand.get<std::string>("-o");
                return true;
            }
            if (obdCommand.is_subcommand_used(obdPidCommand)) {
                options.command = Command::ObdPid;
                readUdsPlatformConnectionArguments(obdPidCommand, options);
                const auto pid = parseHexId(obdPidCommand.get<std::string>("--id"));
                if (pid > 0xFF) {
                    throw std::runtime_error("OBD PID must fit in one byte");
                }
                options.obdPid = static_cast<uint8_t>(pid);
                options.obdOutputPath = obdPidCommand.get<std::string>("-o");
                return true;
            }
            if (obdCommand.is_subcommand_used(obdDtcReadCommand)) {
                options.command = Command::ObdDtcRead;
                readUdsPlatformConnectionArguments(obdDtcReadCommand, options);
                options.obdOutputPath = obdDtcReadCommand.get<std::string>("-o");
                return true;
            }
            if (obdCommand.is_subcommand_used(obdDtcClearCommand)) {
                options.command = Command::ObdDtcClear;
                readUdsPlatformConnectionArguments(obdDtcClearCommand, options);
                options.confirmDestructive = obdDtcClearCommand.get<bool>("--yes");
                options.obdOutputPath = obdDtcClearCommand.get<std::string>("-o");
                return true;
            }
            std::cerr << obdCommand;
            return false;
        }
        if (program.is_subcommand_used(scriptCommand)) {
            options.command = Command::Script;
            options.scriptPath = scriptCommand.get<std::string>("--file");
            options.scriptOutputDir = scriptCommand.get<std::string>("--output-dir");
            options.tracePath = scriptCommand.get<std::string>("--trace");
            options.deviceName = scriptCommand.get<std::string>("-d");
            options.scriptDryRun = scriptCommand.get<bool>("--dry-run");
            options.confirmDestructive = scriptCommand.get<bool>("--yes");
            if (scriptCommand.is_used("-f")) {
                options.platformName = scriptCommand.get<std::string>("-f");
                options.scriptPlatformOverride = common::parseCarPlatform(options.platformName);
            }
            if (scriptCommand.is_used("-e")) {
                const auto ecuId = scriptCommand.get<unsigned>("-e");
                if (ecuId > 0xFF) throw std::runtime_error("--ecu must fit in one byte");
                options.scriptEcuOverride = static_cast<uint8_t>(ecuId);
            }
            options.baudrateOverride = scriptCommand.is_used("-b")
                ? std::optional<uint32_t>{scriptCommand.get<unsigned>("-b")}
                : std::nullopt;
            if (scriptCommand.is_used("--var")) {
                for (const auto& assignment : scriptCommand.get<std::vector<std::string>>("--var")) {
                    const auto separator = assignment.find('=');
                    if (separator == std::string::npos || separator == 0) {
                        throw std::runtime_error("--var must be NAME=VALUE");
                    }
                    options.scriptVariables.emplace_back(
                        assignment.substr(0, separator), assignment.substr(separator + 1));
                }
            }
            return true;
        }
        std::cerr << program;
        return false;
    }
    catch (const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        // Print usage for the selected subcommand.
        const argparse::ArgumentParser* target = &program;
        for (auto* command : {&identCommand, &sendCommand, &canCommand, &probeCommand, &wakeCommand,
                              &didCommand, &scanCommand, &sessionCommand, &testerPresentCommand,
                              &resetCommand, &routineCommand, &routineScanCommand,
                              &monitorCommand, &dtcCommand, &obdCommand, &scriptCommand,
                              &busesCommand}) {
            try {
                if (program.is_subcommand_used(*command)) {
                    target = command;
                    break;
                }
            }
            catch (...) {
            }
        }
        std::cerr << *target;
        return false;
    }
}

} // namespace volvodiag
