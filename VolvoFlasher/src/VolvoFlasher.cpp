#include <common/protocols/UDSError.hpp>
#include <common/protocols/UDSMessage.hpp>
#include <common/protocols/UDSProtocolCommonSteps.hpp>
#include <common/protocols/UDSPinFinder.hpp>
#include <common/protocols/UDSRequest.hpp>
#include <common/CommonData.hpp>
#include <common/CliSupport.hpp>
#include <common/J2534ChannelProvider.hpp>
#include <common/RuntimeDiagnostics.hpp>
#include <common/VBFParser.hpp>
#include <common/Util.hpp>

#include <j2534/J2534.hpp>
#include <j2534/J2534Channel.hpp>

#include <flasher/SBLProviderVBF.hpp>
#include <flasher/UDSFlasher.hpp>
#include <flasher/UDSReader.hpp>

#include <argparse/argparse.hpp>

#include <easylogging++.h>

#include <cctype>
#include <charconv>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <stdexcept>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

INITIALIZE_EASYLOGGINGPP

enum class RunMode
{
	None,
	Flash,
	Read,
	Wakeup,
	Pin,
	Diag,
	Reset,
	Program,
	UdsRaw
};

enum class ProgramMode
{
	Vehicle,
	Bench
};

enum class ReadFormat
{
	Hex,
	Bin
};

ProgramMode parseProgramMode(const std::string& input)
{
	const auto normalized = common::toLower(input);
	if (normalized == "vehicle") {
		return ProgramMode::Vehicle;
	}
	if (normalized == "bench") {
		return ProgramMode::Bench;
	}
	throw std::runtime_error("Invalid --program-mode, required values: vehicle or bench");
}

const char* programModeToString(ProgramMode mode)
{
	switch (mode) {
	case ProgramMode::Vehicle:
		return "vehicle";
	case ProgramMode::Bench:
		return "bench";
	}
	return "unknown";
}

ReadFormat parseReadFormat(const std::string& input)
{
	const auto normalized = common::toLower(input);
	if (normalized == "hex") {
		return ReadFormat::Hex;
	}
	if (normalized == "bin") {
		return ReadFormat::Bin;
	}
	throw std::runtime_error("Invalid --format, required values: hex or bin");
}

const char* readFormatToString(ReadFormat format)
{
	switch (format) {
	case ReadFormat::Hex:
		return "hex";
	case ReadFormat::Bin:
		return "bin";
	}
	return "unknown";
}

void ensureOutputDirectoryExists(const std::string& path)
{
	const std::filesystem::path outputPath{ path };
	if (const auto parent = outputPath.parent_path(); !parent.empty()) {
		std::filesystem::create_directories(parent);
	}
}

void UDSProgramMode(common::CarPlatform carPlatform, uint8_t ecuId, j2534::J2534& j2534,
	unsigned long holdSeconds, std::optional<uint32_t> baudrateOverride = std::nullopt);

// Parses a UDS diagnostic session spec into its subfunction byte; "none" -> 0.
uint8_t parseUdsSessionByte(const std::string& input) {
	const auto value = common::toLower(input);
	if (value == "none") {
		return 0x00;
	}
	if (value == "default") {
		return 0x01;
	}
	if (value == "programming") {
		return 0x02;
	}
	if (value == "ext" || value == "extended") {
		return 0x03;
	}
	const auto numeric = common::parseHexU32(value);
	if (numeric == 0 || numeric > 0x7F) {
		throw std::runtime_error("--session must be none|default|programming|ext|extended or a hex byte 01-7F");
	}
	return static_cast<uint8_t>(numeric);
}

bool getRunOptions(int argc, const char* argv[], std::string& deviceName,
	std::optional<unsigned long>& baudrateOverride, std::string& flashPath, uint64_t& pin, bool& pinSpecified,
	uint8_t& ecuId, unsigned long& start, unsigned long& datasize,
	RunMode& runMode, std::string& sblPath, common::CarPlatform& carPlatform,
	bool& pinUpward, bool& resetFunctional, unsigned long& programHoldSeconds,
	ProgramMode& flashProgramMode, ReadFormat& readFormat,
	std::vector<std::string>& rawData, bool& noWakeup, bool& attachRunningSbl,
	bool& udsRawWake, uint8_t& udsRawSession, bool& noSblAuth, bool& skipFallAsleep,
	common::PinSearchWindow& pinWindow, bool& pinWindowSet, bool& cliArgsError) {
	argparse::ArgumentParser program("VolvoFlasher", "1.0", argparse::default_arguments::help);
    const auto addDebugArgument = [](argparse::ArgumentParser& parser) {
        parser.add_argument("--debug").default_value(false).implicit_value(true).nargs(0)
            .help("Enable verbose debug logging");
    };
    addDebugArgument(program);
	program.add_argument("-d", "--device").default_value(std::string{}).help("Device name");
	program.add_argument("-b", "--baudrate").scan<'u', unsigned long>()
		.help("CAN bus speed override; by default every bus opens at its configured speed");
	program.add_argument("-f", "--platform").default_value(std::string{ "p3" }).help("Car's platform, supported values: p3, p3_y413, p3_y283_iam, p3_y283_icm, p3_p313_icm, p3_p313_iam, p3_y555_iam, p3_y555_icm, p3_y312h_iam, p3_y312h_icm");
	program.add_argument("-e", "--ecu").scan<'x', unsigned int>().default_value(0x10u).help("ECU id, hexadecimal byte (P3 default: ECM 0x10)");
	program.add_argument("-p", "--pin").help("PIN to unlock ECU, exactly 5 hex bytes, e.g. AABBCCDDEE (or \"AA BB CC DD EE\")");

	argparse::ArgumentParser flash_command("flash", "1.0", argparse::default_arguments::help);
    addDebugArgument(flash_command);
	flash_command.add_description(
		"Flash BIN to ECU.\n"
		"Interrupt policy: Ctrl+C aborts cleanly until erasing starts; once erase has begun\n"
		"the operation ignores interruptions and runs to completion - stopping mid-write\n"
		"would leave the ECU without a valid application.");
	flash_command.add_argument("-i", "--input").help("File to flash");
	flash_command.add_argument("-s", "--sbl").default_value(std::string()).help("File with SBL, required for UDS flashing");
	flash_command.add_argument("--program-mode").required()
		.help("Programming mode handling, required: vehicle or bench");
	flash_command.add_argument("--attach-running-sbl").default_value(false).implicit_value(true).nargs(0)
		.help("Flash through an already running RAM SBL; skips the programming-session broadcast and SBL load/start");
	flash_command.add_argument("--skip-fall-asleep").default_value(false).implicit_value(true).nargs(0)
		.help("Skip the programming-session broadcast prelude; use when the module is already up (bench, prepared by CEM, or attached)");

	argparse::ArgumentParser read_command("read", "1.0", argparse::default_arguments::help);
    addDebugArgument(read_command);
	read_command.add_description("Read ECU memory range");
	read_command.add_argument("-o", "--output").help("File to write");
	read_command.add_argument("-s", "--start").scan<'x', unsigned long>().help("Begin address to read");
	read_command.add_argument("-sz", "--size").scan<'x', unsigned long>().help("Datasize to read");
	read_command.add_argument("--sbl").default_value(std::string()).help("File with SBL, required for UDS reading");
	read_command.add_argument("--format").default_value(std::string{ "hex" }).help("Output format: hex or bin");
	read_command.add_argument("--program-mode").default_value(std::string{ "bench" })
		.help("Programming mode handling for UDS reading: vehicle or bench");
	read_command.add_argument("--attach-running-sbl").default_value(false).implicit_value(true).nargs(0)
		.help("Read through an already running RAM SBL; skips the programming-session broadcast and SBL load/start");
	read_command.add_argument("--skip-fall-asleep").default_value(false).implicit_value(true).nargs(0)
		.help("Skip the programming-session broadcast prelude; use when the module is already up (bench, prepared by CEM, or attached)");
	read_command.add_argument("--no-sbl-auth").default_value(false).implicit_value(true).nargs(0)
		.help("Skip SecurityAccess (27 01) against the SBL; use with a resident/read SBL that does not implement it");

	argparse::ArgumentParser pin_command("pin", "1.0", argparse::default_arguments::help);
    addDebugArgument(pin_command);
	pin_command.add_description(
		"Bruteforce ECM/CEM SecurityAccess PIN.\n"
		"By default the CEM online search is run: it scans the window 0xFFFF000000..0xFFFFFFFFFF\n"
		"downward, looking for a 3-byte hex key collision (2^24 candidates, typically hours).\n"
		"The found value has the form FFFFxxxxxx - it is NOT the factory PIN from a CEM dump;\n"
		"it only unlocks this ECU over SecurityAccess. -p/-d/--floor/--ceil switch to an expert\n"
		"manual scan from an explicit start value and/or explicit window.");
	pin_command.add_argument("-d", "--down").default_value(false).implicit_value(true).nargs(0).help("Scan pins downward");
	pin_command.add_argument("--floor").scan<'x', unsigned long long>()
		.help("Explicit scan window lower bound, hex (e.g. 0xffff000000)");
	pin_command.add_argument("--ceil").scan<'x', unsigned long long>()
		.help("Explicit scan window upper bound, hex (e.g. 0xffffffffff)");

	argparse::ArgumentParser wakeup_command("wakeup", "1.0", argparse::default_arguments::help);
    addDebugArgument(wakeup_command);
	wakeup_command.add_description(
		"Send a functional ECUReset broadcast (11 11, then 11 81 on 0x7DF): this resets\n"
		"every module that hears it - it does not wake anything up.");

	argparse::ArgumentParser diag_command("diag", "1.0", argparse::default_arguments::help);
    addDebugArgument(diag_command);
	diag_command.add_description("Probe UDS ECU connectivity without writing anything");

	argparse::ArgumentParser reset_command("reset", "1.0", argparse::default_arguments::help);
    addDebugArgument(reset_command);
	reset_command.add_description("Send UDS ECU reset");
	reset_command.add_argument("--functional").default_value(false).implicit_value(true).nargs(0)
		.help("Broadcast emergency reset 0x7DF:11 81");

	argparse::ArgumentParser program_command("program", "1.0", argparse::default_arguments::help);
    addDebugArgument(program_command);
	program_command.add_description("Enter P3 functional programming mode");
	program_command.add_argument("--hold").scan<'u', unsigned long>().default_value(0u)
		.help("Keep TesterPresent running for N seconds after entering programming mode");

	argparse::ArgumentParser uds_raw_command("uds-raw", "1.0", argparse::default_arguments::help);
    addDebugArgument(uds_raw_command);
	uds_raw_command.add_description("Send raw UDS request(s) to ECU and print response (spelled uds-raw, with a hyphen). Optional --sbl keeps the same RAM SBL session for probes.");
	uds_raw_command.add_argument("--data").required().append().help("Hex bytes, repeatable, e.g. \"10 03\" or \"31 01 03 01 00 7F 80 80\"");
	uds_raw_command.add_argument("--sbl").default_value(std::string()).help("Optional SBL VBF to load/start before raw request(s)");
	uds_raw_command.add_argument("--program-mode").default_value(std::string{ "bench" })
		.help("Programming mode handling when --sbl is used: vehicle or bench");
	uds_raw_command.add_argument("--no-wakeup").default_value(false).implicit_value(true).nargs(0)
		.help("Do not send the functional ECUReset cleanup after --sbl raw session");
	uds_raw_command.add_argument("--wake").default_value(false).implicit_value(true).nargs(0)
		.help("Send a functional wake burst (7DF 10 82) on the ECU bus before the raw request(s)");
	uds_raw_command.add_argument("--session").default_value(std::string{ "none" })
		.help("Enter a diagnostic session first: none|default|programming|ext|extended (or hex 01-7F)");

	program.add_subparser(flash_command);
	program.add_subparser(read_command);
	program.add_subparser(pin_command);
	program.add_subparser(wakeup_command);
	program.add_subparser(diag_command);
	program.add_subparser(reset_command);
	program.add_subparser(program_command);
	program.add_subparser(uds_raw_command);
	try {
		program.parse_args(argc, argv);
		if (program.is_subcommand_used(flash_command)) {
			flashPath = flash_command.get("-i");
			sblPath = flash_command.get("-s");
			flashProgramMode = parseProgramMode(flash_command.get<std::string>("--program-mode"));
			attachRunningSbl = flash_command.get<bool>("--attach-running-sbl");
			skipFallAsleep = flash_command.get<bool>("--skip-fall-asleep");
			runMode = RunMode::Flash;
		}
		else if (program.is_subcommand_used(read_command)) {
			flashPath = read_command.get("-o");
		start = read_command.get<unsigned long>("-s");
		datasize = read_command.get<unsigned long>("-sz");
		if (datasize == 0) {
			throw std::runtime_error("--size must be greater than zero");
		}
		if (start > std::numeric_limits<unsigned long>::max() - datasize) {
			throw std::runtime_error("--start + --size exceeds the 32-bit address space");
		}
			sblPath = read_command.get<std::string>("--sbl");
			readFormat = parseReadFormat(read_command.get<std::string>("--format"));
			flashProgramMode = parseProgramMode(read_command.get<std::string>("--program-mode"));
			attachRunningSbl = read_command.get<bool>("--attach-running-sbl");
			noSblAuth = read_command.get<bool>("--no-sbl-auth");
			skipFallAsleep = read_command.get<bool>("--skip-fall-asleep");
			runMode = RunMode::Read;
		}
		else if (program.is_subcommand_used(pin_command)) {
			pinUpward = !pin_command.get<bool>("-d");
			const auto floorArg = pin_command.present<unsigned long long>("--floor");
			const auto ceilArg = pin_command.present<unsigned long long>("--ceil");
			if (floorArg) {
				pinWindow.floorPin = *floorArg;
			}
			if (ceilArg) {
				pinWindow.ceilPin = *ceilArg;
			}
			pinWindowSet = floorArg.has_value() || ceilArg.has_value();
			runMode = RunMode::Pin;
		}
		else if (program.is_subcommand_used(wakeup_command)) {
			runMode = RunMode::Wakeup;
		}
		else if (program.is_subcommand_used(diag_command)) {
			runMode = RunMode::Diag;
		}
		else if (program.is_subcommand_used(reset_command)) {
			resetFunctional = reset_command.get<bool>("--functional");
			runMode = RunMode::Reset;
		}
		else if (program.is_subcommand_used(program_command)) {
			programHoldSeconds = program_command.get<unsigned long>("--hold");
			runMode = RunMode::Program;
		}
		else if (program.is_subcommand_used(uds_raw_command)) {
			rawData = uds_raw_command.get<std::vector<std::string>>("--data");
			sblPath = uds_raw_command.get<std::string>("--sbl");
			flashProgramMode = parseProgramMode(uds_raw_command.get<std::string>("--program-mode"));
			noWakeup = uds_raw_command.get<bool>("--no-wakeup");
			udsRawWake = uds_raw_command.get<bool>("--wake");
			udsRawSession = parseUdsSessionByte(uds_raw_command.get<std::string>("--session"));
			runMode = RunMode::UdsRaw;
		}
		else {
			std::cout << program;
			return false;
		}
		deviceName = program.get("-d");
		// Only an explicitly given -b overrides the platform configuration; the default
		// value stays a display default, not a silent forced speed.
		baudrateOverride = program.present<unsigned long>("-b");
		const auto parsedEcuId = program.get<unsigned int>("-e");
		if (parsedEcuId > 0xFF) {
			throw std::runtime_error("ECU id is out of range: " + std::to_string(parsedEcuId));
		}
		ecuId = static_cast<uint8_t>(parsedEcuId);
		carPlatform = common::parseCarPlatform(program.get<std::string>("-f"));
		pinSpecified = program.present("-p").has_value();
		if (pinSpecified) {
			pin = common::securityPinToUint64(common::parseSecurityPin(program.get<std::string>("-p")));
		}
		return true;
	}
	catch (const std::exception& err) {
		std::cerr << err.what() << std::endl;
		std::cerr << program;
		// Distinguish a parse failure from "no subcommand given": scripts must see a
		// nonzero exit code instead of a successful-looking device listing.
		cliArgsError = true;
	}
	return false;
}

class FlasherCallback final : public flasher::FlasherCallback {
public:
    FlasherCallback() = default;

	void OnProgress(std::chrono::milliseconds /*timePoint*/, size_t currentValue,
		size_t maxValue) override {
        if (maxValue == 0) {
            return;
        }

        const size_t percent = std::min<size_t>(100, currentValue * 100 / maxValue);
        if (percent == _lastPercent) {
            return;
        }

        if (percent == 100 || percent / 5 != _lastPercent / 5) {
            std::cout << " " << percent << "%" << std::flush;
        }
        _lastPercent = percent;
	}

    void OnState(flasher::FlasherState state) override {
        _lastPercent = std::numeric_limits<size_t>::max();
        std::cout << std::endl;
        using flasher::FlasherState;
        switch(state) {
        case FlasherState::Initial:
            std::cout << "Starting";
            break;
        case FlasherState::FallAsleep:
            std::cout << "Go to sleep";
            break;
        case FlasherState::Authorize:
            std::cout << "Authorizing";
            break;
		case FlasherState::LoadBootloader:
            std::cout << "Bootloader loading";
            break;
        case FlasherState::StartBootloader:
            std::cout << "Bootloader starting";
            break;
		case FlasherState::EraseFlash:
            std::cout << "Flash erasing";
            break;
        case FlasherState::WriteFlash:
            std::cout << "Flash writing";
            break;
        case FlasherState::ReadFlash:
            std::cout << "Flash reading";
            break;
        case FlasherState::WakeUp:
            std::cout << "Waking up";
            break;
        case FlasherState::Done:
            std::cout << "Done";
            break;
        case FlasherState::Error:
            std::cout << "Error";
            break;
        }
    }

private:
    size_t _lastPercent{std::numeric_limits<size_t>::max()};
};

void findPin2(j2534::J2534& j2534, common::CarPlatform carPlatform, uint8_t ecuId,
	uint64_t startPin = 0, bool upward = true,
	std::optional<common::PinSearchWindow> window = {})
{
	std::chrono::time_point savedTime = std::chrono::steady_clock::now();
	uint64_t savedPin = startPin;
	common::UDSPinFinder pinFinder(j2534, carPlatform, ecuId, [&savedTime, &savedPin](common::UDSPinFinder::State state, uint64_t currentPin) {
		switch (state) {
		case common::UDSPinFinder::State::FallAsleep:
			std::cout << "Programming session broadcast" << std::endl;
			break;
		case common::UDSPinFinder::State::KeepAlive:
			std::cout << "Start keep alive" << std::endl;
			break;
		case common::UDSPinFinder::State::Work: {
			// Progress is throttled by wall time: the callback fires on every candidate,
			// and printing per candidate would dominate the scan time.
			const auto now = std::chrono::steady_clock::now();
			if (now - savedTime < std::chrono::seconds(1)) {
				break;
			}
			uint64_t pinDiff = currentPin > savedPin ? currentPin - savedPin : savedPin - currentPin;
			const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - savedTime).count();
			const auto pinPerSec = elapsedMs > 0 ? pinDiff * 1000 / static_cast<uint64_t>(elapsedMs) : 0;
			savedPin = currentPin;
			savedTime = now;
			std::cout << "Trying PIN " << std::hex << currentPin << ", " << std::dec << pinPerSec << " pins/sec" << std::endl;
			break;
		}
		}
		}, upward ? common::UDSPinFinder::Direction::Up : common::UDSPinFinder::Direction::Down, startPin,
		window.value_or(common::PinSearchWindow{}));

	if (!pinFinder.start()) {
		std::cout << "Failed to start PIN finder" << std::endl;
	}
	else {
		while (pinFinder.getCurrentState() != common::UDSPinFinder::State::Done && pinFinder.getCurrentState() != common::UDSPinFinder::State::Error) {
			if (common::stopRequested.load()) {
				pinFinder.stop();
				common::resetStopRequested();
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		if (const auto foundPin{ pinFinder.getFoundPin() }) {
			std::cout << "Found PIN code "
				<< std::hex << std::setfill('0') << *foundPin << std::endl;
		}
		else {
			std::cout << "Last checked PIN code "
				<< std::hex << std::setfill('0') << savedPin << std::endl;
		}
	}
}

void UDSFlash(common::CarPlatform carPlatform, uint8_t ecuId,
	std::unique_ptr<j2534::J2534> j2534, std::optional<uint32_t> baudrateOverride, uint64_t pin,
	const std::string& flashPath,
	const std::string& sblPath, ProgramMode programMode, bool attachRunningSbl, bool skipFallAsleepCli)
{
	LOG(INFO) << "UDS flash start platform=" << static_cast<int>(carPlatform)
		<< " ecu=0x" << std::hex << static_cast<int>(ecuId)
		<< " baudrate=" << std::dec << (baudrateOverride ? std::to_string(*baudrateOverride) : std::string("config"))
		<< " flashPath=" << flashPath
		<< " sblPath=" << (sblPath.empty() ? "<missing>" : sblPath)
		<< " programMode=" << programModeToString(programMode)
		<< " attachRunningSbl=" << attachRunningSbl;
	common::VBFParser vbfParser;
	std::ifstream flashVbf(flashPath, std::ios_base::binary);
	if (!flashVbf) {
		throw std::runtime_error("Failed to open flash VBF: " + flashPath);
	}
	const common::VBF flash{ vbfParser.parse(flashVbf) };
	const auto ecuInfo{ common::getEcuInfoByEcuId(carPlatform, ecuId) };
	LOG(INFO) << "UDS flash target can=0x" << std::hex << std::get<1>(ecuInfo).canId
		<< " chunks=" << std::dec << flash.chunks.size()
		<< " eraseBlocks=" << flash.header.eraseBlocks.size();
	std::string additionalData;
	std::unique_ptr<flasher::SBLProviderBase> sblProvider;
	if (sblPath.empty() && !attachRunningSbl) {
		throw std::runtime_error("SBL VBF is required for UDS flashing; pass -s/--sbl");
	}
	else if (!attachRunningSbl) {
		std::ifstream sblVbf(sblPath, std::ios_base::binary);
		if (!sblVbf) {
			throw std::runtime_error("Failed to open SBL VBF: " + sblPath);
		}
		const common::VBF bootloader{ vbfParser.parse(sblVbf) };
		sblProvider = std::make_unique<flasher::SBLProviderVBF>(bootloader);
	}

	bool skipFallAsleep = skipFallAsleepCli;
	if (programMode == ProgramMode::Vehicle) {
		UDSProgramMode(carPlatform, ecuId, *j2534, 0, baudrateOverride);
		skipFallAsleep = true;
	}
	else {
		LOG(INFO) << "Bench program mode selected, skipping CEM programming mode"
			<< (skipFallAsleepCli ? "; --skip-fall-asleep set, skipping broadcast prelude" : "");
	}

	flasher::FlasherParameters flasherParameters{
		carPlatform,
		ecuId,
		additionalData,
		std::move(sblProvider),
		flash,
		baudrateOverride
	};
	flasher::UDSFlasherParameters udsFlasherParameters{
		{ (pin >> 32) & 0xFF, (pin >> 24) & 0xFF, (pin >> 16) & 0xFF, (pin >> 8) & 0xFF, pin & 0xFF },
		skipFallAsleep,
		attachRunningSbl };
	flasher::UDSFlasher flasher{ *j2534, std::move(flasherParameters), std::move(udsFlasherParameters) };
	FlasherCallback callback;
	flasher.registerCallback(callback);
	flasher.run();
	const bool success = flasher.getCurrentState() ==
		flasher::FlasherState::Done;
	std::cout << std::endl
		<< ((success)
			? "Flashing done"
			: "Flashing error. Try again.")
		<< std::endl;
	if (!success && !flasher.getLastError().empty()) {
		std::cout << "Last error: " << flasher.getLastError() << std::endl;
	}
}

void printBytes(const std::vector<uint8_t>& bytes)
{
	if (bytes.empty()) {
		std::cout << "(empty)";
		return;
	}
	const auto oldFlags = std::cout.flags();
	const auto oldFill = std::cout.fill();
	for (const auto byte: bytes) {
		std::cout << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
			<< static_cast<int>(byte) << " ";
	}
	std::cout.flags(oldFlags);
	std::cout.fill(oldFill);
}

bool runUdsProbe(const j2534::J2534Channel& channel, uint32_t canId,
	const std::string& label, const std::vector<uint8_t>& request,
	const std::vector<uint8_t>& expectedResponseData = {}, size_t timeout = 2000)
{
	std::cout << label << " TX: ";
	printBytes(request);
	std::cout << std::endl;
	try {
		common::UDSRequest udsRequest(canId, request);
		const auto response = expectedResponseData.empty()
			? udsRequest.process(channel, timeout)
			: udsRequest.process(channel, expectedResponseData, 1, timeout);
		std::cout << label << " RX: ";
		printBytes(response);
		std::cout << std::endl;
		return true;
	}
	catch (const common::UDSError& ex) {
		std::cout << label << " NRC: " << ex.what() << std::endl;
	}
	catch (const std::exception& ex) {
		std::cout << label << " error: " << ex.what() << std::endl;
	}
	return false;
}

using common::parseHexBytes;

bool isRetryablePostStartTxFailure(const common::UDSRequestTxError& ex)
{
	return ex.status() == ERR_TIMEOUT && ex.written() == 0;
}

void reopenRawUdsChannel(common::J2534ChannelProvider& channelProvider,
	std::vector<std::unique_ptr<j2534::J2534Channel>>& channels, uint8_t ecuId)
{
	channels.front().reset();
	channels.front() = channelProvider.getChannelForEcu(ecuId);
	if (!channels.front()) {
		throw std::runtime_error("Failed to reopen J2534 channel for ECU");
	}
	channels.front()->clearRx();
	channels.front()->clearTx();
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

std::vector<uint8_t> processRawUdsRequest(j2534::J2534Channel& channel, uint32_t canId,
	const std::vector<uint8_t>& request)
{
	common::UDSRequest udsRequest(canId, request);
	return udsRequest.process(channel, 5000);
}

std::vector<uint8_t> processRawUdsRequestWithOptionalRetry(
	common::J2534ChannelProvider& channelProvider,
	std::vector<std::unique_ptr<j2534::J2534Channel>>& channels,
	uint8_t ecuId, uint32_t canId, const std::vector<uint8_t>& request, bool allowRetry)
{
	try {
		return processRawUdsRequest(*channels.front(), canId, request);
	}
	catch (const common::UDSRequestTxError& ex) {
		if (!allowRetry || !isRetryablePostStartTxFailure(ex)) {
			throw;
		}
		LOG(WARNING) << "first post-start TX failed written=0, reopening channel and retrying once";
		reopenRawUdsChannel(channelProvider, channels, ecuId);
		return processRawUdsRequest(*channels.front(), canId, request);
	}
}

std::vector<uint8_t> processPostStartWarmupRequest(
	common::J2534ChannelProvider& channelProvider,
	std::vector<std::unique_ptr<j2534::J2534Channel>>& channels,
	uint8_t ecuId, uint32_t canId, const std::vector<uint8_t>& request)
{
	try {
		return processRawUdsRequestWithOptionalRetry(channelProvider, channels, ecuId, canId, request, true);
	}
	catch (const common::UDSRequestRxTimeout&) {
		LOG(WARNING) << "post-start warmup read timeout, reopening channel and retrying once";
		reopenRawUdsChannel(channelProvider, channels, ecuId);
		return processRawUdsRequest(*channels.front(), canId, request);
	}
}

void warmupPostStartRawChannel(common::J2534ChannelProvider& channelProvider,
	std::vector<std::unique_ptr<j2534::J2534Channel>>& channels, uint8_t ecuId, uint32_t canId)
{
	LOG(INFO) << "post-start channel warmup enter";
	channels.front()->clearRx();
	channels.front()->clearTx();
	std::this_thread::sleep_for(std::chrono::milliseconds(200));

	const std::vector<uint8_t> warmupRequest{ 0x22, 0xF1, 0x8C };
	(void)processPostStartWarmupRequest(channelProvider, channels, ecuId, canId, warmupRequest);

	LOG(INFO) << "post-start warmup TX/RX ok";
	LOG(INFO) << "post-start warmup done";
}

// Functional wake burst (single-frame ISO-TP 10 82 on 0x7DF), mirroring VolvoDiag's
// prelude --wake, to bring a sleeping/uninitialised bus up before raw requests.
void sendUdsRawWakeBurst(j2534::J2534& j2534, common::CarPlatform carPlatform, uint8_t ecuId)
{
	const auto bus = std::get<0>(common::getEcuInfoByEcuId(carPlatform, ecuId));
	auto rawChannel = common::openRawCanChannel(j2534, bus);
	const std::vector<uint8_t> wakeFrame = common::makeCanFrame(0x7DF, { 0x02, 0x10, 0x82, 0, 0, 0, 0, 0 });
	constexpr size_t burstCount = 10;
	for (size_t i = 0; i < burstCount; ++i) {
		const auto status = rawChannel->writeMsg(wakeFrame, 1000);
		if (status != STATUS_NOERROR) {
			throw std::runtime_error("Failed to send wake frame: " + common::j2534StatusToString(status));
		}
		if (i + 1 < burstCount) {
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
	}
	LOG(INFO) << "uds-raw wake burst sent (7DF 10 82 x" << burstCount << ")";
}

void UDSRaw(common::CarPlatform carPlatform, uint8_t ecuId, j2534::J2534& j2534,
	uint64_t pin, const std::vector<std::string>& rawData, const std::string& sblPath,
	ProgramMode programMode, bool noWakeup, bool wake, uint8_t session,
	std::optional<uint32_t> baudrateOverride)
{
	const auto ecuInfo{ common::getEcuInfoByEcuId(carPlatform, ecuId) };
	if (std::get<0>(ecuInfo).protocolId != ISO15765) {
		throw std::runtime_error("uds-raw supports UDS/ISO15765 ECUs only");
	}
	const auto canId = std::get<1>(ecuInfo).canId;
	if (rawData.empty()) {
		throw std::runtime_error("uds-raw requires at least one --data");
	}

	if (wake) {
		sendUdsRawWakeBurst(j2534, carPlatform, ecuId);
	}

	bool skipFallAsleep = false;
	if (!sblPath.empty() && programMode == ProgramMode::Vehicle) {
		UDSProgramMode(carPlatform, ecuId, j2534, 0, baudrateOverride);
		skipFallAsleep = true;
	}
	else if (!sblPath.empty()) {
		LOG(INFO) << "Bench program mode selected, skipping CEM programming mode";
	}

	common::J2534ChannelProvider channelProvider{ j2534, carPlatform, baudrateOverride };
	std::vector<std::unique_ptr<j2534::J2534Channel>> channels;
	// getChannelForEcu throws on failure, so the channel here is always usable.
	channels.emplace_back(channelProvider.getChannelForEcu(ecuId));
	// Always fetched fresh from the vector: reopenRawUdsChannel may replace the channel
	// object mid-session (post-start TX/RX failure), so a reference bound here once would
	// dangle across the warmup path.
	const auto currentChannel = [&channels]() -> j2534::J2534Channel& { return *channels.front(); };

	if (!sblPath.empty()) {
		LOG(INFO) << "UDS raw SBL session start sblPath=" << sblPath
			<< " programMode=" << programModeToString(programMode)
			<< " noWakeup=" << noWakeup;
		if (skipFallAsleep) {
			LOG(INFO) << "Programming-session broadcast skipped, vehicle programming mode was prepared by CEM";
		}
		else if (!common::UDSProtocolCommonSteps::broadcastProgrammingSessionPrelude(channels)) {
			throw std::runtime_error("SBL programming-session broadcast failed");
		}
		common::UDSProtocolCommonSteps::keepAlive(currentChannel());
		if (!common::UDSProtocolCommonSteps::authorizeWithRetry(currentChannel(), canId, common::getPinArray(pin))) {
			throw std::runtime_error("SBL security access failed");
		}
		common::VBFParser vbfParser;
		std::ifstream sblVbf(sblPath, std::ios_base::binary);
		if (!sblVbf) {
			throw std::runtime_error("Failed to open SBL VBF: " + sblPath);
		}
		const common::VBF bootloader{ vbfParser.parse(sblVbf) };
		if (!common::UDSProtocolCommonSteps::transferData(currentChannel(), canId, bootloader, [](size_t) {})) {
			throw std::runtime_error("SBL transfer failed");
		}
		if (!common::UDSProtocolCommonSteps::startRoutine(currentChannel(), canId, bootloader.header.call)) {
			throw std::runtime_error("SBL start failed");
		}
		warmupPostStartRawChannel(channelProvider, channels, ecuId, canId);
	}

	if (session != 0) {
		if (!runUdsProbe(currentChannel(), canId, "Session", { 0x10, session }, { session }, 3000)) {
			throw std::runtime_error("Failed to enter requested diagnostic session");
		}
	}

	if (sblPath.empty() && !wake && session == 0) {
		LOG(WARNING) << "uds-raw sending without any wake/session prelude; "
			<< "if the bus is asleep or uninitialised the request will just time out "
			<< "(see --wake / --session)";
		std::cerr << "warning: no wake/session prelude; a timeout may mean the bus "
			<< "was never activated (try --wake / --session)" << std::endl;
	}

	try {
		for (size_t index = 0; index < rawData.size(); ++index) {
			const auto request = parseHexBytes(rawData[index]);
			if (request.empty()) {
				throw std::runtime_error("uds-raw request is empty");
			}
			std::cout << "UDS raw TX[" << (index + 1) << "]: ";
			printBytes(request);
			std::cout << std::endl;
			const bool allowFirstPostStartRetry = !sblPath.empty() && index == 0;
			const auto response = processRawUdsRequestWithOptionalRetry(
				channelProvider, channels, ecuId, canId, request, allowFirstPostStartRetry);
			std::cout << "UDS raw RX[" << (index + 1) << "]: ";
			printBytes(response);
			std::cout << std::endl;
		}
	}
	catch (...) {
		if (!sblPath.empty() && !noWakeup) {
			common::UDSProtocolCommonSteps::broadcastEcuReset(channels);
		}
		throw;
	}
	if (!sblPath.empty() && !noWakeup) {
		common::UDSProtocolCommonSteps::broadcastEcuReset(channels);
	}
}

void UDSDiag(common::CarPlatform carPlatform, uint8_t ecuId, j2534::J2534& j2534,
	std::optional<uint32_t> baudrateOverride)
{
	const auto ecuInfo{ common::getEcuInfoByEcuId(carPlatform, ecuId) };
	if (std::get<0>(ecuInfo).protocolId != ISO15765) {
		throw std::runtime_error("diag supports UDS/ISO15765 ECUs only");
	}
	const auto canId = std::get<1>(ecuInfo).canId;
	std::cout << "UDS diag probe: ECU 0x" << std::hex << static_cast<int>(ecuId)
		<< ", CAN ID 0x" << canId << std::dec << std::endl;

	common::J2534ChannelProvider channelProvider{ j2534, carPlatform, baudrateOverride };
	const auto channel = channelProvider.getChannelForEcu(ecuId);

	const bool extendedSessionOk = runUdsProbe(*channel, canId, "Extended session",
		{ 0x10, 0x03 }, { 0x03 }, 3000);
	if (!extendedSessionOk) {
		std::cout << "Extended session failed, skipping DID probes" << std::endl;
		return;
	}
	const auto keepAliveIds = common::UDSProtocolCommonSteps::keepAlive(*channel);
	runUdsProbe(*channel, canId, "Serial number F18C", { 0x22, 0xF1, 0x8C }, { 0xF1, 0x8C }, 3000);
	runUdsProbe(*channel, canId, "Boot SW ID F180", { 0x22, 0xF1, 0x80 }, { 0xF1, 0x80 }, 3000);
	runUdsProbe(*channel, canId, "Software ID F1AF", { 0x22, 0xF1, 0xAF }, { 0xF1, 0xAF }, 3000);
	channel->stopPeriodicMsg(keepAliveIds);
}

void UDSWakeup(common::CarPlatform carPlatform, uint8_t ecuId, j2534::J2534& j2534,
	std::optional<uint32_t> baudrateOverride)
{
	common::J2534ChannelProvider channelProvider{ j2534, carPlatform, baudrateOverride };
	auto channels = channelProvider.getUdsChannels(ecuId);
	if (channels.empty()) {
		throw std::runtime_error("Failed to open J2534 UDS channels");
	}
	common::UDSProtocolCommonSteps::broadcastEcuReset(channels);
	std::cout << "Wakeup frames sent" << std::endl;
}

bool startPeriodicOnAllChannels(const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels,
	const common::UDSMessage& message, std::vector<std::vector<unsigned long>>& msgIds,
	const std::string& label, unsigned long intervalMs = 20)
{
	msgIds.resize(channels.size());
	if (channels.empty()) {
		LOG(ERROR) << label << " failed: no open channels";
		return false;
	}
	bool success = true;
	for (size_t i = 0; i < channels.size(); ++i) {
		msgIds[i] = channels[i]->startPeriodicMsgs(message, intervalMs);
		if (msgIds[i].empty()) {
			LOG(ERROR) << label << " failed to start periodic message on channel " << i;
			success = false;
		}
	}
	return success;
}

void stopPeriodicOnAllChannels(const std::vector<std::unique_ptr<j2534::J2534Channel>>& channels,
	const std::vector<std::vector<unsigned long>>& msgIds)
{
	for (size_t i = 0; i < channels.size(); ++i) {
		if (!msgIds[i].empty()) {
			channels[i]->stopPeriodicMsg(msgIds[i]);
		}
	}
}

void UDSFunctionalEmergencyReset(common::CarPlatform carPlatform, uint8_t ecuId, j2534::J2534& j2534,
	std::optional<uint32_t> baudrateOverride)
{
	common::J2534ChannelProvider channelProvider{ j2534, carPlatform, baudrateOverride };
	auto channels = channelProvider.getUdsChannels(ecuId);
	if (channels.empty()) {
		throw std::runtime_error("Failed to open J2534 UDS channels");
	}
	std::vector<std::vector<unsigned long>> msgIds;
	const bool started = startPeriodicOnAllChannels(channels, common::UDSMessage(0x7DF, { 0x11, 0x81 }),
		msgIds, "Functional emergency reset", 20);
	common::waitForStopOrTimeout(std::chrono::milliseconds(200));
	stopPeriodicOnAllChannels(channels, msgIds);
	if (!started) {
		throw std::runtime_error("Functional emergency reset failed to start periodic messages");
	}
	std::cout << "Functional emergency reset sent: 7DF 11 81" << std::endl;
}

void UDSReset(common::CarPlatform carPlatform, uint8_t ecuId, j2534::J2534& j2534, bool functional,
	std::optional<uint32_t> baudrateOverride)
{
	if (functional) {
		UDSFunctionalEmergencyReset(carPlatform, ecuId, j2534, baudrateOverride);
		return;
	}

	const auto ecuInfo{ common::getEcuInfoByEcuId(carPlatform, ecuId) };
	if (std::get<0>(ecuInfo).protocolId != ISO15765) {
		throw std::runtime_error("reset supports UDS/ISO15765 ECUs only");
	}
	const auto canId = std::get<1>(ecuInfo).canId;
	common::J2534ChannelProvider channelProvider{ j2534, carPlatform, baudrateOverride };
	const auto channel = channelProvider.getChannelForEcu(ecuId);
	runUdsProbe(*channel, canId, "ECU hard reset", { 0x11, 0x01 }, { 0x01 }, 2000);
}

bool readCemProgramModeStatus(const j2534::J2534Channel& cemChannel, uint32_t cemCanId, uint8_t& status)
{
	try {
		common::UDSRequest sessionStatusRequest(cemCanId, { 0x22, 0xD1, 0x00 });
		const auto response = sessionStatusRequest.process(cemChannel, { 0xD1, 0x00 }, 1, 2000);
		if (response.empty()) {
			return false;
		}
		status = response[0];
		return true;
	}
	catch (const std::exception& ex) {
		std::cout << "CEM program mode status error: " << ex.what() << std::endl;
		return false;
	}
}

void UDSProgramMode(common::CarPlatform carPlatform, uint8_t /*ecuId*/, j2534::J2534& j2534,
	unsigned long holdSeconds, std::optional<uint32_t> baudrateOverride)
{
	constexpr uint8_t cemEcuId = 0x52;
	constexpr uint8_t kCemProgrammingActive = 0x02;

	const auto cemInfo{ common::getEcuInfoByEcuId(carPlatform, cemEcuId) };
	if (std::get<0>(cemInfo).protocolId != ISO15765) {
		throw std::runtime_error("program mode verification supports UDS/ISO15765 CEM only");
	}
	const auto cemCanId = std::get<1>(cemInfo).canId;

	common::J2534ChannelProvider channelProvider{ j2534, carPlatform, baudrateOverride };
	auto channels = channelProvider.getUdsChannels(cemEcuId);
	if (channels.empty()) {
		throw std::runtime_error("Failed to open J2534 UDS channels");
	}
	auto& cemChannel = common::getChannelByEcuId(carPlatform, cemEcuId, channels);

	uint8_t status = 0;
	if (readCemProgramModeStatus(cemChannel, cemCanId, status)) {
		std::cout << "CEM current programming status: 0x" << std::hex
			<< static_cast<int>(status) << std::dec << std::endl;
		if (status == kCemProgrammingActive) {
			std::cout << "Programming mode already active" << std::endl;
		}
	}

	{
		std::vector<std::vector<unsigned long>> msgIds;
		const bool started = startPeriodicOnAllChannels(channels,
			common::UDSMessage(0x7DF, { 0x10, 0x82 }), msgIds, "Program mode", 20);
		if (!started) {
			throw std::runtime_error("Program mode failed to start periodic messages");
		}
		common::waitForStopOrTimeout(std::chrono::milliseconds(180));
		stopPeriodicOnAllChannels(channels, msgIds);
	}

	status = 0;
	const bool verified = readCemProgramModeStatus(cemChannel, cemCanId, status);
	if (verified) {
		std::cout << "CEM programming status after request: 0x" << std::hex
			<< static_cast<int>(status) << std::dec << std::endl;
	}
	if (!verified || status != kCemProgrammingActive) {
		std::cout << "Program mode was not confirmed by CEM" << std::endl;
		throw std::runtime_error("Program mode was not confirmed by CEM");
	}
	else {
		std::cout << "Program mode OK" << std::endl;
	}

	if (holdSeconds > 0) {
		std::vector<std::vector<unsigned long>> msgIds;
		const bool started = startPeriodicOnAllChannels(channels,
			common::UDSMessage(0x7DF, { 0x3E, 0x80 }), msgIds, "TesterPresent hold", 1900);
		if (!started) {
			throw std::runtime_error("TesterPresent hold failed to start periodic messages");
		}
		std::cout << "Holding TesterPresent for " << holdSeconds << " seconds" << std::endl;
		common::waitForStopOrTimeout(std::chrono::seconds(holdSeconds));
		stopPeriodicOnAllChannels(channels, msgIds);
	}
}

uint8_t intelHexChecksum(const std::vector<uint8_t>& record)
{
	uint8_t sum = 0;
	for (const auto byte : record) {
		sum += byte;
	}
	return static_cast<uint8_t>((~sum + 1) & 0xFF);
}

void writeIntelHexRecord(std::ostream& output, uint8_t type, uint16_t address, const std::vector<uint8_t>& data)
{
	std::vector<uint8_t> record;
	record.reserve(data.size() + 5);
	record.push_back(static_cast<uint8_t>(data.size()));
	record.push_back(static_cast<uint8_t>((address >> 8) & 0xFF));
	record.push_back(static_cast<uint8_t>(address & 0xFF));
	record.push_back(type);
	record.insert(record.end(), data.cbegin(), data.cend());
	record.push_back(intelHexChecksum(record));

	output << ':';
	output << std::uppercase << std::hex << std::setfill('0');
	for (const auto byte : record) {
		output << std::setw(2) << static_cast<int>(byte);
	}
	output << '\n';
}

void saveIntelHex(const std::string& path, uint32_t start, const std::vector<uint8_t>& data)
{
	ensureOutputDirectoryExists(path);
	// Intel-HEX extended addresses wrap at 4 GiB; refuse instead of emitting a file whose
	// later records alias the beginning of the address space.
	if (start != 0 && data.size() > 0xFFFFFFFFull - start) {
		throw std::runtime_error("Read range wraps past the end of the 32-bit address space");
	}
	std::ofstream output(path);
	if (!output) {
		throw std::runtime_error("Failed to open output file: " + path);
	}
	uint32_t currentUpper = 0xFFFFFFFF;
	for (size_t offset = 0; offset < data.size();) {
		const uint32_t absoluteAddress = start + static_cast<uint32_t>(offset);
		const uint32_t upper = absoluteAddress >> 16;
		if (upper != currentUpper) {
			writeIntelHexRecord(output, 0x04, 0x0000, {
				static_cast<uint8_t>((upper >> 8) & 0xFF),
				static_cast<uint8_t>(upper & 0xFF)
			});
			currentUpper = upper;
		}

		const uint16_t lowAddress = static_cast<uint16_t>(absoluteAddress & 0xFFFF);
		const size_t bytesToSegmentEnd = 0x10000u - lowAddress;
		const size_t lineSize = std::min({ static_cast<size_t>(16), data.size() - offset, bytesToSegmentEnd });
		std::vector<uint8_t> line(data.cbegin() + offset, data.cbegin() + offset + lineSize);
		writeIntelHexRecord(output, 0x00, lowAddress, line);
		offset += lineSize;
	}
	writeIntelHexRecord(output, 0x01, 0x0000, {});
}

void saveRawBin(const std::string& path, const std::vector<uint8_t>& data)
{
	ensureOutputDirectoryExists(path);
	std::ofstream output(path, std::ios_base::binary | std::ios_base::out);
	if (!output) {
		throw std::runtime_error("Failed to open output file: " + path);
	}
	output.write(reinterpret_cast<const char*>(data.data()), data.size());
}

void saveReadResult(const std::string& path, uint32_t start, const std::vector<uint8_t>& data, ReadFormat format)
{
	if (format == ReadFormat::Hex) {
		saveIntelHex(path, start, data);
	}
	else {
		saveRawBin(path, data);
	}
}

// Writes <dump>.integrity.txt next to the saved artifact: the verdict, which algorithm matched,
// the CRC the ECU returned, both computed sums, block count, size and start address. Always
// written - even when the SBL sent no CRC, the file records that this was checked. This is the
// "quiet" contract: the console stays clean on success, but the evidence stays on disk.
//
// The body is produced by common::formatIntegrityReport, which prints algorithm only when the
// verdict is Ok and ecu_crc only when the response actually carried a CRC (see its tests).
void writeIntegrityReport(const std::string& dumpPath, const common::UploadReadResult& upload)
{
	const std::string reportPath = dumpPath + ".integrity.txt";
	std::ofstream report(reportPath);
	if (!report) {
		LOG(ERROR) << "Failed to write integrity report: " << reportPath;
		return;
	}
	report << common::formatIntegrityReport(upload);
	LOG(INFO) << "Integrity report written: " << reportPath;
}

void readFlash(std::unique_ptr<j2534::J2534> j2534, common::CarPlatform carPlatform, uint8_t ecuId,
	const std::string& flashPath, unsigned long start, unsigned long datasize, uint64_t pin,
	const std::string& sblPath, ProgramMode programMode, ReadFormat readFormat, bool attachRunningSbl,
	bool noSblAuth, bool skipFallAsleepCli, std::optional<uint32_t> baudrateOverride)
{
	const auto ecuInfo{ common::getEcuInfoByEcuId(carPlatform, ecuId) };
	if (std::get<0>(ecuInfo).protocolId == ISO15765) {
		if (sblPath.empty() && !attachRunningSbl) {
			throw std::runtime_error("SBL VBF is required for UDS reading; pass --sbl");
		}
		LOG(INFO) << "UDS read start platform=" << static_cast<int>(carPlatform)
			<< " ecu=0x" << std::hex << static_cast<int>(ecuId)
			<< " can=0x" << std::get<1>(ecuInfo).canId
			<< " start=0x" << start
			<< " size=0x" << datasize
			<< " output=" << flashPath
			<< " format=" << readFormatToString(readFormat)
			<< " sblPath=" << (sblPath.empty() ? "<none>" : sblPath)
			<< " programMode=" << programModeToString(programMode)
			<< " attachRunningSbl=" << attachRunningSbl;
		common::VBFParser vbfParser;
		std::unique_ptr<common::VBF> bootloader;
		if (!attachRunningSbl) {
			std::ifstream sblVbf(sblPath, std::ios_base::binary);
			if (!sblVbf) {
				throw std::runtime_error("Failed to open SBL VBF: " + sblPath);
			}
			bootloader = std::make_unique<common::VBF>(vbfParser.parse(sblVbf));
		}
		std::vector<uint8_t> bin;

		bool skipFallAsleep = skipFallAsleepCli;
		if (programMode == ProgramMode::Vehicle) {
			UDSProgramMode(carPlatform, ecuId, *j2534, 0, baudrateOverride);
			skipFallAsleep = true;
		}
		else {
			LOG(INFO) << "Bench program mode selected, skipping CEM programming mode"
				<< (skipFallAsleepCli ? "; --skip-fall-asleep set, skipping broadcast prelude" : "");
		}

		flasher::FlasherParameters flasherParameters{
			carPlatform,
			ecuId,
			"",
			attachRunningSbl ? nullptr : std::make_unique<flasher::SBLProviderVBF>(*bootloader),
			{{}, {}},
			baudrateOverride
		};
		flasher::UDSReaderParameters udsReaderParameters{
			common::getPinArray(pin),
			skipFallAsleep,
			attachRunningSbl,
			noSblAuth,
			static_cast<uint32_t>(start),
			static_cast<uint32_t>(datasize)
		};
		flasher::UDSReader flasher(*j2534, std::move(flasherParameters), std::move(udsReaderParameters), bin);
		FlasherCallback callback;
		flasher.registerCallback(callback);
		flasher.run();
		const bool success = flasher.getCurrentState() ==
			flasher::FlasherState::Done;
		std::cout << std::endl
			<< ((success)
				? "Reading done"
				: "Reading error. Try again.")
			<< std::endl;
		if (!success && !flasher.getLastError().empty()) {
			std::cout << "Last error: " << flasher.getLastError() << std::endl;
		}
		// A dump that was actually read (bytes arrived, size was verified) is always saved. The
		// integrity verdict is diagnostic: a CRC mismatch on the closing 77 is not a reason to
		// discard the payload, and it does not make the process exit non-zero.
		if (!bin.empty()) {
			saveReadResult(flashPath, static_cast<uint32_t>(start), bin, readFormat);
			writeIntegrityReport(flashPath, flasher.getLastUploadResult());
		}
		// Non-zero exit stays reserved for real read failures: NRC, timeout, undershoot, size
		// mismatch. Those leave the reader state off Done.
		if (!success) {
			throw std::runtime_error("Reading failed (see log for details)");
		}
		return;
	}
	throw std::runtime_error("Only UDS (ISO15765) P3 ECUs are supported");
}

std::string getSehModuleName(void* address)
{
	MEMORY_BASIC_INFORMATION info{};
	if (address == nullptr || VirtualQuery(address, &info, sizeof(info)) == 0 || info.AllocationBase == nullptr) {
		return "<unknown>";
	}
	char modulePath[MAX_PATH]{};
	if (GetModuleFileNameA(reinterpret_cast<HMODULE>(info.AllocationBase), modulePath, MAX_PATH) == 0) {
		return "<unknown>";
	}
	return modulePath;
}

LONG WINAPI SehLoggingFilter(EXCEPTION_POINTERS* ep) {
	const auto* record = ep ? ep->ExceptionRecord : nullptr;
	const auto code = record ? record->ExceptionCode : 0;
	void* address = record ? record->ExceptionAddress : nullptr;
	LOG(ERROR) << "Unhandled SEH 0x" << std::hex << code
		<< " at 0x" << reinterpret_cast<uintptr_t>(address)
		<< " module=" << getSehModuleName(address);
	if (record && code == EXCEPTION_ACCESS_VIOLATION && record->NumberParameters >= 2) {
		LOG(ERROR) << "Access violation "
			<< (record->ExceptionInformation[0] ? "write" : "read")
			<< " address=0x" << std::hex << record->ExceptionInformation[1];
	}
	return EXCEPTION_EXECUTE_HANDLER;
}

int main(int argc, const char* argv[]) {
    common::initLogger("VolvoFlasher.log", common::isDebugLoggingRequested(argc, argv));
    common::printRuntimeDiagnostics("VolvoFlasher");
    SetUnhandledExceptionFilter(SehLoggingFilter);
    common::installConsoleCtrlHandler();
	std::optional<unsigned long> baudrateOverride;
	std::string deviceName;
	std::string flashPath;
	std::string sblPath;
	common::CarPlatform carPlatform = common::CarPlatform::Undefined;
	uint64_t pin = 0;
	bool pinSpecified = false;
	unsigned long start = 0;
	unsigned long datasize = 0;
	uint8_t ecuId = 0;
	RunMode runMode = RunMode::None;
	bool scanPinsUpward = true;
	bool resetFunctional = false;
	unsigned long programHoldSeconds = 0;
	ProgramMode flashProgramMode = ProgramMode::Bench;
	ReadFormat readFormat = ReadFormat::Hex;
	std::vector<std::string> rawData;
	bool noWakeup = false;
	bool attachRunningSbl = false;
	bool udsRawWake = false;
	uint8_t udsRawSession = 0;
	bool noSblAuth = false;
	bool skipFallAsleep = false;
	common::PinSearchWindow pinWindow;
	bool pinWindowSet = false;
	bool cliArgsError = false;
	const auto devices = common::getAvailableDevices();
	const bool optionsOk = getRunOptions(argc, argv, deviceName, baudrateOverride, flashPath, pin, pinSpecified, ecuId, start, datasize,
		runMode, sblPath, carPlatform, scanPinsUpward, resetFunctional, programHoldSeconds, flashProgramMode,
		readFormat, rawData, noWakeup, attachRunningSbl,
		udsRawWake, udsRawSession, noSblAuth, skipFallAsleep, pinWindow, pinWindowSet, cliArgsError);
	if (cliArgsError) {
		return 2;
	}
	if (optionsOk) {
		j2534::DeviceInfo device;
		try {
			device = common::selectSingleDevice(devices, deviceName);
		}
		catch (const std::exception& ex) {
			LOG(WARNING) << ex.what();
			std::cout << ex.what() << std::endl;
			if (devices.empty()) {
				common::printJ2534ArchitectureHint(std::cout);
			}
			return 1;
		}
		try {
			std::unique_ptr<j2534::J2534> j2534{ common::openJ2534Device(device) };
			LOG(INFO) << "Selected device=" << device.deviceName
				<< " library=" << device.libraryName
				<< " mode=" << static_cast<int>(runMode)
				<< " platform=" << static_cast<int>(carPlatform)
				<< " ecu=0x" << std::hex << static_cast<int>(ecuId)
				<< " baudrate=" << std::dec << (baudrateOverride ? std::to_string(*baudrateOverride) : std::string("config"))
				<< " input=" << flashPath;
			// Print the resolved target loudly so a forgotten/wrong --ecu (which otherwise
			// silently picks a different module's CAN ids and just times out) is obvious.
			try {
				const auto targetInfo{ common::getEcuInfoByEcuId(carPlatform, ecuId) };
				const auto& targetBus = std::get<0>(targetInfo);
				const auto& targetEcu = std::get<1>(targetInfo);
				std::cerr << "[target] ecu=0x" << std::hex << static_cast<int>(ecuId)
					<< " canId=0x" << targetEcu.canId << std::dec
					<< " bus=\"" << targetBus.name << "\""
					<< " ecuName=\"" << targetEcu.name << "\""
					<< " baudrate=" << targetBus.baudrate << std::endl;
			}
			catch (const std::exception& ex) {
				LOG(INFO) << "resolved-target banner skipped: " << ex.what();
			}
			// If no --pin was given, fall back to a publicly known SecurityAccess PIN for the
			// target ECU from the configuration. --pin always wins; the config value only fills
			// the gap for read/flash/raw against an ECU that has a known key.
			if (!pinSpecified
				&& (runMode == RunMode::Read || runMode == RunMode::Flash || runMode == RunMode::UdsRaw)) {
				try {
					const auto ecuInfo{ common::getEcuInfoByEcuId(carPlatform, ecuId) };
					const auto& targetEcu = std::get<1>(ecuInfo);
					if (targetEcu.hasSecurityPin()) {
						pin = common::securityPinToUint64(targetEcu.securityPin);
						LOG(INFO) << "using known PIN from configuration for " << targetEcu.name;
						std::cout << "Using known PIN from configuration for " << targetEcu.name << std::endl;
					}
				}
				catch (const std::exception& ex) {
					LOG(INFO) << "known-PIN lookup skipped: " << ex.what();
				}
			}
			if (runMode == RunMode::Wakeup) {
				UDSWakeup(carPlatform, ecuId, *j2534, baudrateOverride);
			}
			else if (runMode == RunMode::Pin) {
				if (!pinSpecified && pinWindowSet) {
					// Expert explicit window without a start PIN: sweep it top-down.
					const auto start = pinWindow.ceilPin ? *pinWindow.ceilPin : *pinWindow.floorPin;
					findPin2(*j2534, carPlatform, ecuId, start, false, pinWindow);
				}
				else if (pinSpecified) {
					findPin2(*j2534, carPlatform, ecuId, pin, scanPinsUpward, pinWindow);
				}
				else {
					// Default CEM online search: the FFFF-prefixed window swept downward from
					// its top; the scan stops at the floor instead of running away below it.
					findPin2(*j2534, carPlatform, ecuId, common::kCemPinWindowCeil, false,
						common::PinSearchWindow{ common::kCemPinWindowFloor, common::kCemPinWindowCeil });
				}
			}
			else if (runMode == RunMode::Read) {
				readFlash(std::move(j2534), carPlatform, ecuId, flashPath, start, datasize,
					pin, sblPath, flashProgramMode, readFormat, attachRunningSbl, noSblAuth,
					skipFallAsleep, baudrateOverride);
			}
			else if (runMode == RunMode::Flash) {
				const auto ecuInfo{ common::getEcuInfoByEcuId(carPlatform, ecuId) };
				if (std::get<0>(ecuInfo).protocolId != ISO15765) {
					throw std::runtime_error("Only UDS (ISO15765) P3 ECUs are supported");
				}
				UDSFlash(carPlatform, ecuId, std::move(j2534), baudrateOverride, pin, flashPath, sblPath, flashProgramMode,
					attachRunningSbl, skipFallAsleep);
			}
			else if (runMode == RunMode::Diag) {
				UDSDiag(carPlatform, ecuId, *j2534, baudrateOverride);
			}
			else if (runMode == RunMode::Reset) {
				UDSReset(carPlatform, ecuId, *j2534, resetFunctional, baudrateOverride);
			}
			else if (runMode == RunMode::Program) {
				UDSProgramMode(carPlatform, ecuId, *j2534, programHoldSeconds, baudrateOverride);
			}
			else if (runMode == RunMode::UdsRaw) {
				UDSRaw(carPlatform, ecuId, *j2534, pin, rawData, sblPath, flashProgramMode, noWakeup,
					udsRawWake, udsRawSession, baudrateOverride);
			}
		}
		catch (const std::exception& ex) {
			LOG(ERROR) << "Command failed: " << ex.what();
			std::cout << ex.what() << std::endl;
			return 1;
		}
		catch (const char* ex) {
			LOG(ERROR) << "Command failed: " << ex;
			std::cout << ex << std::endl;
			return 1;
		}
		catch (...) {
			LOG(ERROR) << "Command failed with unknown exception";
			std::cout << "exception" << std::endl;
			return 1;
		}
	}
	else {
		common::printAvailableDevices(std::cout, devices);
	}
	return 0;
}
