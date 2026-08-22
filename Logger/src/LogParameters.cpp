#include "logger/LogParameters.hpp"

#include <common/Util.hpp>

#include <easylogging++.h>

#pragma warning(push)
#pragma warning(disable : 4996)
#include "../fast-cpp-csv-parser/csv.h"
#pragma warning(pop)

#include <cctype>
#include <fstream>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace logger {

	DataType getDataType(const std::string& input) {
		if (common::toLower(input) == "f")
			return DataType::Float;
		if (common::toLower(input) == "i" || common::toLower(input) == "int" || input.empty())
			return DataType::Int;

		throw std::invalid_argument("expected \"I\" or \"F\"");
	}

	using common::trim;

	template <typename Reader> void LogParameters::load(Reader& reader) {
		std::string name;
		std::string addr;
		std::string size;
		std::string bitmask;
		std::string unit;
		std::string dataType;
		std::string isSigned;
		std::string isInverseConversion;
		std::string factor;
		std::string offset;
		std::string description;
		reader.read_header(io::ignore_extra_column | io::ignore_missing_column, "Name", "Address", "Size", "DataType",
			"Bitmask", "Unit", "Signed", "I", "Factor", "Offset",
			"Comment");
		while (reader.read_row(name, addr, size, dataType, bitmask, unit, isSigned,
			isInverseConversion, factor, offset, description)) {
			const auto csvError = [&reader, &name](const char* column, const std::string& reason) {
				return std::runtime_error(std::string("Invalid value in log parameters CSV at line ")
					+ std::to_string(reader.get_file_line()) + ", parameter \"" + name
					+ "\", column \"" + column + "\": " + reason);
			};
			// Required numeric columns (no default): empty value is an error.
			const auto parseUnsigned = [&csvError](const std::string& value, const char* column, int base) {
				try {
					size_t processedChars = 0;
					const auto trimmed = trim(value);
					const auto parsed = std::stoul(trimmed, &processedChars, base);
					if (processedChars != trimmed.size()) {
						throw std::invalid_argument("unexpected trailing characters");
					}
					return parsed;
				}
				catch (const std::exception& ex) {
					throw csvError(column, ex.what());
				}
			};
			// Optional numeric columns: a blank value falls back to the given default.
			const auto parseUnsignedOr = [&parseUnsigned](const std::string& value, const char* column,
				int base, unsigned long defaultValue) {
				return trim(value).empty() ? defaultValue : parseUnsigned(value, column, base);
			};
			const auto parseDoubleOr = [&csvError](const std::string& value, const char* column, double defaultValue) {
				const auto trimmed = trim(value);
				if (trimmed.empty()) {
					return defaultValue;
				}
				try {
					size_t processedChars = 0;
					const auto parsed = std::stod(trimmed, &processedChars);
					if (processedChars != trimmed.size()) {
						throw std::invalid_argument("unexpected trailing characters");
					}
					return parsed;
				}
				catch (const std::exception& ex) {
					throw csvError(column, ex.what());
				}
			};
			const auto parseBoolOr = [&parseUnsignedOr, &csvError](const std::string& value, const char* column) {
				const auto parsed = parseUnsignedOr(value, column, 10, 0);
				if (parsed > 1) {
					throw csvError(column, "expected 0 or 1");
				}
				return parsed != 0;
			};
			const auto parseDataType = [&csvError](const std::string& value) {
				try {
					return getDataType(trim(value));
				}
				catch (const std::exception& ex) {
					throw csvError("DataType", ex.what());
				}
			};
			const auto parsedSize = parseUnsigned(size, "Size", 10);
			if (parsedSize == 0 || parsedSize > 4) {
				throw csvError("Size", "expected 1..4 bytes");
			}
			const auto parsedDataType = parseDataType(dataType);
			const auto parsedBitmask = parseUnsignedOr(bitmask, "Bitmask", 16, 0);
			const auto parsedIsSigned = parseBoolOr(isSigned, "Signed");
			if (parsedBitmask != 0) {
				const auto maxValue = parsedSize == 4 ? std::numeric_limits<uint32_t>::max() : ((1u << (parsedSize * 8)) - 1);
				if (parsedBitmask > maxValue) {
					throw csvError("Bitmask", "does not fit into Size bytes");
				}
			}
			if (parsedDataType == DataType::Float) {
				if (parsedSize != 4) {
					throw csvError("Size", "float parameters must be 4 bytes");
				}
				if (parsedBitmask != 0) {
					throw csvError("Bitmask", "float parameters cannot use bitmask");
				}
				if (parsedIsSigned) {
					throw csvError("Signed", "float parameters cannot be signed");
				}
			}
			const auto parsedFactor = parseDoubleOr(factor, "Factor", 1.0);
			if (parsedFactor == 0.0) {
				LOG(WARNING) << "Log parameter \"" << name << "\" has Factor=0; its logged value will always be "
					<< (parseBoolOr(isInverseConversion, "I") ? "0" : "just the Offset");
			}
			_parameters.emplace_back(name, parseUnsigned(addr, "Address", 16), parsedSize,
				parsedDataType, parsedBitmask, unit,
				parsedIsSigned, parseBoolOr(isInverseConversion, "I"),
				parsedFactor, parseDoubleOr(offset, "Offset", 0.0), description);
		}
		if (_parameters.empty()) {
			throw std::runtime_error("Log parameters CSV contains no parameters");
		}
	}

	LogParameters::LogParameters(const std::string& path) {
		// Open through std::ifstream and hand the stream to the named-reader constructor:
		// CSVReader's path constructor uses a narrow fopen, which cannot open non-ASCII
		// (e.g. Cyrillic) paths on Windows. The ifstream constructor handles them natively.
		std::ifstream stream(path);
		if (!stream.is_open()) {
			throw std::runtime_error("Cannot open log parameters file: " + path);
		}
		io::CSVReader<11> reader{ "log.params", stream };
		load(reader);
	}

	LogParameters::LogParameters(std::istream& stream) {
		io::CSVReader<11> reader{ "log.params", stream };
		load(reader);
	}

	const LogParameters& LogParameters::operator=(const LogParameters& rhs) {
		if (this == &rhs)
			return *this;

		_parameters = rhs._parameters;

		return *this;
	}

	const std::vector<LogParameter>& LogParameters::parameters() const {
		return _parameters;
	}

} // namespace logger
