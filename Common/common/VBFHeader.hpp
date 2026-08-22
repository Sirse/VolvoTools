#pragma once

#include <vector>
#include <string>

namespace common {

	enum class NetworkType {
		UNKNOWN,
		CAN_HS,
		CAN_MS
	};

	enum class FrameFormat {
		UNKNOWN,
		CAN_STANDARD,
		CAN_EXTENDED
	};

	enum class SWPartType {
		UNKNOWN,
		SBL,
		DATA,
		EXE,
		SIGCFG,
		CARCFG
	};

	enum class SessionType {
		DEFAULT,
		PROGRAMMING,
		EXTENDED,
		SAFETY_SYSTEM,
		OTHER
	};

	enum class FlashStrategy {
		INPLACE,
		SWAP,
		DUAL_BANK,
		BACKGROUND,
		TRI_BANK,
		SEQUENTIAL,
		OVERLAY,
		UNKNOWN
	};

	struct EraseBlock
	{
		uint32_t startAddr;
		uint32_t length;
	};

	struct ChecksumBlock
	{
		uint32_t startAddr;
		uint32_t endAddr;
		uint32_t checksum;
	};

	struct VBFHeader {
		double vbfVersion{};
		std::vector<std::string> description;
		std::string swPartNumber;
		std::string swVersion;
		SWPartType swPartType{ SWPartType::UNKNOWN };
		NetworkType network{ NetworkType::UNKNOWN };
		uint32_t ecuAddress{};
		FrameFormat frameFormat{ FrameFormat::UNKNOWN };
		uint32_t call{};
		// Volvo's data_format_identifier: 0x00 is plain data, anything else means the chunk
		// payload is packed (0x10 is the compressed variant). We can't unpack it, so this
		// mostly exists so we can refuse the file instead of flashing garbage into an ECU.
		uint8_t dataFormatIdentifier{};
		uint32_t fileChecksum{};
		std::vector<EraseBlock> eraseBlocks;
		std::vector<ChecksumBlock> checksumTable;
		FlashStrategy flashStrategy{ FlashStrategy::INPLACE };
		SessionType sessionType{ SessionType::PROGRAMMING };
		uint8_t securityAccessLevel{ 0x27 };
		std::string signature;
		std::string certificateIdentifier;
	};

	// True when the chunk payload is packed rather than plain data. We have no unpacker, so
	// writing such a file to an ECU would brick it - callers must refuse instead.
	//
	// Packed VBF uses Volvo's own LZSS variant. From the
	// SDA research (see volvotools_bug_review B-11): window 1024 (EI=10), match length 2..17
	// (EJ=4), threshold P=1, single continuous bitwise flag stream. A packed block keeps the same
	// addr(4)/len(4)/data(len)/crc16(2) layout, where len is the COMPRESSED length, and the CRC
	// covers the DECOMPRESSED bytes, so unpacking is self-verifying. No verified packed-VBF
	// sample is available in this repo yet; until one is, refuse rather than guess.
	inline bool isPackedDataFormat(const VBFHeader& header)
	{
		return header.dataFormatIdentifier != 0x00;
	}

} // namespace common
