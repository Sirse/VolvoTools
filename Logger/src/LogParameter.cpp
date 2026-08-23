#include "logger/LogParameter.hpp"

namespace {
    float processSign(uint32_t value, bool isSigned, size_t size) {
		if (isSigned) {
			switch (size) {
			case 1:
				return static_cast<float>(static_cast<int8_t>(value));
			case 2:
				return static_cast<float>(static_cast<int16_t>(value));
			case 3:
				// A plain int32_t cast would read a raw 0x800000 as +8388608: the sign
				// lives in bit 23 and must be extended from there.
				return static_cast<float>(static_cast<int32_t>((value & 0x800000) ? (value | 0xFF000000) : value));
			default:
				return static_cast<float>(static_cast<int32_t>(value));
			}
        }
		else {
			return static_cast<float>(value);
		}
	}
} // namespace

namespace logger {
	LogParameter::LogParameter(const std::string& name, uint32_t addr, size_t size,
		DataType dataType, uint32_t bitmask, const std::string& unit,
		bool isSigned, bool isInverseConversion,
		double factor, double offset,
		const std::string& description)
		: _name{ name }, _addr{ addr }, _size{ size }, _dataType{ dataType },
		_bitmask{ bitmask }, _unit{ unit }, _isSigned{ isSigned },
		_isInverseConversion{ isInverseConversion }, _factor{ factor },
		_offset{ offset }, _description{ description }
	{
	}

	const std::string& LogParameter::name() const { return _name; }

	uint32_t LogParameter::addr() const { return _addr; }

	size_t LogParameter::size() const { return _size; }

	DataType LogParameter::dataType() const { return _dataType; }

	uint32_t LogParameter::bitmask() const { return _bitmask; }

	const std::string& LogParameter::unit() const { return _unit; }

	const bool LogParameter::isSigned() const { return _isSigned; }

	const bool LogParameter::isInverseConversion() const {
		return _isInverseConversion;
	}

	double LogParameter::factor() const { return _factor; }

	double LogParameter::offset() const { return _offset; }

	const std::string& LogParameter::description() const { return _description; }

	static float ieee_float(uint32_t f) {
		static_assert(sizeof(float) == sizeof f, "`float` has a weird size.");
		float ret;
		std::memcpy(&ret, &f, sizeof(float));
		return ret;
	}

	double LogParameter::formatValue(uint32_t value) const {
        float float_value = 0;
		if (_dataType == DataType::Float) {
            float_value = ieee_float(value);
		}
        else {
            if (_bitmask) {
                value &= _bitmask;
            }
            float_value = processSign(value, _isSigned, _size);
        }
		if (_isInverseConversion) {
            const double denominator = float_value + _offset;
            if (denominator == 0.0) {
                return 0.0;
            }
            return _factor / denominator;
		}
		else {
            return float_value * _factor + _offset;
		}
	}

} // namespace logger
