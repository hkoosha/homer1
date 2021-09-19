#pragma once

#include <cstdint>
#include <sstream>

#include "i2c.hpp"

using std::uint8_t;
using std::uint16_t;
using std::uint64_t;


namespace homer1 {
namespace Sht3x {

const uint64_t ERROR_NONE = i2c::ERROR_NONE;
//---
const uint64_t ERROR_DATA_NOT_AVAILABLE_YET = i2c::ERROR_MAX << 1;
const uint64_t ERROR_REFRESH_DATA = ERROR_DATA_NOT_AVAILABLE_YET << 1;
const uint64_t ERROR_BAD_CRC_TEMPERATURE = ERROR_REFRESH_DATA << 1;
const uint64_t ERROR_BAD_CRC_HUMIDITY = ERROR_BAD_CRC_TEMPERATURE << 1;

const uint8_t ADDR_1 = 0x44;
const uint32_t DELAY = 10;
const uint64_t MEASUREMENT_GAP_MILLIS = 2000;

struct SensorData final
{
    uint64_t error;
    uint64_t time_to_read;
    float temperature;
    float humidity;
};

class Sensor final
{
public:
    Sensor& operator=(const Sensor& other) = delete;

    Sensor& operator=(Sensor&& other) = delete;

    Sensor(const Sensor& other) = delete;


    Sensor(Sensor&& other) noexcept = default;

    explicit Sensor(i2c::Device i2c) noexcept;

    ~Sensor() noexcept = default;


    SensorData read_data() noexcept;


private:

    void refresh_data() noexcept;

    const i2c::Device i2c;

    uint64_t last_update{0};
    uint64_t error{ERROR_NONE};
    uint64_t time_to_read{0};
    float temperature{};
    float humidity{};
};


SensorData copy(const SensorData& data) noexcept;

void dump(const SensorData& data,
          std::stringstream& ss) noexcept;

}
}
