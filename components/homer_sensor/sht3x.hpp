#pragma once

#include <cstdint>
#include <sstream>
#include <limits>

#include "i2c.hpp"
#include "homer_util.hpp"

using std::uint8_t;
using std::uint16_t;
using std::uint64_t;
using std::endl;


namespace homer1 {
namespace Sht3x {

const uint64_t ERROR_NONE = i2c::ERROR_NONE;
//---
const uint64_t ERROR_DATA_NOT_AVAILABLE_YET = i2c::ERROR_MAX << 1;
const uint64_t ERROR_REFRESH_DATA = ERROR_DATA_NOT_AVAILABLE_YET << 1;
const uint64_t ERROR_BAD_CRC_TEMPERATURE = ERROR_REFRESH_DATA << 1;
const uint64_t ERROR_BAD_CRC_HUMIDITY = ERROR_BAD_CRC_TEMPERATURE << 1;

const uint8_t I2C_ADDR = 0x44;
const uint32_t I2C_DELAY = 10;
const uint64_t MEASUREMENT_GAP_MILLIS = 2000;

struct SensorData final : public Dumper
{
    uint64_t error;
    uint64_t time_to_read;
    float temperature;
    float humidity;

    explicit SensorData() noexcept:
            error{ERROR_DATA_NOT_AVAILABLE_YET},
            time_to_read{std::numeric_limits<uint64_t>::max()},
            temperature{std::numeric_limits<float>::quiet_NaN()},
            humidity{std::numeric_limits<float>::quiet_NaN()}
    {
    }

    explicit SensorData(const uint64_t error,
                        const uint64_t time_to_read,
                        const float temperature,
                        const float humidity) noexcept:
            error{error},
            time_to_read{time_to_read},
            temperature{temperature},
            humidity{humidity}
    {
    }

    void dump(std::stringstream& ss) const noexcept override
    {
        ss << "ERR: " << uint64_to_bin(this->error) << endl;
        ss << "TTR: " << this->time_to_read << endl;
        ss << "temperature: " << this->temperature << endl;
        ss << "humidity:    " << this->humidity << endl;
    }

    bool has_data() const noexcept override
    {
        return (this->error & ERROR_DATA_NOT_AVAILABLE_YET) == ERROR_NONE;
    }
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

}
}
