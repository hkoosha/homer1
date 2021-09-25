#pragma once

#include <cstdint>
#include <cstddef>
#include <iostream>
#include <limits>

#include "esp_err.h"

#include "i2c.hpp"

using std::size_t;
using std::int16_t;
using std::uint16_t;
using std::uint32_t;
using std::uint64_t;
using std::endl;


namespace homer1 {
namespace Bmp180 {

const TickType_t I2C_DELAY = 1000 / portTICK_RATE_MS;
const uint32_t SEA_LEVEL_PRESSURE = 101325;
const uint8_t I2C_ADDR = 0x77;

const uint64_t ERROR_NONE = i2c::ERROR_NONE;
//---
const uint64_t ERROR_DATA_NOT_AVAILABLE = i2c::ERROR_MAX << 1;
const uint64_t ERROR_INIT = ERROR_DATA_NOT_AVAILABLE << 1;
const uint64_t ERROR_READ_SENSOR_ROM = ERROR_INIT << 1;
const uint64_t ERROR_READ_UNCOMPENSATED_TEMPERATURE = ERROR_READ_SENSOR_ROM << 1;
const uint64_t ERROR_READ_UNCOMPENSATED_PRESSURE = ERROR_READ_UNCOMPENSATED_TEMPERATURE << 1;
const uint64_t ERROR_CALCULATE_B5 = ERROR_READ_UNCOMPENSATED_PRESSURE << 1;
const uint64_t ERROR_READ_PRESSURE = ERROR_CALCULATE_B5 << 1;


struct SensorData final : public Dumper
{
    uint64_t error;
    uint64_t time_to_read;
    uint32_t pressure;
    float temperature;

    explicit SensorData() noexcept:
            error{ERROR_DATA_NOT_AVAILABLE},
            time_to_read{std::numeric_limits<uint64_t>::max()},
            pressure{std::numeric_limits<uint32_t>::max()},
            temperature{std::numeric_limits<float>::quiet_NaN()}
    {
    }

    explicit SensorData(const uint64_t error,
                        const uint64_t time_to_read,
                        const uint32_t pressure,
                        const float temperature) noexcept:
            error{error},
            time_to_read{time_to_read},
            pressure{pressure},
            temperature{temperature}
    {
    }

    void dump(std::stringstream& ss) const noexcept override
    {
        ss << "ERR: " << uint64_to_bin(this->error) << endl;
        ss << "TTR: " << this->time_to_read << endl;
        ss << "temperature:  " << this->temperature << endl;
        ss << "pressure:     " << this->pressure << endl;
    }

    bool has_data() const noexcept override
    {
        return (this->error & ERROR_DATA_NOT_AVAILABLE) == ERROR_NONE;
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


    uint64_t init();

    bool is_initialized() const noexcept;

    SensorData read_data(uint32_t reference_pressure = SEA_LEVEL_PRESSURE) const;

private:

    static void read_temperature(int32_t b5,
                                 float& temperature) noexcept;

    uint64_t read_pressure(int32_t b5,
                           uint32_t& pressure) const noexcept;

    uint64_t read_uncompensated_pressure(uint32_t& up) const noexcept;

    uint64_t read_uncompensated_temperature(int16_t& temp) const noexcept;

    uint64_t calculate_b5(int32_t& b5) const noexcept;

    uint64_t read_sensor_rom() noexcept;


    const int32_t oversampling = 3;
    const i2c::Device i2c;
    bool initialized{};

    int16_t ac1{};
    int16_t ac2{};
    int16_t ac3{};
    uint16_t ac4{};
    uint16_t ac5{};
    uint16_t ac6{};
    int16_t b1{};
    int16_t b2{};
    int16_t mb{};
    int16_t mc{};
    int16_t md{};
};

}
}
