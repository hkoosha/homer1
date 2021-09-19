#pragma once

#include <cstdint>
#include <cstddef>
#include <iostream>

#include "esp_err.h"

#include "i2c.hpp"

using std::size_t;
using std::int16_t;
using std::uint16_t;
using std::uint32_t;
using std::uint64_t;


namespace homer1 {
namespace Bmp180 {

const TickType_t I2C_DELAY = 1000 / portTICK_RATE_MS;
const uint32_t SEA_LEVEL_PRESSURE = 101325;
const uint8_t I2C_ADDR = 0x77;

const uint64_t ERROR_NONE = i2c::ERROR_NONE;
//---
const uint64_t ERROR_INIT = i2c::ERROR_MAX << 1;
const uint64_t ERROR_READ_SENSOR_ROM = ERROR_INIT << 1;
const uint64_t ERROR_READ_UNCOMPENSATED_TEMPERATURE = ERROR_READ_SENSOR_ROM << 1;
const uint64_t ERROR_READ_UNCOMPENSATED_PRESSURE = ERROR_READ_UNCOMPENSATED_TEMPERATURE << 1;
const uint64_t ERROR_CALCULATE_B5 = ERROR_READ_UNCOMPENSATED_PRESSURE << 1;
const uint64_t ERROR_READ_PRESSURE = ERROR_CALCULATE_B5 << 1;


enum Mode
{
    // ULTRA_LOW_POWER = 0,
    // STANDARD = 1,
    // HIGH_RES = 2,
    ULTRA_HIGH_RES = 3,
};

struct SensorData final
{
    uint64_t error;
    uint64_t time_to_read;
    uint32_t pressure;
    float altitude;
    float temperature;
};

class Sensor final
{
public:

    Sensor& operator=(const Sensor& other) = delete;

    Sensor& operator=(Sensor&& other) = delete;

    Sensor(const Sensor& other) = delete;


    Sensor(Sensor&& other) noexcept = default;

    explicit Sensor(i2c::Device i2c,
                    Mode mode = Mode::ULTRA_HIGH_RES) noexcept;

    ~Sensor() noexcept = default;


    uint64_t init();

    bool is_initialized() const noexcept;

    SensorData read_data(uint32_t reference_pressure = 0) const;

private:

    static void read_altitude(uint32_t reference_pressure,
                              uint32_t absolute_pressure,
                              float& altitude) noexcept;

    static void read_temperature(int32_t b5,
                                 float& temperature) noexcept;

    uint64_t read_pressure(int32_t b5,
                           uint32_t& pressure) const noexcept;

    uint64_t read_uncompensated_pressure(uint32_t& up) const noexcept;

    uint64_t read_uncompensated_temperature(int16_t& temp) const noexcept;

    uint64_t calculate_b5(int32_t& b5) const noexcept;

    uint64_t read_sensor_rom() noexcept;


    const Mode oversampling;
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


SensorData copy(const SensorData& data) noexcept;

void dump(const SensorData& data,
          std::stringstream& ss) noexcept;

}
}
