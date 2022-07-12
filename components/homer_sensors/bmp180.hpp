#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>

#include "esp_err.h"

#include "homer_util.hpp"
#include "homer_i2c.hpp"
#include "homer_sensor.hpp"

using std::size_t;
using std::int16_t;
using std::uint16_t;
using std::uint32_t;
using std::uint64_t;


namespace homer1::Bmp180 {

const char* const NAME = "BMP180";

const TickType_t I2C_DELAY = 1000 / portTICK_PERIOD_MS;
const uint32_t SEA_LEVEL_PRESSURE = 101325;
const uint8_t I2C_ADDR = 0x77;
const int32_t OVERSAMPLING_ULTRA_HIGH_RES = 3;


const uint64_t ERROR_NOT_INITIALIZED = i2c::ERROR_MAX << 1;
const uint64_t ERROR_ALREADY_INITIALIZED = ERROR_NOT_INITIALIZED << 1;
const uint64_t ERROR_INIT = ERROR_ALREADY_INITIALIZED << 1;
const uint64_t ERROR_READ_SENSOR_ROM = ERROR_INIT << 1;
const uint64_t ERROR_READ_UNCOMPENSATED_TEMPERATURE = ERROR_READ_SENSOR_ROM << 1;
const uint64_t ERROR_READ_UNCOMPENSATED_PRESSURE = ERROR_READ_UNCOMPENSATED_TEMPERATURE << 1;
const uint64_t ERROR_CALCULATE_B5 = ERROR_READ_UNCOMPENSATED_PRESSURE << 1;
const uint64_t ERROR_READ_PRESSURE = ERROR_CALCULATE_B5 << 1;

const char* const SENSOR_ATTR_PRESSURE = "pressure";
const char* const SENSOR_ATTR_TEMPERATURE = "temperature";


const char* err_to_string(uint64_t err) noexcept;

class SensorData final : public HomerSensorData
{
public:
    SensorData& operator=(const SensorData& other) noexcept;

    SensorData& operator=(SensorData&& other) noexcept;

    SensorData(const SensorData& other) noexcept = default;

    SensorData(SensorData&& other) noexcept;

    SensorData() noexcept;

    ~SensorData() noexcept override = default;


    // TODO uint? or int? negative pressure? check datasheet again.
    uint32_t pressure;
    float temperature;

protected:
    void do_dump(std::stringstream& ss) const noexcept override;

    void do_dump(HomerSensorDump& map) const noexcept override;

    [[nodiscard]] const char* do_sensor_err_to_str(uint64_t err) const noexcept override;

    void invalidate() noexcept override;

};

class Sensor final : public HomerSensor<SensorData>
{
public:
    Sensor& operator=(const Sensor& other) = delete;

    Sensor(const Sensor& other) = delete;


    Sensor& operator=(Sensor&& other) noexcept;

    Sensor(Sensor&& other) noexcept;

    Sensor(i2c::Device i2c,
           uint32_t reference_pressure,
           int32_t oversampling) noexcept;


    ~Sensor() noexcept override = default;


    HwErr init() noexcept;

    [[nodiscard]] bool is_initialized() const noexcept;


protected:
    void refresh_data() noexcept override;

    SensorData& get_raw_data() noexcept override;

private:
    HwErr read_pressure(int32_t b5,
                        uint32_t& pressure) noexcept;

    HwErr read_uncompensated_pressure(uint32_t& up) noexcept;

    HwErr read_uncompensated_temperature(int16_t& temp) noexcept;

    HwErr calculate_b5(int32_t& b5) noexcept;

    HwErr read_sensor_rom() noexcept;


    uint32_t reference_pressure;
    int32_t oversampling = 3;
    i2c::Device i2c;
    bool initialized;
    SensorData data;

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
