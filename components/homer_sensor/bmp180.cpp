#include <stdexcept>
#include <cmath>
#include <sstream>
#include <cstddef>

#include "esp_log.h"

#include "util.hpp"
#include "bmp180.hpp"

using std::size_t;
using std::int16_t;
using std::uint8_t;
using std::uint16_t;
using std::uint32_t;
using std::uint64_t;
using std::endl;


namespace homer1 {

namespace {

const char* TAG = "Bmp180";


const uint8_t CAL_AC1 = 0xAA;
const uint8_t CAL_AC2 = 0xAC;
const uint8_t CAL_AC3 = 0xAE;
const uint8_t CAL_AC4 = 0xB0;
const uint8_t CAL_AC5 = 0xB2;
const uint8_t CAL_AC6 = 0xB4;
const uint8_t CAL_B1 = 0xB6;
const uint8_t CAL_B2 = 0xB8;
const uint8_t CAL_MB = 0xBA;
const uint8_t CAL_MC = 0xBC;
const uint8_t CAL_MD = 0xBE;

const uint8_t CONTROL = 0xF4;
const uint8_t DATA_TO_READ = 0xF6;
const uint8_t READ_TEMP_CMD = 0x2E;
const uint8_t READ_PRESSURE_CMD = 0x34;

}

namespace Bmp180 {

Sensor::Sensor(const i2c::Device i2c) noexcept:
        i2c{i2c},
        initialized{false}
{
}


// =============================================================================

SensorData Sensor::read_data(const uint32_t reference_pressure) const
{
    if (!this->initialized) {
        ESP_LOGE(TAG, "BMP180 not initialized");
        throw std::runtime_error("BMP180 not initialized");
    }

    SensorData data{};
    data.time_to_read = 0;
    const auto then = now_millis();

    int32_t b5;
    data.error = this->calculate_b5(b5);
    if (data.error != ERROR_NONE) {
        data.time_to_read = now_millis() - then;
        return data;
    }

    Sensor::read_temperature(b5, data.temperature);
    Sensor::read_altitude(reference_pressure, data.pressure, data.altitude);

    if (reference_pressure > 0) {
        data.error = this->read_pressure(b5, data.pressure);
        if (data.error != ERROR_NONE) {
            data.time_to_read = now_millis() - then;
            return data;
        }
    }
    else {
        data.pressure = 0;
    }

    data.time_to_read = now_millis() - then;
    return data;
}

void Sensor::read_altitude(const uint32_t reference_pressure,
                           const uint32_t absolute_pressure,
                           float& altitude) noexcept
{
    const auto x = static_cast<float>(absolute_pressure) / static_cast<float>(reference_pressure);
    altitude = static_cast<float>(44330 * (1.0 - powf(x, 0.190295)));
}

void Sensor::read_temperature(const int32_t b5,
                              float& temperature) noexcept
{
    auto f = static_cast<float>((b5 + 8) >> 4);
    temperature = f / 10.0F;
}

uint64_t Sensor::read_pressure(const int32_t b5,
                               uint32_t& pressure) const noexcept
{
    uint64_t err;

    uint32_t up;
    err = this->read_uncompensated_pressure(up);
    if (err != ERROR_NONE)
        return err | ERROR_READ_PRESSURE;

    const int32_t b6 = b5 - 4000;

    int32_t x1 = (this->b2 * (b6 * b6) >> 12) >> 11;
    int32_t x2 = (this->ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    const int32_t b3 = (((static_cast<int32_t>(this->ac1) * 4 + x3) << this->oversampling) + 2) >> 2;

    x1 = (this->ac3 * b6) >> 13;
    x2 = (this->b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    const uint32_t b4 = (this->ac4 * static_cast<uint32_t>(x3 + 32768)) >> 15;

    const uint32_t b7 = static_cast<uint32_t>(up - b3) * (50000 >> this->oversampling);

    int32_t p = b7 < 0x80000000
                ? (b7 << 1) / b4 // NOLINT(cppcoreguidelines-narrowing-conversions)
                : (b7 / b4) << 1; // NOLINT(cppcoreguidelines-narrowing-conversions)

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    pressure = p;
    return ERROR_NONE;
}

uint64_t Sensor::read_uncompensated_pressure(uint32_t& up) const noexcept
{
    uint64_t err;

    err = this->i2c.write(CONTROL, READ_PRESSURE_CMD + (this->oversampling << 6));
    if (err != ERROR_NONE)
        return err | ERROR_READ_UNCOMPENSATED_PRESSURE;

    my_sleep_ticks(2 + (3 << this->oversampling));

    err = this->i2c.read_uint32(DATA_TO_READ, up);
    if (err != ERROR_NONE)
        return err | ERROR_READ_UNCOMPENSATED_PRESSURE;

    up >>= (8 - this->oversampling);
    return ERROR_NONE;
}

uint64_t Sensor::read_uncompensated_temperature(int16_t& temp) const noexcept
{
    uint64_t err;

    err = this->i2c.write(CONTROL, READ_TEMP_CMD);
    if (err != ERROR_NONE)
        return err | ERROR_READ_UNCOMPENSATED_TEMPERATURE;

    my_sleep_ticks(5);

    err = this->i2c.read_int16(DATA_TO_READ, temp);
    if (err != ERROR_NONE)
        return err | ERROR_READ_UNCOMPENSATED_TEMPERATURE;

    return ERROR_NONE;
}

uint64_t Sensor::calculate_b5(int32_t& b5) const noexcept
{
    int16_t ut;

    const uint64_t err = this->read_uncompensated_temperature(ut);
    if (err != ERROR_NONE)
        return err | ERROR_CALCULATE_B5;

    const int32_t x1 = ((ut - static_cast<int32_t>(this->ac6)) * static_cast<int32_t>(this->ac5)) >> 15;
    const int32_t x2 = (static_cast<int32_t>(this->mc) << 11) / (x1 + this->md);

    b5 = x1 + x2;
    return ERROR_NONE;
}

uint64_t Sensor::init()
{
    if (this->initialized)
        throw std::runtime_error("BMP180 already initialized");

    uint64_t err;

    const uint8_t reg = 0x00;
    err = this->i2c.write_to_slave(&reg, 1);
    if (err != ERROR_NONE)
        return err | ERROR_INIT;

    ESP_LOGI(TAG, "BMP180 sensor found at 0x%02x", I2C_ADDR);

    err = this->read_sensor_rom();
    if (err != ERROR_NONE)
        return err | ERROR_READ_SENSOR_ROM;

    this->initialized = true;
    return ERROR_NONE;
}

uint64_t Sensor::read_sensor_rom() noexcept
{
    uint64_t err;

    err = this->i2c.read_int16(CAL_AC1, this->ac1);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "ac1 read failed");
        return err | ERROR_READ_SENSOR_ROM;
    }

    err = this->i2c.read_int16(CAL_AC2, this->ac2);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "ac2 read failed");
        return err | ERROR_READ_SENSOR_ROM;
    }

    err = this->i2c.read_int16(CAL_AC3, this->ac3);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "ac3 read failed");
        return err | ERROR_READ_SENSOR_ROM;
    }

    err = this->i2c.read_uint16(CAL_AC4, this->ac4);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "ac4 read failed");
        return err | ERROR_READ_SENSOR_ROM;
    }

    err = this->i2c.read_uint16(CAL_AC5, this->ac5);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "ac5 read failed");
        return err | ERROR_READ_SENSOR_ROM;
    }

    err = this->i2c.read_uint16(CAL_AC6, this->ac6);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "ac6 read failed");
        return err | ERROR_READ_SENSOR_ROM;
    }

    err = this->i2c.read_int16(CAL_B1, this->b1);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "b1 read failed");
        return err | ERROR_READ_SENSOR_ROM;
    }

    err = this->i2c.read_int16(CAL_B2, this->b2);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "b2 read failed");
        return err | ERROR_READ_SENSOR_ROM;
    }

    err = this->i2c.read_int16(CAL_MB, this->mb);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "mb read failed");
        return err | ERROR_READ_SENSOR_ROM;
    }

    err = this->i2c.read_int16(CAL_MC, this->mc);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "mc read failed");
        return err | ERROR_READ_SENSOR_ROM;
    }

    err = this->i2c.read_int16(CAL_MD, this->md);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "md read failed");
        return err | ERROR_READ_SENSOR_ROM;
    }

    return ERROR_NONE;
}

bool Sensor::is_initialized() const noexcept
{
    return this->initialized;
}
}

}
