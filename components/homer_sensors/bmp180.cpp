#include <cmath>
#include <sstream>
#include <string>

#include "esp_log.h"

#include "homer_sensor.hpp"
#include "homer_util.hpp"
#include "bmp180.hpp"

using std::int16_t;
using std::uint8_t;
using std::uint32_t;
using std::uint64_t;

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

float calculate_temperature(const int32_t b5) noexcept
{
    const auto f = static_cast<float>((b5 + 8) >> 4);
    return f / 10.0F;
}

}

namespace Bmp180 {

SensorData::SensorData() noexcept:
        HomerSensorData(),
        pressure{std::numeric_limits<uint32_t>::max()},
        temperature{std::numeric_limits<float>::quiet_NaN()}
{
}

SensorData& SensorData::operator=(const SensorData& other) noexcept
{
    if (this == &other)
        return *this;

    this->pressure = other.pressure;
    this->temperature = other.temperature;

    HomerSensorData::operator=(other);

    return *this;
}

SensorData& SensorData::operator=(SensorData&& other) noexcept
{
    if (this == &other)
        return *this;

    this->pressure = other.pressure;
    this->temperature = other.temperature;

    HomerSensorData::operator=(std::move(other));

    return *this;
}

SensorData::SensorData(SensorData&& other) noexcept:
        HomerSensorData(std::move(other)),
        pressure{other.pressure},
        temperature{other.temperature}
{
}

void SensorData::do_dump(std::stringstream& ss) const noexcept
{
    ss << "temperature:  " << this->temperature << std::endl;
    ss << "pressure:     " << this->pressure << std::endl;
}

void SensorData::do_dump(HomerSensorDump& map) const noexcept
{
    assert(false);
}

void SensorData::invalidate() noexcept
{
    this->pressure = std::numeric_limits<uint32_t>::max();
    this->temperature = std::numeric_limits<float>::quiet_NaN();
}


// =============================================================================

Sensor::Sensor(i2c::Device i2c,
               const uint32_t reference_pressure,
               const int32_t oversampling) noexcept:
        HomerSensor(I2C_DELAY),
        reference_pressure{reference_pressure},
        oversampling{oversampling},
        i2c{std::move(i2c)},
        initialized{false},
        data{}
{
}

Sensor& Sensor::operator=(Sensor&& other) noexcept
{
    if (this == &other)
        return *this;

    this->reference_pressure = other.reference_pressure;
    this->oversampling = other.oversampling;
    this->i2c = std::move(other.i2c);
    this->initialized = other.initialized;
    this->data = std::move(other.data);

    this->ac1 = other.ac1;
    this->ac2 = other.ac2;
    this->ac3 = other.ac3;
    this->ac4 = other.ac4;
    this->ac5 = other.ac5;
    this->ac6 = other.ac6;
    this->b1 = other.b1;
    this->b2 = other.b2;
    this->mb = other.mb;
    this->mc = other.mc;
    this->md = other.md;

    HomerSensor::operator=(std::move(other));

    return *this;
}

Sensor::Sensor(Sensor&& other) noexcept:
        HomerSensor(std::move(other)),
        reference_pressure{other.reference_pressure},
        oversampling{other.oversampling},
        i2c{std::move(other.i2c)},
        initialized{other.initialized},
        data{std::move(other.data)}
{
}


void Sensor::refresh_data() noexcept
{
    if (!this->initialized) {
        ESP_LOGE(TAG, "BMP180 not initialized");
        this->data._get_error().add_sensor_err(ERROR_NOT_INITIALIZED);
        return;
    }

    int32_t b5;
    auto err = this->calculate_b5(b5);
    this->data._get_error().merge_from(err);
    if (err.has_error()) {
        ESP_LOGE(TAG, "refresh_data failed");
        return;
    }

    this->data.temperature = calculate_temperature(b5);

    if (this->reference_pressure > 0) {
        err = this->read_pressure(b5, this->data.pressure);
        this->data._get_error().merge_from(err);
        if (err.has_error()) {
            ESP_LOGE(TAG, "refresh_data failed");
            return;
        }
    }
    else {
        this->data.pressure = 0;
    }

    this->data._get_error().mark_ok_has_data();
}

HwErr Sensor::read_pressure(const int32_t b5,
                            uint32_t& pressure) noexcept
{
    uint32_t up;
    auto err = this->read_uncompensated_pressure(up);
    if (err.has_error()) {
        ESP_LOGE(TAG, "read_pressure failed");
        err.add_sensor_err(ERROR_READ_PRESSURE);
        return err;
    }

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
    return err;
}

HwErr Sensor::read_uncompensated_pressure(uint32_t& up) noexcept
{
    auto err = this->i2c.write(CONTROL, READ_PRESSURE_CMD + (this->oversampling << 6));
    if (err.has_error()) {
        ESP_LOGE(TAG, "read_uncompensated_pressure failed");
        err.add_sensor_err(ERROR_READ_UNCOMPENSATED_PRESSURE);
        return err;
    }

    my_sleep_ticks(2 + (3 << this->oversampling));

    err = this->i2c.read_uint32(DATA_TO_READ, up);
    if (err.has_error()) {
        ESP_LOGE(TAG, "read_uncompensated_pressure failed");
        err.add_sensor_err(ERROR_READ_UNCOMPENSATED_PRESSURE);
        return err;
    }

    up >>= (8 - this->oversampling);
    return err;
}

HwErr Sensor::read_uncompensated_temperature(int16_t& temp) noexcept
{
    auto err = this->i2c.write(CONTROL, READ_TEMP_CMD);
    if (err.has_error()) {
        ESP_LOGE(TAG, "read_uncompensated_temperature failed");
        err.add_sensor_err(ERROR_READ_UNCOMPENSATED_TEMPERATURE);
        return err;
    }

    my_sleep_ticks(5);

    err = this->i2c.read_int16(DATA_TO_READ, temp);
    if (err.has_error()) {
        ESP_LOGE(TAG, "read_uncompensated_temperature failed");
        err.add_sensor_err(ERROR_READ_UNCOMPENSATED_TEMPERATURE);
        return err;
    }

    return err;
}

HwErr Sensor::calculate_b5(int32_t& b5) noexcept
{
    int16_t ut;
    auto err = this->read_uncompensated_temperature(ut);
    if (err.has_error()) {
        ESP_LOGE(TAG, "calculate_b5 failed");
        err.add_sensor_err(ERROR_CALCULATE_B5);
        return err;
    }

    const int32_t x1 = ((ut - static_cast<int32_t>(this->ac6)) * static_cast<int32_t>(this->ac5)) >> 15;
    const int32_t x2 = (static_cast<int32_t>(this->mc) << 11) / (x1 + this->md);

    b5 = x1 + x2;
    return err;
}

HwErr Sensor::init() noexcept
{
    if (this->initialized) {
        ESP_LOGE(TAG, "already initialized");
        return HwErr::make_ok();
    }

    const uint8_t reg = 0x00;
    auto err = this->i2c.write_to_slave(&reg, 1);
    this->data._get_error().merge_from(err);
    if (err.has_error()) {
        ESP_LOGE(TAG, "init failed");
        return err;
    }

    ESP_LOGI(TAG, "sensor found at 0x%02x", I2C_ADDR);

    err = this->read_sensor_rom();
    this->data._get_error().merge_from(err);
    if (err.has_error()) {
        ESP_LOGE(TAG, "init failed");
        return err;
    }

    ESP_LOGI(TAG, "sensor initialized");
    this->initialized = true;
    return err;
}

HwErr Sensor::read_sensor_rom() noexcept
{
    auto err = this->i2c.read_int16(CAL_AC1, this->ac1);
    if (err.has_error()) {
        ESP_LOGE(TAG, "rom read failed: ac1");
        err.add_sensor_err(ERROR_READ_SENSOR_ROM);
        return err;
    }

    err = this->i2c.read_int16(CAL_AC2, this->ac2);
    if (err.has_error()) {
        ESP_LOGE(TAG, "rom read failed: ac2");
        err.add_sensor_err(ERROR_READ_SENSOR_ROM);
        return err;
    }

    err = this->i2c.read_int16(CAL_AC3, this->ac3);
    if (err.has_error()) {
        ESP_LOGE(TAG, "rom read failed: ac3");
        err.add_sensor_err(ERROR_READ_SENSOR_ROM);
        return err;
    }

    err = this->i2c.read_uint16(CAL_AC4, this->ac4);
    if (err.has_error()) {
        ESP_LOGE(TAG, "rom read failed: ac4");
        err.add_sensor_err(ERROR_READ_SENSOR_ROM);
        return err;
    }

    err = this->i2c.read_uint16(CAL_AC5, this->ac5);
    if (err.has_error()) {
        ESP_LOGE(TAG, "rom read failed: ac5");
        err.add_sensor_err(ERROR_READ_SENSOR_ROM);
        return err;
    }

    err = this->i2c.read_uint16(CAL_AC6, this->ac6);
    if (err.has_error()) {
        ESP_LOGE(TAG, "rom read failed: ac6");
        err.add_sensor_err(ERROR_READ_SENSOR_ROM);
        return err;
    }

    err = this->i2c.read_int16(CAL_B1, this->b1);
    if (err.has_error()) {
        ESP_LOGE(TAG, "rom read failed: b1");
        err.add_sensor_err(ERROR_READ_SENSOR_ROM);
        return err;
    }

    err = this->i2c.read_int16(CAL_B2, this->b2);
    if (err.has_error()) {
        ESP_LOGE(TAG, "rom read failed: b2");
        err.add_sensor_err(ERROR_READ_SENSOR_ROM);
        return err;
    }

    err = this->i2c.read_int16(CAL_MB, this->mb);
    if (err.has_error()) {
        ESP_LOGE(TAG, "rom read failed: mb");
        err.add_sensor_err(ERROR_READ_SENSOR_ROM);
        return err;
    }

    err = this->i2c.read_int16(CAL_MC, this->mc);
    if (err.has_error()) {
        ESP_LOGE(TAG, "rom read failed: mc");
        err.add_sensor_err(ERROR_READ_SENSOR_ROM);
        return err;
    }

    err = this->i2c.read_int16(CAL_MD, this->md);
    if (err.has_error()) {
        ESP_LOGE(TAG, "rom read failed: md");
        err.add_sensor_err(ERROR_READ_SENSOR_ROM);
        return err;
    }

    return err;
}

bool Sensor::is_initialized() const noexcept
{
    return this->initialized;
}

SensorData& Sensor::get_raw_data() noexcept
{
    return this->data;
}

}

}
