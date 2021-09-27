#include <cstdint>

#include "esp_log.h"

#include "homer_util.hpp"
#include "homer_i2c.hpp"
#include "homer_sensor.hpp"
#include "sht3x.hpp"

using std::uint8_t;
using std::uint16_t;

namespace homer1 {

namespace {

const char* TAG = "SHT3X";

bool crc_matches(const uint8_t msb,
                 const uint8_t lsb,
                 const uint8_t expected_crc) noexcept
{
    uint8_t crc = 0xFF ^ msb;

    for (uint8_t i = 0; i < 8; i++)
        crc = crc & 0x80
              ? (crc << 1) ^ 0x31
              : crc << 1;

    crc ^= lsb;
    for (uint8_t i = 0; i < 8; i++)
        crc = crc & 0x80
              ? (crc << 1) ^ 0x31
              : crc << 1;

    return crc == expected_crc;
}

}

namespace Sht3x {

const char* err_to_string(const uint64_t err) noexcept
{
    switch (err) {
        case ERROR_BAD_CRC_TEMPERATURE:
            return "bad_crc_temperature";

        case ERROR_BAD_CRC_HUMIDITY:
            return "bad_crc_humidity";

        default:
            return nullptr;
    }
}

}

namespace Sht3x {

SensorData& SensorData::operator=(const SensorData& other) noexcept
{
    if (this == &other)
        return *this;

    this->temperature = other.temperature;
    this->humidity = other.humidity;

    HomerSensorData::operator=(other);

    return *this;
}

SensorData& SensorData::operator=(SensorData&& other) noexcept
{
    if (this == &other)
        return *this;

    this->temperature = other.temperature;
    this->humidity = other.humidity;

    HomerSensorData::operator=(std::move(other));

    return *this;
}

SensorData::SensorData(const SensorData& other) noexcept:
        HomerSensorData(other)
{
    this->temperature = other.temperature;
    this->humidity = other.humidity;
}

SensorData::SensorData(SensorData&& other) noexcept:
        HomerSensorData(std::move(other))
{
    this->temperature = other.temperature;
    this->humidity = other.humidity;
}

SensorData::SensorData() noexcept:
        HomerSensorData(),
        temperature{std::numeric_limits<float>::quiet_NaN()},
        humidity{std::numeric_limits<float>::quiet_NaN()}
{
}


void SensorData::do_dump(HomerSensorDump& map) const noexcept
{
    insert(map, SENSOR_ATTR_TEMPERATURE, this->temperature);
    insert(map, SENSOR_ATTR_HUMIDITY, this->humidity);
}

void SensorData::do_dump(std::stringstream& ss) const noexcept
{
    ss << "temperature: " << this->temperature << endl;
    ss << "humidity:    " << this->humidity << endl;
}

void SensorData::invalidate() noexcept
{
    this->temperature = std::numeric_limits<float>::quiet_NaN();
    this->humidity = std::numeric_limits<float>::quiet_NaN();
}

const char* SensorData::do_sensor_err_to_str(const uint64_t err) const noexcept
{
    return err_to_string(err);
}

}

namespace Sht3x {

Sensor::Sensor(i2c::Device i2c) noexcept:
        HomerSensor<SensorData>(MEASUREMENT_GAP_MILLIS),
        i2c{std::move(i2c)},
        data{}
{
}

Sensor::Sensor(Sensor&& other) noexcept:
        HomerSensor<SensorData>(std::move(other)),
        i2c{std::move(other.i2c)},
        data{std::move(other.data)}
{
}

Sensor& Sensor::operator=(Sensor&& other) noexcept
{
    if (this == &other)
        return *this;

    this->i2c = std::move(other.i2c);
    this->data = std::move(other.data);

    HomerSensor::operator=(std::move(other));

    return *this;
}

void Sensor::refresh_data() noexcept
{
    auto err = this->i2c.write(0x24, 0x00);
    this->data._get_error().merge_from(err);
    if (err.has_error()) {
        ESP_LOGE(TAG, "i2c write failed");
        return;
    }

    my_sleep_millis(30);

    uint8_t read[6];
    err = this->i2c.read_from_slave(read, 6);
    this->data._get_error().merge_from(err);
    if (err.has_error()) {
        ESP_LOGE(TAG, "i2c read from slave failed");
        return;
    }

    if (!crc_matches(read[0], read[1], read[2])) {
        this->data._get_error().add_sensor_err(ERROR_BAD_CRC_TEMPERATURE);
        ESP_LOGE(TAG, "CRC mismatch for temperature");
        return;
    }
    if (!crc_matches(read[3], read[4], read[5])) {
        this->data._get_error().add_sensor_err(ERROR_BAD_CRC_HUMIDITY);
        ESP_LOGE(TAG, "CRC mismatch for humidity");
        return;
    }

    const uint16_t t = (read[0] << 8) + read[1];
    const uint16_t h = (read[3] << 8) + read[4];

    this->data.temperature = static_cast<float>(t * 0.00267033 - 45.);
    this->data.humidity = static_cast<float>(h * 0.0015259);
    this->data._get_error().mark_ok_has_data();
}

SensorData& Sensor::get_raw_data() noexcept
{
    return this->data;
}

}

}