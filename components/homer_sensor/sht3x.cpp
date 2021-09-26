#include <cstdint>
#include <iostream>
#include <limits>

#include "sht3x.hpp"
#include "i2c.hpp"
#include "homer_util.hpp"

using std::uint8_t;
using std::uint16_t;
using std::endl;

namespace homer1 {

namespace {

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

Sensor::Sensor(const i2c::Device i2c) noexcept:
        i2c{i2c}
{
}

SensorData Sensor::read_data() noexcept
{
    if (this->last_update == 0 || (this->last_update + MEASUREMENT_GAP_MILLIS <= now_millis()))
        this->refresh_data();

    const auto has_error = this->error != ERROR_NONE;
    return SensorData{
            this->error,
            this->time_to_read,
            has_error ? std::numeric_limits<float>::quiet_NaN() : this->temperature,
            has_error ? std::numeric_limits<float>::quiet_NaN() : this->humidity,
    };
}

void Sensor::refresh_data() noexcept
{
    const auto then = now_millis();

    this->error = this->i2c.write(0x24, 0x00);
    if (this->error != ERROR_NONE) {
        this->time_to_read = now_millis() - then;
        return;
    }

    my_sleep_millis(30);

    uint8_t data[6];
    this->error = this->i2c.read_from_slave(data, 6);
    if (this->error != ERROR_NONE) {
        this->time_to_read = now_millis() - then;
        return;
    }

    if (!crc_matches(data[0], data[1], data[2])) {
        this->time_to_read = now_millis() - then;
        this->error = ERROR_BAD_CRC_TEMPERATURE;
        return;
    }
    if (!crc_matches(data[3], data[4], data[5])) {
        this->time_to_read = now_millis() - then;
        this->error = ERROR_BAD_CRC_HUMIDITY;
        return;
    }

    const uint16_t t = (data[0] << 8) + data[1];
    const uint16_t h = (data[3] << 8) + data[4];

    this->temperature = static_cast<float>(t * 0.00267033 - 45.);
    this->humidity = static_cast<float>(h * 0.0015259);

    const auto now = now_millis();
    this->error = ERROR_NONE;
    this->time_to_read = now - then;
    this->last_update = now;
}

}
}