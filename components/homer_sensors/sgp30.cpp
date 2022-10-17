#include <cstdint>
#include <cmath>

#include "esp_log.h"

#include "homer_util.hpp"
#include "homer_i2c.hpp"
#include "homer_sensor.hpp"
#include "sgp30.hpp"

using std::uint8_t;
using std::uint16_t;

namespace homer1 {

namespace {

uint8_t calculate_crc(const uint8_t* data, size_t len)
{
    uint8_t crc = 0xFF;
    for (auto i = 0; i < len; i++) {
        crc ^= data[i];
        for (auto b = 0; b < 8; b++)
            crc = (crc & 0x80)
                  ? (crc << 1) ^ 0x31
                  : crc << 1;
    }

    return crc;
}

}

// Data
namespace Sgp30 {

SensorData& SensorData::operator=(const SensorData& other) noexcept
{
    if (this == &other)
        return *this;

    this->tvoc = other.tvoc;
    this->eco2 = other.eco2;
    this->raw_h2 = other.raw_h2;
    this->raw_ethanol = other.raw_ethanol;

    this->serial_number0 = other.serial_number0;
    this->serial_number1 = other.serial_number1;
    this->serial_number2 = other.serial_number2;

    HomerSensorData::operator=(other);

    return *this;
}

SensorData& SensorData::operator=(SensorData&& other) noexcept
{
    if (this == &other)
        return *this;

    this->tvoc = other.tvoc;
    this->eco2 = other.eco2;
    this->raw_h2 = other.raw_h2;
    this->raw_ethanol = other.raw_ethanol;

    this->serial_number0 = other.serial_number0;
    this->serial_number1 = other.serial_number1;
    this->serial_number2 = other.serial_number2;

    HomerSensorData::operator=(std::move(other));

    return *this;
}

SensorData::SensorData(const SensorData& other) noexcept:
        HomerSensorData(other)
{
    this->tvoc = other.tvoc;
    this->eco2 = other.eco2;
    this->raw_h2 = other.raw_h2;
    this->raw_ethanol = other.raw_ethanol;

    this->serial_number0 = other.serial_number0;
    this->serial_number1 = other.serial_number1;
    this->serial_number2 = other.serial_number2;
}

SensorData::SensorData(SensorData&& other) noexcept:
        HomerSensorData(std::move(other.error),
                        other._time_to_read,
                        other._read_start,
                        other._name),
        tvoc{other.tvoc},
        eco2{other.eco2},
        raw_h2{other.raw_h2},
        raw_ethanol{other.raw_ethanol},
        serial_number0{other.serial_number0},
        serial_number1{other.serial_number1},
        serial_number2{other.serial_number2}
{
}

SensorData::SensorData() noexcept:
        HomerSensorData(NAME),
        tvoc{std::numeric_limits<uint16_t>::max()},
        eco2{std::numeric_limits<uint16_t>::max()},
        raw_h2{std::numeric_limits<uint16_t>::max()},
        raw_ethanol{std::numeric_limits<uint16_t>::max()},
        serial_number0{0},
        serial_number1{0},
        serial_number2{0}
{
}


void SensorData::do_dump(HomerSensorDumpMap& map) const noexcept
{
    map.insert({SENSOR_ATTR_TVOC, std::to_string(this->tvoc)});
    map.insert({SENSOR_ATTR_ECO2, std::to_string(this->eco2)});
    map.insert({SENSOR_ATTR_RAW_H2, std::to_string(this->raw_h2)});
    map.insert({SENSOR_ATTR_RAW_ETHANOL, std::to_string(this->raw_ethanol)});
}

void SensorData::do_dump(std::stringstream& ss) const noexcept
{
    ss << "tvoc:    " << this->tvoc << endl;
    ss << "eco2:    " << this->eco2 << endl;
    ss << "h2:      " << this->raw_h2 << endl;
    ss << "ethanol: " << this->raw_ethanol << endl;
}

void SensorData::invalidate() noexcept
{
    this->tvoc = std::numeric_limits<uint16_t>::max();
    this->eco2 = std::numeric_limits<uint16_t>::max();
    this->raw_h2 = std::numeric_limits<uint16_t>::max();
    this->raw_ethanol = std::numeric_limits<uint16_t>::max();
    this->serial_number0 = 0;
    this->serial_number1 = 0;
    this->serial_number2 = 0;
}

const char* SensorData::do_sensor_err_to_str(const uint64_t err) const noexcept
{
    switch (err) {
        case ERROR_I2C_CRC_MISMATCH:
            return "crc_mismatch";

        case ERROR_BAD_VALUE_GIVEN:
            return "bad_data_given";

        case ERROR_UNSUPPORTED_SENSOR:
            return "unsupported_sensor";

        default:
            return nullptr;
    }
}

void SensorData::serialize(Serializer& sz) const noexcept
{
    if (!this->error.is_ok())
        return;

    sz.write((uint8_t) 46)
            ->write(this->tvoc)
            ->write(this->eco2)
            ->write(this->raw_h2)
            ->write(this->raw_ethanol);
}

}

// Sensor
namespace Sgp30 {

Sensor::Sensor(i2c::Device* i2c) :
        HomerSensor<SensorData>(MEASUREMENT_GAP_MILLIS),
        i2c{i2c},
        data{},
        initialized{false}
{
    if (!this->i2c)
        throw std::runtime_error("i2c is null");
}

Sensor::Sensor(Sensor&& other) noexcept:
        HomerSensor<SensorData>(other._refresh_every, other._last_update),
        i2c{other.i2c},
        data{std::move(other.data)},
        initialized{other.initialized}
{
}

Sensor& Sensor::operator=(Sensor&& other) noexcept
{
    if (this == &other)
        return *this;

    this->i2c = other.i2c;
    this->data = std::move(other.data);
    this->initialized = other.initialized;

    HomerSensor::operator=(std::move(other));

    return *this;
}

void Sensor::refresh_data() noexcept
{
    auto err = this->measure();
    this->data._get_error().merge_from(err);
    if (err.has_error()) {
        ESP_LOGE(NAME, "i2c measure failed");
        return;
    }

    err = this->measure_raw();
    this->data._get_error().merge_from(err);
    if (err.has_error()) {
        ESP_LOGE(NAME, "i2c measure_raw failed");
        return;
    }

    // The stupid sensors is keep sensing 0 in between. Use a previously read
    // value if still relevant (read within 30 secs).
    if (this->data.tvoc == 0 && (now_millis() - this->_last_update) < 30000)
        this->data.tvoc = this->last_tvoc;
    else
        this->last_tvoc = this->data.tvoc;

    this->data._get_error().mark_ok_has_data();
}

SensorData& Sensor::get_raw_data() noexcept
{
    return this->data;
}

HwErr Sensor::init() noexcept
{
    if (this->initialized) {
        ESP_LOGE(NAME, "already initialized");
        return HwErr::make_ok();
    }

    this->i2c->set_delay(I2C_DELAY);

    const uint8_t feature_set_cmd[2] = {0x20, 0x2F};
    uint16_t feature_set;
    auto err = this->send_and_read(10, feature_set_cmd, 2, &feature_set, 1);
    if (!err.is_ok())
        return err;

    if ((feature_set & 0xF0) != 0x0020) {
        err.add_sensor_err(ERROR_UNSUPPORTED_SENSOR);
        return err;
    }

    my_sleep_millis(10);

    const uint8_t iaq_init_cmd[2] = {0x20, 0x03};
    err = this->send_and_read(10, iaq_init_cmd, 2, nullptr, 0);

    this->initialized = err.is_ok();

    return err;
}

bool Sensor::is_initialized() const noexcept
{
    return this->initialized;
}


HwErr Sensor::send_and_read(uint16_t delay_millis,
                            const uint8_t* cmd,
                            size_t cmd_len,
                            uint16_t* read,
                            size_t read_len) const noexcept
{
    this->i2c->set_delay(I2C_DELAY);

    auto err = this->i2c->write_to_slave(I2C_ADDR, cmd, cmd_len);
    if (!err.is_ok())
        return err;

    my_sleep_millis(delay_millis);

    if (read_len == 0)
        return err;

    auto reply_len = read_len * 3;
    uint8_t reply_buffer[reply_len];

    err = this->i2c->read_from_slave(I2C_ADDR, reply_buffer, reply_len);
    if (!err.is_ok())
        return err;

    for (auto i = 0; i < read_len; i++) {
        auto crc = calculate_crc(reply_buffer + i * 3, 2);
        if (crc != reply_buffer[i * 3 + 2]) {
            err.add_sensor_err(ERROR_I2C_CRC_MISMATCH);
            return err;
        }

        read[i] = reply_buffer[i * 3];
        read[i] <<= 8;
        read[i] |= reply_buffer[i * 3 + 1];
    }

    return err;
}

HwErr Sensor::set_absolute_humidity(float temperature,
                                    float humidity)
{
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15

    // [g/m^3]
    const double absolute_humidity =
            216.7 * ((humidity / 100.0) * 6.112 *
                     exp((17.62 * temperature) / (243.12 + temperature)) /
                     (273.15 + temperature));

    // [mg/m^3]
    const auto absolute_humidity_scaled = static_cast<uint32_t>(1000.0 * absolute_humidity);

    return this->set_absolute_humidity(absolute_humidity_scaled);
}

HwErr Sensor::set_absolute_humidity(uint32_t absolute_humidity)
{
    if (absolute_humidity > 256000)
        return HwErr::make_sensor_err(ERROR_BAD_VALUE_GIVEN);

    auto scaled = static_cast<uint16_t>(
            (static_cast<uint64_t>(absolute_humidity) * 256 * 16777) >> 24
    );

    uint8_t cmd[5];
    cmd[0] = 0x20;
    cmd[1] = 0x61;
    cmd[2] = scaled >> 8;
    cmd[3] = scaled & 0xFF;
    cmd[4] = calculate_crc(cmd + 2, 2);

    return this->send_and_read(10, cmd, 5, nullptr, 0);
}

HwErr Sensor::set_iaq_baseline(uint16_t eco2_base, uint16_t tvoc_base)
{
    uint8_t cmd[8];
    cmd[0] = 0x20;
    cmd[1] = 0x1e;
    cmd[2] = tvoc_base >> 8;
    cmd[3] = tvoc_base & 0xFF;
    cmd[4] = calculate_crc(cmd + 2, 2);
    cmd[5] = eco2_base >> 8;
    cmd[6] = eco2_base & 0xFF;
    cmd[7] = calculate_crc(cmd + 5, 2);

    return this->send_and_read(10, cmd, 8, nullptr, 0);
}

HwErr Sensor::measure_raw()
{
    const uint8_t cmd[] = {0x20, 0x50};
    uint16_t reply[2];

    auto err = this->send_and_read(25, cmd, 2, reply, 2);
    if (!err.is_ok())
        return err;

    this->data.raw_h2 = reply[0];
    this->data.raw_ethanol = reply[1];
    return err;
}

HwErr Sensor::measure()
{
    const uint8_t command[] = {0x20, 0x08};
    uint16_t reply[2];

    auto err = this->send_and_read(20, command, 2, reply, 2);
    if (!err.is_ok())
        return err;

    this->data.eco2 = reply[0];
    this->data.tvoc = reply[1];
    return err;
}


}

}
