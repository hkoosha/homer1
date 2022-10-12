#include <stdexcept>
#include <cstdint>
#include <cstddef>

#include "esp_log.h"
#include "driver/uart.h"

#include "homer_util.hpp"
#include "homer_sensor.hpp"
#include "pms5003.hpp"

using std::size_t;
using std::uint16_t;
using std::int32_t;


namespace homer1 {

namespace {

const auto UART_READ_WAIT = 500;
const auto UART_READ_WAIT_TICKS = UART_READ_WAIT / portTICK_PERIOD_MS;

const auto MAX_RETRIES = 10;

const auto MAGIC0 = 0x42;
const auto MAGIC1 = 0x4D;

}

namespace Pms5003 {

SensorData& SensorData::operator=(const SensorData& other) noexcept
{
    if (this == &other)
        return *this;

    this->pm10_standard = other.pm10_standard;
    this->pm25_standard = other.pm25_standard;
    this->pm100_standard = other.pm100_standard;
    this->pm10_env = other.pm10_env;
    this->pm25_env = other.pm25_env;
    this->pm100_env = other.pm100_env;
    this->particles_03 = other.particles_03;
    this->particles_05 = other.particles_05;
    this->particles_10 = other.particles_10;
    this->particles_25 = other.particles_25;
    this->particles_50 = other.particles_50;
    this->particles_100 = other.particles_100;

    this->_name = other._name;

    HomerSensorData::operator=(other);

    return *this;
}

SensorData& SensorData::operator=(SensorData&& other) noexcept
{
    if (this == &other)
        return *this;

    this->pm10_standard = other.pm10_standard;
    this->pm25_standard = other.pm25_standard;
    this->pm100_standard = other.pm100_standard;
    this->pm10_env = other.pm10_env;
    this->pm25_env = other.pm25_env;
    this->pm100_env = other.pm100_env;
    this->particles_03 = other.particles_03;
    this->particles_05 = other.particles_05;
    this->particles_10 = other.particles_10;
    this->particles_25 = other.particles_25;
    this->particles_50 = other.particles_50;
    this->particles_100 = other.particles_100;

    this->_name = std::move(other._name);

    HomerSensorData::operator=(std::move(other));

    return *this;
}

SensorData::SensorData() noexcept:
        HomerSensorData{NAME},
        pm10_standard{std::numeric_limits<uint16_t>::max()},
        pm25_standard{std::numeric_limits<uint16_t>::max()},
        pm100_standard{std::numeric_limits<uint16_t>::max()},
        pm10_env{std::numeric_limits<uint16_t>::max()},
        pm25_env{std::numeric_limits<uint16_t>::max()},
        pm100_env{std::numeric_limits<uint16_t>::max()},
        particles_03{std::numeric_limits<uint16_t>::max()},
        particles_05{std::numeric_limits<uint16_t>::max()},
        particles_10{std::numeric_limits<uint16_t>::max()},
        particles_25{std::numeric_limits<uint16_t>::max()},
        particles_50{std::numeric_limits<uint16_t>::max()},
        particles_100{std::numeric_limits<uint16_t>::max()},
        _name{"PMS5003"}
{
}

SensorData::SensorData(SensorData&& other) noexcept:
        HomerSensorData(std::move(other)),
        pm10_standard{other.pm10_standard},
        pm25_standard{other.pm25_standard},
        pm100_standard{other.pm100_standard},
        pm10_env{other.pm10_env},
        pm25_env{other.pm25_env},
        pm100_env{other.pm100_env},
        particles_03{other.particles_03},
        particles_05{other.particles_05},
        particles_10{other.particles_10},
        particles_25{other.particles_25},
        particles_50{other.particles_50},
        particles_100{other.particles_100},
        _name{std::move(other._name)}
{
}

SensorData::SensorData(const SensorData& other) noexcept: // NOLINT(modernize-use-equals-default)
        HomerSensorData(other),
        pm10_standard{other.pm10_standard},
        pm25_standard{other.pm25_standard},
        pm100_standard{other.pm100_standard},
        pm10_env{other.pm10_env},
        pm25_env{other.pm25_env},
        pm100_env{other.pm100_env},
        particles_03{other.particles_03},
        particles_05{other.particles_05},
        particles_10{other.particles_10},
        particles_25{other.particles_25},
        particles_50{other.particles_50},
        particles_100{other.particles_100},
        _name{other._name}
{
}


void SensorData::do_dump(HomerSensorDumpMap& map) const noexcept
{
    map.insert({SENSOR_ATTR_PM10_STANDARD, std::to_string(this->pm10_standard)});
    map.insert({SENSOR_ATTR_PM25_STANDARD, std::to_string(this->pm25_standard)});
    map.insert({SENSOR_ATTR_PM100_STANDARD, std::to_string(this->pm100_standard)});

    map.insert({SENSOR_ATTR_PM10_ENV, std::to_string(this->pm10_env)});
    map.insert({SENSOR_ATTR_PM25_ENV, std::to_string(this->pm25_env)});
    map.insert({SENSOR_ATTR_PM100_ENV, std::to_string(this->pm100_env)});

    map.insert({SENSOR_ATTR_PARTICLES_03, std::to_string(this->particles_03)});
    map.insert({SENSOR_ATTR_PARTICLES_05, std::to_string(this->particles_05)});
    map.insert({SENSOR_ATTR_PARTICLES_10, std::to_string(this->particles_10)});
    map.insert({SENSOR_ATTR_PARTICLES_25, std::to_string(this->particles_25)});
    map.insert({SENSOR_ATTR_PARTICLES_50, std::to_string(this->particles_50)});
    map.insert({SENSOR_ATTR_PARTICLES_100, std::to_string(this->particles_100)});
}

void SensorData::do_dump(std::stringstream& ss) const noexcept
{
    ss << "ENV PM10:  " << this->pm10_env << std::endl;
    ss << "ENV PM25:  " << this->pm25_env << std::endl;
    ss << "ENV PM100: " << this->pm100_env << std::endl;
    ss << "PTC PM03:  " << this->particles_03 << std::endl;
    ss << "PTC PM05:  " << this->particles_05 << std::endl;
    ss << "PTC PM10:  " << this->particles_10 << std::endl;
    ss << "PTC PM25:  " << this->particles_25 << std::endl;
    ss << "PTC PM50:  " << this->particles_50 << std::endl;
    ss << "PTC PM100: " << this->particles_100 << std::endl;
}

void SensorData::invalidate() noexcept
{
    this->pm10_standard = std::numeric_limits<uint16_t>::max();
    this->pm25_standard = std::numeric_limits<uint16_t>::max();
    this->pm100_standard = std::numeric_limits<uint16_t>::max();
    this->pm10_env = std::numeric_limits<uint16_t>::max();
    this->pm25_env = std::numeric_limits<uint16_t>::max();
    this->pm100_env = std::numeric_limits<uint16_t>::max();
    this->particles_03 = std::numeric_limits<uint16_t>::max();
    this->particles_05 = std::numeric_limits<uint16_t>::max();
    this->particles_10 = std::numeric_limits<uint16_t>::max();
    this->particles_25 = std::numeric_limits<uint16_t>::max();
    this->particles_50 = std::numeric_limits<uint16_t>::max();
    this->particles_100 = std::numeric_limits<uint16_t>::max();
}

const char* SensorData::do_sensor_err_to_str(const uint64_t err) const noexcept
{
    switch (err) {
        case ERROR_UART_READ:
            return "uart_read";

        case ERROR_UART_BUFFER_FLUSH:
            return "uart_buffer_flush";

        case ERROR_READ_ATTEMPTS_EXHAUSTED:
            return "read_attempts_exhausted";

        case ERROR_BAD_MAGIC_VALUE:
            return "bad_magic_value";

        case ERROR_BAD_CHECKSUM:
            return "bad_checksum";

        default:
            return nullptr;
    }
}

}

namespace Pms5003 {

Sensor::Sensor(Sensor&& other) noexcept:
        HomerSensor(other._refresh_every, other._last_update),
        port{other.port},
        uart_buffer{other.uart_buffer},
        data{std::move(other.data)}
{
}

Sensor::Sensor(const uart_port_t port) :
        HomerSensor(MEASUREMENT_DELAY),
        port{port},
        data{}
{
    this->uart_buffer = new uint8_t[32];
    if (!this->uart_buffer) {
        ESP_LOGE(NAME, "could not allocate buffer[32]");
        throw std::runtime_error("could not allocate buffer[32]");
    }
}

Sensor& Sensor::operator=(Sensor&& other) noexcept
{
    if (this == &other)
        return *this;

    this->port = other.port;
    this->uart_buffer = other.uart_buffer;
    this->data = std::move(other.data);

    other.uart_buffer = nullptr;

    HomerSensor::operator=(std::move(other));

    return *this;
}

Sensor::~Sensor() noexcept
{
    delete[] this->uart_buffer;
    this->uart_buffer = nullptr;
}

void Sensor::refresh_data() noexcept
{
    auto err = this->read_frame();
    if (err.has_error()) {
        this->data._get_error().merge_from(err);
        return;
    }

    if (!this->checksum_check()) {
        this->data._get_error().add_sensor_err(ERROR_BAD_CHECKSUM);
        return;
    }

    this->put_buffer_into_data();
    this->data._get_error().mark_ok_has_data();
}

// Haha, I'm going to use goto all over the place.
// TODO This could probably use a ring buffer.
HwErr Sensor::read_frame() noexcept
{
    size_t loop = 0;

    start_over:
    ESP_LOGV(NAME, "trying to receive from sensor, try=%d", loop);

    for (size_t i = 0; i < 32; i++)
        this->uart_buffer[i] = 0;

    const auto err = uart_flush_input(this->port);
    if (err != ESP_OK) {
        ESP_LOGE(NAME, "failed to flush buffer");
        return {ERROR_UART_BUFFER_FLUSH, err};
    }

    size_t remaining = 32;
    while (this->uart_buffer[0] != MAGIC0 || this->uart_buffer[1] != MAGIC1 || remaining) {
        loop++;
        if (loop > MAX_RETRIES) {
            ESP_LOGE(NAME, "attempts exhausted while trying to read from sensor");
            return HwErr::make_sensor_err(ERROR_READ_ATTEMPTS_EXHAUSTED);
        }

        const auto from = 32 - remaining;
        const auto len = uart_read_bytes(this->port,
                                         this->uart_buffer + from,
                                         remaining,
                                         UART_READ_WAIT_TICKS);
        if (len < 0) {
            ESP_LOGE(NAME, "uart read error: %d", len);
            return {ERROR_UART_READ, len};
        }

        bool found_magic = false;
        size_t magic_index = 0;
        for (; magic_index < 31; magic_index++)
            if (this->uart_buffer[magic_index] == MAGIC0 &&
                this->uart_buffer[magic_index + 1] == MAGIC1) {
                found_magic = true;
                break;
            }
        if (!found_magic)
            goto start_over;

        if (magic_index > 0)
            for (size_t i = magic_index; i < 32; i++)
                this->uart_buffer[i - magic_index] = this->uart_buffer[i];

        remaining -= len;
    }

    if (this->uart_buffer[0] != MAGIC0 || this->uart_buffer[1] != MAGIC1) {
        ESP_LOGE(NAME, "wrong magic0, expecting=%02x-%02x got=%02x-%02x",
                 MAGIC0,
                 MAGIC1,
                 this->uart_buffer[0],
                 this->uart_buffer[1]);
        return HwErr::make_sensor_err(ERROR_BAD_MAGIC_VALUE);
    }

    return HwErr::make_ok();
}

void Sensor::put_buffer_into_data() noexcept
{
    this->data.pm10_standard = this->uart_buffer[4] * 256 + this->uart_buffer[5];
    this->data.pm25_standard = this->uart_buffer[6] * 256 + this->uart_buffer[7];
    this->data.pm100_standard = this->uart_buffer[8] * 256 + this->uart_buffer[9];

    this->data.pm10_env = this->uart_buffer[10] * 256 + this->uart_buffer[11];
    this->data.pm25_env = this->uart_buffer[12] * 256 + this->uart_buffer[13];
    this->data.pm100_env = this->uart_buffer[14] * 256 + this->uart_buffer[15];

    this->data.particles_03 = this->uart_buffer[16] * 256 + this->uart_buffer[17];
    this->data.particles_05 = this->uart_buffer[18] * 256 + this->uart_buffer[19];
    this->data.particles_10 = this->uart_buffer[20] * 256 + this->uart_buffer[21];
    this->data.particles_25 = this->uart_buffer[22] * 256 + this->uart_buffer[23];
    this->data.particles_50 = this->uart_buffer[24] * 256 + this->uart_buffer[25];
    this->data.particles_100 = this->uart_buffer[26] * 256 + this->uart_buffer[27];
}

bool Sensor::checksum_check() const noexcept
{
    const uint16_t checksum = this->uart_buffer[30] * 256 + this->uart_buffer[31];

    uint16_t sum = 0;
    for (size_t i = 0; i < 30; i++)
        sum += this->uart_buffer[i];

    if (sum != checksum)
        ESP_LOGE(NAME, "sum=%d, checksum=%d", sum, checksum);

    return sum == checksum;
}

SensorData& Sensor::get_raw_data() noexcept
{
    return this->data;
}

}

}
