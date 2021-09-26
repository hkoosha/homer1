#pragma once

#include <cstdint>
#include <sstream>
#include <limits>

#include "esp_log.h"

#include "homer_util.hpp"

using std::uint16_t;
using std::uint32_t;
using std::uint64_t;
using std::endl;


namespace homer1 {
namespace Pms5003 {

const uint64_t ERROR_NONE = 0;
//---
const uint64_t ERROR_NO_DATA_AVAILABLE = 1;
const uint64_t ERROR_UART_READ = ERROR_NO_DATA_AVAILABLE << 1;
const uint64_t ERROR_UART_BUFFER_FLUSH = ERROR_UART_READ << 1;
const uint64_t ERROR_READ_ATTEMPTS_EXHAUSTED = ERROR_UART_BUFFER_FLUSH << 1;
const uint64_t ERROR_BAD_MAGIC_VALUE = ERROR_READ_ATTEMPTS_EXHAUSTED << 1;
const uint64_t ERROR_BAD_CHECKSUM = ERROR_BAD_MAGIC_VALUE << 1;


struct SensorData final : public Dumper
{
    uint64_t error;
    uint64_t time_to_read;

    uint16_t pm10_standard;
    uint16_t pm25_standard;
    uint16_t pm100_standard;

    uint16_t pm10_env;
    uint16_t pm25_env;
    uint16_t pm100_env;

    uint16_t particles_03;
    uint16_t particles_05;
    uint16_t particles_10;
    uint16_t particles_25;
    uint16_t particles_50;
    uint16_t particles_100;


    explicit SensorData() noexcept:
            error{ERROR_NO_DATA_AVAILABLE},
            time_to_read{std::numeric_limits<uint64_t>::max()},
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
            particles_100{std::numeric_limits<uint16_t>::max()}
    {
    }


    void dump(std::stringstream& ss) const noexcept override
    {
        ss << "ERR: " << uint64_to_bin(this->error) << endl;
        ss << "TTR: " << this->time_to_read << endl;

        ss << "ENV PM10:  " << this->pm10_env << endl;
        ss << "ENV PM25:  " << this->pm25_env << endl;
        ss << "ENV PM100: " << this->pm100_env << endl;
        ss << "PTC PM03:  " << this->particles_03 << endl;
        ss << "PTC PM05:  " << this->particles_05 << endl;
        ss << "PTC PM10:  " << this->particles_10 << endl;
        ss << "PTC PM25:  " << this->particles_25 << endl;
        ss << "PTC PM50:  " << this->particles_50 << endl;
        ss << "PTC PM100: " << this->particles_100 << endl;
    }

    bool has_data() const noexcept override
    {
        return (this->error & ERROR_NO_DATA_AVAILABLE) == ERROR_NONE;
    }
};

class Sensor final
{
public:

    Sensor(const Sensor& other) = delete;

    Sensor& operator=(const Sensor& other) = delete;

    Sensor& operator=(Sensor&& other) = delete;


    Sensor(Sensor&& other) noexcept:
            port{other.port},
            uart_buffer{other.uart_buffer}
    {
        other.uart_buffer = nullptr;
    }

    explicit Sensor(uart_port_t port);

    ~Sensor() noexcept;


    SensorData read_data() noexcept;

private:

    uint64_t read_frame() noexcept;

    void put_buffer_into_data(SensorData& data) const noexcept;

    bool checksum_check() const noexcept;

    const uart_port_t port;
    uint8_t* uart_buffer;
};

}
}
