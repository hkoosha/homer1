#pragma once

#include <cstdint>
#include <sstream>

#include "esp_log.h"

#include "util.hpp"

using std::uint16_t;
using std::uint32_t;
using std::uint64_t;


namespace homer1 {
namespace Pms5003 {

const uint64_t ERROR_NONE = 0;
//---
const uint64_t ERROR_UART_READ = 1;
const uint64_t ERROR_UART_BUFFER_FLUSH = ERROR_UART_READ << 1;
const uint64_t ERROR_READ_ATTEMPTS_EXHAUSTED = ERROR_UART_BUFFER_FLUSH << 1;
const uint64_t ERROR_BAD_MAGIC_VALUE = ERROR_READ_ATTEMPTS_EXHAUSTED << 1;
const uint64_t ERROR_BAD_CHECKSUM = ERROR_BAD_MAGIC_VALUE << 1;


struct SensorData final
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


SensorData copy(const SensorData& data) noexcept;

void dump(const SensorData& data,
          std::stringstream& ss,
          bool includeStd = false) noexcept;

}
}
