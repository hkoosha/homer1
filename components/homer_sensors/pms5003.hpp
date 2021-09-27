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

const uint64_t ERROR_UART_READ = ERROR_NO_DATA_AVAILABLE << 1;
const uint64_t ERROR_UART_BUFFER_FLUSH = ERROR_UART_READ << 1;
const uint64_t ERROR_READ_ATTEMPTS_EXHAUSTED = ERROR_UART_BUFFER_FLUSH << 1;
const uint64_t ERROR_BAD_MAGIC_VALUE = ERROR_READ_ATTEMPTS_EXHAUSTED << 1;
const uint64_t ERROR_BAD_CHECKSUM = ERROR_BAD_MAGIC_VALUE << 1;

const uint64_t MEASUREMENT_DELAY = 1000;


class SensorData final : public HomerSensorData
{
public:
    SensorData& operator=(const SensorData& other) noexcept;

    SensorData& operator=(SensorData&& other) noexcept;

    SensorData(const SensorData& other) noexcept = default;

    SensorData(SensorData&& other) noexcept;

    SensorData() noexcept;

    ~SensorData() noexcept override = default;


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

protected:
    void do_dump(std::stringstream& ss) const noexcept override
    {
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

    void do_dump(HomerSensorDump& map) const noexcept override;

    void invalidate() noexcept override;
};

class Sensor final : public HomerSensor<SensorData>
{
public:
    Sensor& operator=(const Sensor& other) = delete;

    Sensor(const Sensor& other) = delete;


    Sensor& operator=(Sensor&& other) noexcept;

    Sensor(Sensor&& other) noexcept;

    explicit Sensor(uart_port_t port);

    ~Sensor() noexcept override;


protected:
    void refresh_data() noexcept override;

    SensorData& get_raw_data() noexcept override;

private:
    HwErr read_frame() noexcept;

    void put_buffer_into_data() noexcept;

    bool checksum_check() const noexcept;

    uart_port_t port;
    uint8_t* uart_buffer;
    SensorData data;
};

}
}
