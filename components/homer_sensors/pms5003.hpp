#pragma once

#include <cstdint>
#include <sstream>
#include <limits>

#include "esp_log.h"

#include "homer_util.hpp"

using std::uint16_t;
using std::uint32_t;
using std::uint64_t;


namespace homer1::Pms5003 {

const char* const NAME = "PMS5003";

const uint64_t ERROR_UART_READ = ERROR_NO_DATA_AVAILABLE << 1;
const uint64_t ERROR_UART_BUFFER_FLUSH = ERROR_UART_READ << 1;
const uint64_t ERROR_READ_ATTEMPTS_EXHAUSTED = ERROR_UART_BUFFER_FLUSH << 1;
const uint64_t ERROR_BAD_MAGIC_VALUE = ERROR_READ_ATTEMPTS_EXHAUSTED << 1;
const uint64_t ERROR_BAD_CHECKSUM = ERROR_BAD_MAGIC_VALUE << 1;

const uint64_t MEASUREMENT_DELAY = 1000;

const char* const SENSOR_ATTR_PM10_STANDARD = "pm10_std";
const char* const SENSOR_ATTR_PM25_STANDARD = "pm25_std";
const char* const SENSOR_ATTR_PM100_STANDARD = "pm100_std";
const char* const SENSOR_ATTR_PM10_ENV = "pm10_env";
const char* const SENSOR_ATTR_PM25_ENV = "pm25_env";
const char* const SENSOR_ATTR_PM100_ENV = "pm100_env";
const char* const SENSOR_ATTR_PARTICLES_03 = "particle_03";
const char* const SENSOR_ATTR_PARTICLES_05 = "particle_05";
const char* const SENSOR_ATTR_PARTICLES_10 = "particle_10";
const char* const SENSOR_ATTR_PARTICLES_25 = "particle_25";
const char* const SENSOR_ATTR_PARTICLES_50 = "particle_50";
const char* const SENSOR_ATTR_PARTICLES_100 = "particle_100";


class SensorData final : public HomerSensorData
{
public:
    SensorData& operator=(const SensorData& other) noexcept;

    SensorData& operator=(SensorData&& other) noexcept;

    SensorData(const SensorData& other) noexcept;

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
    void do_dump(std::stringstream& ss) const noexcept override;

    void do_dump(HomerSensorDumpMap& map) const noexcept override;

    [[nodiscard]] const char* do_sensor_err_to_str(uint64_t err) const noexcept override;

    void invalidate() noexcept override;

private:
    std::string _name;
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

    [[nodiscard]] bool checksum_check() const noexcept;

    uart_port_t port;
    uint8_t* uart_buffer;
    SensorData data;
};

}
