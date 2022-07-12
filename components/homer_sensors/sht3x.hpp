#pragma once

#include <cstdint>
#include <sstream>
#include <limits>

#include "homer_util.hpp"
#include "homer_i2c.hpp"
#include "homer_sensor.hpp"

using std::uint8_t;
using std::uint16_t;
using std::uint64_t;
using std::endl;


namespace homer1::Sht3x {

const char* const NAME = "SHT3X";

const uint64_t ERROR_BAD_CRC_TEMPERATURE = ERROR_NO_DATA_AVAILABLE << 1;
const uint64_t ERROR_BAD_CRC_HUMIDITY = ERROR_BAD_CRC_TEMPERATURE << 1;

const uint8_t I2C_ADDR = 0x44;
const uint32_t I2C_DELAY = 10;
const uint64_t MEASUREMENT_GAP_MILLIS = 2000;

const char* const SENSOR_ATTR_TEMPERATURE = "temperature";
const char* const SENSOR_ATTR_HUMIDITY = "humidity";


const char* err_to_string(uint64_t err) noexcept;

class SensorData final : public HomerSensorData
{
public:
    SensorData& operator=(const SensorData& other) noexcept;

    SensorData& operator=(SensorData&& other) noexcept;

    SensorData(const SensorData& other) noexcept;

    SensorData(SensorData&& other) noexcept;

    SensorData() noexcept;

    ~SensorData() noexcept override = default;


    float temperature;
    float humidity;

protected:
    void invalidate() noexcept override;

    void do_dump(std::stringstream& ss) const noexcept override;

    void do_dump(HomerSensorDump& map) const noexcept override;

    [[nodiscard]] const char* do_sensor_err_to_str(uint64_t err) const noexcept override;
};

class Sensor final : public HomerSensor<SensorData>
{
public:
    Sensor& operator=(const Sensor& other) = delete;

    Sensor(const Sensor& other) = delete;


    Sensor& operator=(Sensor&& other) noexcept;

    Sensor(Sensor&& other) noexcept;

    explicit Sensor(i2c::Device i2c) noexcept;

    ~Sensor() noexcept override = default;


protected:
    void refresh_data() noexcept override;

    SensorData& get_raw_data() noexcept override;

private:
    i2c::Device i2c;
    SensorData data;
};

}
