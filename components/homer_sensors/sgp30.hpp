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


namespace homer1::Sgp30 {

const char* const NAME = "SGP30";

const uint8_t I2C_ADDR = 0x58;
const uint32_t I2C_DELAY = 20;
const uint64_t MEASUREMENT_GAP_MILLIS = 2000;

const char* const SENSOR_ATTR_TVOC = "tvoc";
const char* const SENSOR_ATTR_ECO2 = "co2";
const char* const SENSOR_ATTR_RAW_H2 = "H2";
const char* const SENSOR_ATTR_RAW_ETHANOL = "ethanol";

const uint64_t ERROR_I2C_CRC_MISMATCH = ERROR_NO_DATA_AVAILABLE << 1;
const uint64_t ERROR_BAD_VALUE_GIVEN = ERROR_I2C_CRC_MISMATCH << 1;
const uint64_t ERROR_UNSUPPORTED_SENSOR = ERROR_BAD_VALUE_GIVEN << 1;

class SensorData final : public HomerSensorData
{
public:
    SensorData& operator=(const SensorData& other) noexcept;

    SensorData& operator=(SensorData&& other) noexcept;

    SensorData(const SensorData& other) noexcept;

    SensorData(SensorData&& other) noexcept;

    SensorData() noexcept;

    ~SensorData() noexcept override = default;


    void serialize(Serializer& sz) const noexcept override;


    uint16_t tvoc;
    uint16_t eco2;
    uint16_t raw_h2;
    uint16_t raw_ethanol;

    uint16_t serial_number0;
    uint16_t serial_number1;
    uint16_t serial_number2;

protected:
    void invalidate() noexcept override;

    void do_dump(std::stringstream& ss) const noexcept override;

    void do_dump(HomerSensorDumpMap& map) const noexcept override;

    [[nodiscard]] const char* do_sensor_err_to_str(uint64_t err) const noexcept override;
};

class Sensor final : public HomerSensor<SensorData>
{
public:
    Sensor& operator=(const Sensor& other) = delete;

    Sensor(const Sensor& other) = delete;


    Sensor& operator=(Sensor&& other) noexcept;

    Sensor(Sensor&& other) noexcept;

    explicit Sensor(i2c::Device* i2c);

    ~Sensor() noexcept override = default;


    [[nodiscard]] HwErr init() noexcept;

    [[nodiscard]] bool is_initialized() const noexcept;

    [[nodiscard]] HwErr set_absolute_humidity(uint32_t absolute_humidity);

    [[nodiscard]] HwErr set_iaq_baseline(uint16_t eco2_base, uint16_t tvoc_base);


protected:
    void refresh_data() noexcept override;

    [[nodiscard]] SensorData& get_raw_data() noexcept override;

private:
    [[nodiscard]] HwErr measure_raw();

    [[nodiscard]] HwErr measure();

    [[nodiscard]] HwErr send_and_read(uint16_t delay_millis,
                                      const uint8_t* command,
                                      size_t cmd_len,
                                      uint16_t* read,
                                      size_t read_len) const noexcept;

    i2c::Device* i2c;
    SensorData data;
    bool initialized;
};

}
