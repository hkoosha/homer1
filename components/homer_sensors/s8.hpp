#pragma once

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <limits>

#include "driver/uart.h"

#include "homer_sensor.hpp"

using std::size_t;
using std::int16_t;
using std::int32_t;
using std::uint32_t;
using std::uint64_t;
using std::endl;


namespace homer1 {
namespace S8 {

const char* const NAME = "S8";

const uint64_t ERROR_UART_WRITE_BYTES = ERROR_NO_DATA_AVAILABLE << 1;
const uint64_t ERROR_UART_WRITE_BYTES_LEN_TOO_BIG = ERROR_UART_WRITE_BYTES << 1;
const uint64_t ERROR_UART_WRITE_BYTES_LEN_TOO_SHORT = ERROR_UART_WRITE_BYTES_LEN_TOO_BIG << 1;
const uint64_t ERROR_UART_READ_BYTES = ERROR_UART_WRITE_BYTES_LEN_TOO_SHORT << 1;
const uint64_t ERROR_UART_READ_BYTES_LEN_TOO_SHORT = ERROR_UART_READ_BYTES << 1;
const uint64_t ERROR_UART_READ_BYTES_LEN_TOO_BIG = ERROR_UART_READ_BYTES_LEN_TOO_SHORT << 1;
const uint64_t ERROR_UART_READ_BYTES_BAD_CRC = ERROR_UART_READ_BYTES_LEN_TOO_BIG << 1;
//---
const uint64_t ERROR_READ_REG = ERROR_UART_READ_BYTES_BAD_CRC << 1;
//---
const uint64_t ERROR_READ_CO2 = ERROR_READ_REG << 1;
const uint64_t ERROR_READ_ABC_DAYS = ERROR_READ_CO2 << 1;
const uint64_t ERROR_READ_SENSOR_ID = ERROR_READ_ABC_DAYS << 1;
const uint64_t ERROR_READ_SENSOR_FW = ERROR_READ_SENSOR_ID << 1;

const uint64_t MEASUREMENT_GAP_MILLIS = 2000;

const char* const SENSOR_ATTR_CO2 = "co2";
const char* const SENSOR_ATTR_ABC_DAYS = "abc_days";
const char* const SENSOR_ATTR_SENSOR_ID = "id";
const char* const SENSOR_ATTR_SENSOR_FW = "fw";

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


    int16_t co2;
    int16_t abc_days;
    int32_t sensor_id;
    int16_t sensor_fw;

protected:
    void do_dump(std::stringstream& ss) const noexcept override;

    void do_dump(HomerSensorDump& map) const noexcept override;

    const char* do_sensor_err_to_str(uint64_t err) const noexcept override;

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
    HwErr read_co2(int16_t& co2) noexcept;

    HwErr read_abc_days(int16_t& days) noexcept;

    HwErr read_sensor_id(int32_t& id) noexcept;

    HwErr read_sensor_fw(int16_t& fw) noexcept;


    HwErr read_reg(const unsigned char* data,
                   int16_t& reg) noexcept;

    HwErr write_8_bytes_and_read_7_bytes(const unsigned char* bytes) noexcept;

    HwErr write_8_bytes(const unsigned char* bytes) const noexcept;

    HwErr read_7_bytes() noexcept;

    void invalidate(const HwErr& err) noexcept;

    uart_port_t port;
    uint8_t* uart_buffer;
    int16_t abc_days;
    int32_t sensor_id;
    int16_t sensor_fw;
    SensorData data;
};

}
}
