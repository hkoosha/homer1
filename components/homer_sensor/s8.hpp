#pragma once

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <limits>

#include "driver/uart.h"

using std::size_t;
using std::int16_t;
using std::int32_t;
using std::uint32_t;
using std::uint64_t;
using std::endl;


namespace homer1 {
namespace S8 {

const uint64_t ERROR_NONE = 0;
//---
const uint64_t ERROR_DATA_NOT_AVAILABLE = 1;
//---
const uint64_t ERROR_UART_WRITE_BYTES = ERROR_DATA_NOT_AVAILABLE << 1;
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


struct SensorData final : public Dumper
{
    uint64_t error;
    uint64_t time_to_read;

    int32_t sensor_id;
    int16_t co2;
    int16_t abc_days;
    int16_t sensor_fw;

    explicit SensorData() noexcept:
            error{ERROR_DATA_NOT_AVAILABLE},
            time_to_read{std::numeric_limits<uint64_t>::max()},
            sensor_id{0},
            co2{std::numeric_limits<int16_t>::max()},
            abc_days{std::numeric_limits<int16_t>::max()},
            sensor_fw{0}
    {
    }

    void dump(std::stringstream& ss) const noexcept override
    {
        ss << "ERR: " << uint64_to_bin(this->error) << endl;
        ss << "TTR: " << this->time_to_read << endl;

        ss << "CO2: " << this->co2 << endl;

        // ss << "ABC: " << this->abc_days
        //    << " (" << (this->abc_days / 24.) << " days)"
        //    << endl;
        //
        // ss << "SID: 0x" << std::uppercase << std::hex
        //    << this->sensor_id
        //    << std::dec << std::nouppercase
        //    << endl;
        //
        // ss << "SFW: 0x" << std::uppercase << std::hex
        //    << this->sensor_fw
        //    << std::dec << std::nouppercase
        //    << " (" << this->sensor_fw << ")"
        //    << endl;
    }

    bool has_data() const noexcept override
    {
        return (this->error & ERROR_DATA_NOT_AVAILABLE) == ERROR_NONE;
    }
};

class Sensor final
{
public:

    Sensor& operator=(const Sensor& other) = delete;

    Sensor& operator=(Sensor&& other) = delete;


    Sensor(const Sensor& other) noexcept = default;

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

    uint64_t read_co2(int16_t& co2) noexcept;

    uint64_t read_abc_days(int16_t& days) noexcept;

    uint64_t read_sensor_id(int32_t& id) noexcept;

    uint64_t read_sensor_fw(int16_t& fw) noexcept;


    uint64_t read_reg(const unsigned char* data,
                      int16_t& reg) noexcept;

    uint64_t write_8_bytes_and_read_7_bytes(const unsigned char* bytes) noexcept;

    uint64_t write_8_bytes(const unsigned char* bytes) const noexcept;

    uint64_t read_7_bytes() noexcept;


    const uart_port_t port;
    uint8_t* uart_buffer;
    int32_t sensor_id{0};
    int16_t abc_days{0};
    int16_t sensor_fw{0};
};

}
}
