#include <cstddef>
#include <cstdint>
#include <sstream>

#include "esp_log.h"
#include "driver/uart.h"

#include "util.hpp"
#include "s8.hpp"

using std::size_t;
using std::uint8_t;
using std::uint16_t;
using std::uint64_t;
using std::int32_t;
using std::endl;


namespace homer1 {

namespace {
const char* TAG = "Sensor";

const unsigned char ID_HI[] = {0xFE, 0x04, 0x00, 0x1D, 0x00, 0x01, 0xB5, 0xC3};
const unsigned char ID_LO[] = {0xFE, 0x04, 0x00, 0x1E, 0x00, 0x01, 0x45, 0xC3};
const unsigned char FW_REQ[] = {0xFE, 0x04, 0x00, 0x1C, 0x00, 0x01, 0xE4, 0x03};
const unsigned char ABC_REQ[] = {0xFE, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xA1, 0XC3};
const unsigned char CO2_REQ[] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};

const auto UART_READ_WAIT = 500;
const auto UART_READ_WAIT_TICKS = UART_READ_WAIT / portTICK_RATE_MS;


// https://github.com/SFeli/ESP32_S8/blob/master/ESP32_S8_01.ino
uint16_t modbus_crc(const uint8_t* buf,
                    const size_t len) noexcept
{
    uint16_t crc = 0xFFFF;

    for (size_t pos = 0; pos < len; pos++) {
        crc ^= static_cast<uint16_t>(buf[pos]);         // XOR byte into least sig. byte of crc
        for (uint8_t i = 8; i > 0; i--) {   // Loop over each bit
            if ((crc & 0x0001) != 0) {        // If the LSB is set
                crc >>= 1;                      // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else {                            // else LSB is not set
                crc >>= 1;                      // Just shift right
            }
        }
    }

    // Note, this number has low and high bytes swapped,
    // so use it accordingly (or swap bytes)
    return crc;
}
}

namespace S8 {

Sensor::Sensor(const uart_port_t port) :
        port{port}
{
    this->uart_buffer = new uint8_t[8];
    if (!this->uart_buffer) {
        ESP_LOGE(TAG, "could not allocate buffer");
        throw std::runtime_error("could not allocate buffer");
    }
}

Sensor::~Sensor() noexcept
{
    delete[] this->uart_buffer;
}


// =============================================================================

uint64_t Sensor::read_co2(int16_t& co2) noexcept
{
    const auto err = this->read_reg(CO2_REQ, co2);

    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "error reading co2");
        return err | ERROR_READ_CO2;
    }
    return ERROR_NONE;
}

uint64_t Sensor::read_abc_days(int16_t& days) noexcept
{
    const auto err = this->read_reg(ABC_REQ, days);

    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "error reading abc");
        return err | ERROR_READ_ABC_DAYS;
    }
    return ERROR_NONE;
}

uint64_t Sensor::read_sensor_id(int32_t& id) noexcept
{
    uint64_t err;

    int16_t id_hi;
    err = this->read_reg(ID_HI, id_hi);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "error reading id_hi");
        return err | ERROR_READ_SENSOR_ID;
    }

    int16_t id_lo;
    err = this->read_reg(ID_LO, id_lo);
    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "error reading id_lo");
        return err | ERROR_READ_SENSOR_ID;
    }

    const auto hi = static_cast<int32_t>(id_hi);
    const auto lo = static_cast<int32_t>(id_lo);
    id = hi * 256 * 256 + lo;
    return ERROR_NONE;
}

uint64_t Sensor::read_sensor_fw(int16_t& fw) noexcept
{
    const auto err = this->read_reg(FW_REQ, fw);

    if (err != ERROR_NONE) {
        ESP_LOGE(TAG, "error reading fw");
        return err | ERROR_READ_SENSOR_FW;
    }
    return ERROR_NONE;
}

SensorData Sensor::read_data() noexcept
{
    SensorData data{};
    data.error = ERROR_NONE;
    data.time_to_read = 0;

    const auto then = now_millis();

    uint64_t err;

    err = this->read_co2(data.co2);
    data.error |= err;
    if (err != ERROR_NONE) {
        this->abc_days = 0;
        this->sensor_fw = 0;
        this->sensor_id = 0;
    }

    if (this->abc_days > 0) {
        data.abc_days = this->abc_days;
    }
    else {
        err = this->read_abc_days(data.abc_days);
        data.error |= err;
        if (err == ERROR_NONE)
            this->abc_days = data.abc_days;
    }

    if (this->sensor_id > 0) {
        data.sensor_id = this->sensor_id;
    }
    else {
        err = this->read_sensor_id(data.sensor_id);
        data.error |= err;
        if (err == ERROR_NONE)
            this->sensor_id = data.sensor_id;
    }

    if (this->sensor_fw > 0) {
        data.sensor_fw = this->sensor_fw;
    }
    else {
        err = this->read_sensor_fw(data.sensor_fw);
        data.error |= err;
        if (err == ERROR_NONE)
            this->sensor_fw = data.sensor_fw;
    }

    data.time_to_read = now_millis() - then;
    return data;
}

uint64_t Sensor::read_reg(const unsigned char* data,
                          int16_t& reg) noexcept
{
    const uint64_t err = this->write_8_bytes_and_read_7_bytes(data);
    if (err != ERROR_NONE) {
        reg = -1;
        return err | ERROR_READ_REG;
    }

    const auto hi = static_cast<uint16_t>(this->uart_buffer[3]);
    const auto lo = static_cast<uint16_t>(this->uart_buffer[4]);
    const auto value = hi * 256 + lo;
    reg = static_cast<int16_t>(value);

    return ERROR_NONE;
}


uint64_t Sensor::write_8_bytes_and_read_7_bytes(const unsigned char* data) noexcept
{
    uint64_t err;

    err = this->write_8_bytes(data);
    if (err != ERROR_NONE)
        return err;

    err = this->read_7_bytes();
    if (err != ERROR_NONE)
        return err;

    return ERROR_NONE;
}

uint64_t Sensor::write_8_bytes(const unsigned char* data) const noexcept
{
    const auto len = uart_write_bytes(this->port, data, 8);

    if (len < 0) {
        ESP_LOGE(TAG, "uart write error, code=%d", len);
        return ERROR_UART_WRITE_BYTES;
    }
    else if (len > 8) {
        ESP_LOGE(TAG, "uart write assertion error, too much data written, expected=8, written=%d", len);
        return ERROR_UART_WRITE_BYTES_LEN_TOO_BIG;
    }
    else if (len != 8) {
        ESP_LOGE(TAG, "uart write error, could not write all data, expected to write=8, written=%d", len);
        return ERROR_UART_WRITE_BYTES_LEN_TOO_SHORT;
    }
    else {
        ESP_LOGV(TAG, "wrote 8 bytes to uart");
        return ERROR_NONE;
    }
}

uint64_t Sensor::read_7_bytes() noexcept
{
    for (size_t i = 0; i < 8; i++)
        this->uart_buffer[i] = 0;

    const auto len = uart_read_bytes(this->port, this->uart_buffer, 8, UART_READ_WAIT_TICKS);

    if (len < 0) {
        ESP_LOGE(TAG, "uart read error, code=%d", len);
        return ERROR_UART_READ_BYTES;
    }
    else if (len > 7) {
        ESP_LOGE(TAG, "too much data in uart, by now memory is corrupt.");
        return ERROR_UART_READ_BYTES_LEN_TOO_BIG;
    }
    else if (len < 7) {
        ESP_LOGE(TAG, "uart read error, could not read all data, expected to read=7, actual read=%d", len);
        return ERROR_UART_READ_BYTES_LEN_TOO_SHORT;
    }

    const auto crc_hi = static_cast<uint16_t>(this->uart_buffer[6]) * 256;
    const auto crc_lo = static_cast<uint16_t>(this->uart_buffer[5]);
    const auto read_crc = crc_hi + crc_lo;
    const auto calc_crc = modbus_crc(this->uart_buffer, 5);

    if (read_crc != calc_crc) {
        ESP_LOGE(TAG, "uart read error, crc mismatch.");
        return ERROR_UART_READ_BYTES_BAD_CRC;
    }

    return ERROR_NONE;
}

}
}
