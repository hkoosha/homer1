#include <cstdint>
#include <sstream>
#include <limits>

#include "esp_log.h"
#include "driver/uart.h"

#include "homer_util.hpp"
#include "s8.hpp"

using std::size_t;
using std::uint8_t;
using std::uint16_t;
using std::uint64_t;
using std::int32_t;
using std::endl;


namespace homer1 {

namespace {

const unsigned char ID_HI[] = {0xFE, 0x04, 0x00, 0x1D, 0x00, 0x01, 0xB5, 0xC3};
const unsigned char ID_LO[] = {0xFE, 0x04, 0x00, 0x1E, 0x00, 0x01, 0x45, 0xC3};
const unsigned char FW_REQ[] = {0xFE, 0x04, 0x00, 0x1C, 0x00, 0x01, 0xE4, 0x03};
const unsigned char ABC_REQ[] = {0xFE, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xA1, 0XC3};
const unsigned char CO2_REQ[] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};

const auto UART_READ_WAIT = 500;
const auto UART_READ_WAIT_TICKS = UART_READ_WAIT / portTICK_PERIOD_MS;

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

const char* err_to_string(const uint64_t err) noexcept
{
    switch (err) {
        case ERROR_UART_WRITE_BYTES:
            return "uart_write_bytes";

        case ERROR_UART_WRITE_BYTES_LEN_TOO_BIG:
            return "uart_write_bytes_len_too_big";

        case ERROR_UART_WRITE_BYTES_LEN_TOO_SHORT:
            return "uart_write_bytes_len_too_short";

        case ERROR_UART_READ_BYTES:
            return "uart_read_bytes";

        case ERROR_UART_READ_BYTES_LEN_TOO_SHORT:
            return "uart_read_bytes_len_too_short";

        case ERROR_UART_READ_BYTES_LEN_TOO_BIG:
            return "uart_read_bytes_len_too_big";

        case ERROR_UART_READ_BYTES_BAD_CRC:
            return "uart_read_bytes_bad_crc";

        case ERROR_READ_REG:
            return "read_reg";

        case ERROR_READ_CO2:
            return "read_co2";

        case ERROR_READ_ABC_DAYS:
            return "read_abc_days";

        case ERROR_READ_SENSOR_ID:
            return "read_sensor_id";

        case ERROR_READ_SENSOR_FW:
            return "read_sensor_fw";

        default:
            return nullptr;
    }
}

}

namespace S8 {

void SensorData::do_dump(std::stringstream& ss) const noexcept
{
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

void SensorData::do_dump(HomerSensorDump& map) const noexcept
{
    insert(map, SENSOR_ATTR_CO2, this->co2);
    insert(map, SENSOR_ATTR_ABC_DAYS, this->abc_days);
    insert(map, SENSOR_ATTR_SENSOR_ID, this->sensor_id);
    insert(map, SENSOR_ATTR_SENSOR_FW, this->sensor_fw);
}

void SensorData::invalidate() noexcept
{
    this->co2 = std::numeric_limits<int16_t>::max();
    this->abc_days = std::numeric_limits<int16_t>::max();
    this->sensor_id = 0;
    this->sensor_fw = 0;
}

const char* SensorData::do_sensor_err_to_str(const uint64_t err) const noexcept
{
    return err_to_string(err);
}


SensorData& SensorData::operator=(const SensorData& other) noexcept
{
    if (this == &other)
        return *this;

    HomerSensorData::operator=(other);

    this->co2 = other.co2;
    this->abc_days = other.abc_days;
    this->sensor_fw = other.sensor_fw;
    this->sensor_id = other.sensor_id;

    return *this;
}

SensorData& SensorData::operator=(SensorData&& other) noexcept
{
    if (this == &other)
        return *this;

    this->co2 = other.co2;
    this->abc_days = other.abc_days;
    this->sensor_fw = other.sensor_fw;
    this->sensor_id = other.sensor_id;

    HomerSensorData::operator=(std::move(other));

    return *this;
}

SensorData::SensorData(const SensorData& other) noexcept:
        HomerSensorData(other)
{
    this->co2 = other.co2;
    this->abc_days = other.abc_days;
    this->sensor_fw = other.sensor_fw;
    this->sensor_id = other.sensor_id;
}

SensorData::SensorData(SensorData&& other) noexcept:
        HomerSensorData(std::move(other.error),
                        other._time_to_read,
                        other._read_start,
                        other._name),
        co2{other.co2},
        abc_days{other.abc_days},
        sensor_id{other.sensor_id},
        sensor_fw{other.sensor_fw}
{
}

SensorData::SensorData() noexcept:
        HomerSensorData(NAME),
        co2{std::numeric_limits<int16_t>::max()},
        abc_days{std::numeric_limits<int16_t>::max()},
        sensor_id{0},
        sensor_fw{0}
{
}

}

namespace S8 {

Sensor::Sensor(Sensor&& other) noexcept:
        HomerSensor<SensorData>(other._refresh_every, other._last_update),
        port{other.port},
        uart_buffer{other.uart_buffer},
        abc_days{other.abc_days},
        sensor_id{other.sensor_id},
        sensor_fw{other.sensor_fw},
        data{std::move(other.data)}
{
}

Sensor::Sensor(const uart_port_t port) :
        HomerSensor<SensorData>(MEASUREMENT_GAP_MILLIS),
        port{port},
        abc_days{std::numeric_limits<int16_t>::max()},
        sensor_id{0},
        sensor_fw{0},
        data{}
{
    this->uart_buffer = new uint8_t[8];
    if (!this->uart_buffer) {
        ESP_LOGE(NAME, "could not allocate buffer[8]");
        throw std::runtime_error("could not allocate buffer[8]");
    }
}

Sensor& Sensor::operator=(Sensor&& other) noexcept
{
    if (this == &other)
        return *this;

    this->port = other.port;
    this->uart_buffer = other.uart_buffer;
    this->abc_days = other.abc_days;
    this->sensor_id = other.sensor_id;
    this->sensor_fw = other.sensor_fw;
    this->data = std::move(other.data);

    other.uart_buffer = nullptr;

    HomerSensor<SensorData>::operator=(std::move(other));

    return *this;
}

Sensor::~Sensor() noexcept
{
    delete[] this->uart_buffer;
    this->uart_buffer = nullptr;
}


void Sensor::refresh_data() noexcept
{
    auto err = this->read_co2(data.co2);
    if (err.has_error()) {
        this->invalidate(err);
        return;
    }

    if (this->abc_days > 0) {
        this->data.abc_days = this->abc_days;
    }
    else {
        err = this->read_abc_days(data.abc_days);
        if (err.has_error()) {
            this->invalidate(err);
            return;
        }
    }

    if (this->sensor_id > 0) {
        this->data.sensor_id = this->sensor_id;
    }
    else {
        err = this->read_sensor_id(data.sensor_id);
        if (err.has_error()) {
            this->invalidate(err);
            return;
        }
    }

    if (this->sensor_fw > 0) {
        this->data.sensor_fw = this->sensor_fw;
    }
    else {
        err = this->read_sensor_fw(data.sensor_fw);
        if (err.has_error()) {
            this->invalidate(err);
            return;
        }
    }

    this->data._get_error().mark_ok_has_data();
}

void Sensor::invalidate(const HwErr& err) noexcept
{
    this->abc_days = std::numeric_limits<int16_t>::max();
    this->sensor_fw = 0;
    this->sensor_id = 0;
    this->data.abc_days = std::numeric_limits<int16_t>::max();
    this->data.sensor_id = 0;
    this->data.sensor_fw = 0;
    this->data._get_error().merge_from(err);
}


HwErr Sensor::read_co2(int16_t& co2) noexcept
{
    auto err = this->read_reg(CO2_REQ, co2);

    if (err.has_error()) {
        ESP_LOGE(NAME, "error reading co2");
        err.add_sensor_err(ERROR_READ_CO2);
        return err;
    }

    return err;
}

HwErr Sensor::read_abc_days(int16_t& days) noexcept
{
    auto err = this->read_reg(ABC_REQ, days);

    if (err.has_error()) {
        ESP_LOGE(NAME, "error reading abc");
        err.add_sensor_err(ERROR_READ_ABC_DAYS);
        return err;
    }

    return err;
}

HwErr Sensor::read_sensor_id(int32_t& id) noexcept
{
    int16_t id_hi;
    auto err = this->read_reg(ID_HI, id_hi);
    if (err.has_error()) {
        ESP_LOGE(NAME, "error reading id_hi");
        err.add_sensor_err(ERROR_READ_SENSOR_ID);
        return err;
    }

    int16_t id_lo;
    err = this->read_reg(ID_LO, id_lo);
    if (err.has_error()) {
        ESP_LOGE(NAME, "error reading id_lo");
        err.add_sensor_err(ERROR_READ_SENSOR_ID);
        return err;
    }

    const auto hi = static_cast<int32_t>(id_hi);
    const auto lo = static_cast<int32_t>(id_lo);
    id = hi * 256 * 256 + lo;
    return err;
}

HwErr Sensor::read_sensor_fw(int16_t& fw) noexcept
{
    auto err = this->read_reg(FW_REQ, fw);

    if (err.has_error()) {
        ESP_LOGE(NAME, "error reading fw");
        err.add_sensor_err(ERROR_READ_SENSOR_FW);
        return err;
    }

    return err;
}

HwErr Sensor::read_reg(const unsigned char* bytes,
                       int16_t& reg) noexcept
{
    auto err = this->write_8_bytes_and_read_7_bytes(bytes);
    if (err.has_error()) {
        reg = -1;
        err.add_sensor_err(ERROR_READ_REG);
        return err;
    }

    const auto hi = static_cast<uint16_t>(this->uart_buffer[3]);
    const auto lo = static_cast<uint16_t>(this->uart_buffer[4]);
    const auto value = hi * 256 + lo;
    reg = static_cast<int16_t>(value);

    return err;
}


HwErr Sensor::write_8_bytes_and_read_7_bytes(const unsigned char* bytes) noexcept
{
    auto err = this->write_8_bytes(bytes);
    if (err.has_error())
        return err;

    err = this->read_7_bytes();
    return err;
}

HwErr Sensor::write_8_bytes(const unsigned char* bytes) const noexcept
{
    const auto len = uart_write_bytes(this->port, bytes, 8);

    if (len < 0) {
        ESP_LOGE(NAME, "uart write error, code=%d", len);
        return {ERROR_UART_WRITE_BYTES, len};
    }
    else if (len > 8) {
        ESP_LOGE(NAME, "uart write assertion error, too much data written, expected=8, written=%d", len);
        return {ERROR_UART_WRITE_BYTES_LEN_TOO_BIG, ESP_OK};
    }
    else if (len != 8) {
        ESP_LOGE(NAME, "uart write error, could not write all data, expected to write=8, written=%d", len);
        return {ERROR_UART_WRITE_BYTES_LEN_TOO_SHORT, ESP_OK};
    }
    else {
        ESP_LOGV(NAME, "wrote 8 bytes to uart");
        return HwErr::make_ok();
    }
}

HwErr Sensor::read_7_bytes() noexcept
{
    for (size_t i = 0; i < 8; i++)
        this->uart_buffer[i] = 0;

    const auto len = uart_read_bytes(this->port, this->uart_buffer, 8, UART_READ_WAIT_TICKS);

    if (len < 0) {
        ESP_LOGE(NAME, "uart read error, code=%d", len);
        return {ERROR_UART_READ_BYTES, len};
    }
    else if (len > 7) {
        ESP_LOGE(NAME, "too much data in uart, by now memory is corrupt.");
        return {ERROR_UART_READ_BYTES_LEN_TOO_BIG, ESP_OK};
    }
    else if (len < 7) {
        ESP_LOGE(NAME, "uart read error, could not read all data, expected to read=7, actual read=%d", len);
        return {ERROR_UART_READ_BYTES_LEN_TOO_SHORT, ESP_OK};
    }

    const auto crc_hi = static_cast<uint16_t>(this->uart_buffer[6]) * 256;
    const auto crc_lo = static_cast<uint16_t>(this->uart_buffer[5]);
    const auto read_crc = crc_hi + crc_lo;
    const auto calc_crc = modbus_crc(this->uart_buffer, 5);

    if (read_crc != calc_crc) {
        ESP_LOGE(NAME, "uart read error, crc mismatch.");
        return {ERROR_UART_READ_BYTES_BAD_CRC, ESP_OK};
    }

    return HwErr::make_ok();
}

SensorData& Sensor::get_raw_data() noexcept
{
    return this->data;
}

}

}
