#include "esp_log.h"

#include "homer_util.hpp"
#include "homer_i2c.hpp"


namespace homer1 {

namespace {

const char* const TAG = "homer_i2c";

}

namespace i2c {

Device::Device(const i2c_port_t i2c_num) noexcept:
        i2c_num{i2c_num},
        delay{0}
{
}

Device::Device(Device&& other) noexcept:
        i2c_num{other.i2c_num},
        delay{other.delay}
{
}

Device& Device::operator=(Device&& other) noexcept
{
    if (this == &other)
        return *this;

    this->i2c_num = other.i2c_num;
    this->delay = other.delay;

    return *this;
}

}

namespace i2c {

HwErr Device::write_to_slave(uint8_t addr,
                             const uint8_t* data,
                             size_t size) const
{
    this->ensure_delay_set();

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    esp_err_t err;

    err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::write_to_slave", "error 00 i2c_master_start_error: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_START | ERROR_MASTER_WRITE_SLAVE, err};
    }

    err = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::write_to_slave", "error 01 i2c_master_write_byte: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_WRITE_BYTE | ERROR_MASTER_WRITE_SLAVE, err};
    }

    err = i2c_master_write(cmd, data, size, true);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::write_to_slave", "error 02 i2c_master_write: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_WRITE | ERROR_MASTER_WRITE_SLAVE, err};
    }

    err = i2c_master_stop(cmd);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::write_to_slave", "error 03 i2c_master_stop: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_STOP | ERROR_MASTER_WRITE_SLAVE, err};
    }

    err = i2c_master_cmd_begin(this->i2c_num, cmd, this->delay);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::write_to_slave", "error 04 i2c_master_cmd_begin: %d", err);
        switch (err) {
            case ESP_ERR_INVALID_ARG:
                ESP_LOGE("homer_i2c::write_to_slave",
                         "error 04 i2c_master_cmd_begin: ESP_ERR_INVALID_ARG -> parameter error");
                break;
            case ESP_FAIL:
                ESP_LOGE("homer_i2c::write_to_slave", "error 04 i2c_master_cmd_begin: ESP_FAIL -> no ack");
                break;
            case ESP_ERR_INVALID_STATE:
                ESP_LOGE("homer_i2c::write_to_slave",
                         "error 04 i2c_master_cmd_begin: ESP_ERR_INVALID_STATE -> no driver or not master mode");
                break;
            case ESP_ERR_TIMEOUT:
                ESP_LOGE("homer_i2c::write_to_slave", "error 04 i2c_master_cmd_begin: ESP_ERR_TIMEOUT -> busy bus");
                break;
            default:
                break;
        }
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_CMD_BEGIN | ERROR_MASTER_WRITE_SLAVE, err};
    }

    i2c_cmd_link_delete(cmd);
    return HwErr::make_ok();
}

HwErr Device::read_from_slave(uint8_t addr,
                              uint8_t* data,
                              size_t size) const
{
    this->ensure_delay_set();

    if (size == 0)
        return HwErr::make_ok();

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    esp_err_t err;

    err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave", "error 05 i2c_master_start_error: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_START | ERROR_MASTER_READ_SLAVE, err};
    }

    err = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave", "error 06 i2c_master_write_byte: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_WRITE_BYTE | ERROR_MASTER_READ_SLAVE, err};
    }

    if (size > 1) {
        err = i2c_master_read(cmd, data, size - 1, i2c_ack_type_t::I2C_MASTER_ACK);
        if (err != ESP_OK) {
            ESP_LOGE("homer_i2c::read_from_slave", "error 07 i2c_master_read: %d", err);
            i2c_cmd_link_delete(cmd);
            return {ERROR_I2C_MASTER_READ | ERROR_MASTER_READ_SLAVE, err};
        }
    }

    err = i2c_master_read_byte(cmd, data + size - 1, i2c_ack_type_t::I2C_MASTER_NACK);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave", "error 08 i2c_master_read_byte: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_READ_BYTE | ERROR_MASTER_READ_SLAVE, err};
    }

    err = i2c_master_stop(cmd);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave", "error 09 i2c_master_stop: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_STOP | ERROR_MASTER_READ_SLAVE, err};
    }

    err = i2c_master_cmd_begin(this->i2c_num, cmd, this->delay);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave", "error 10 i2c_master_cmd_begin: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_CMD_BEGIN | ERROR_MASTER_READ_SLAVE, err};
    }

    i2c_cmd_link_delete(cmd);
    return HwErr::make_ok();
}

HwErr Device::read_from_slave(uint8_t addr,
                              uint8_t reg,
                              uint8_t* data,
                              size_t size) const
{
    this->ensure_delay_set();

    if (size == 0)
        return HwErr::make_ok();

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    esp_err_t err;

    err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave(reg)", "error 11 i2c_master_start_error: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_START | ERROR_MASTER_READ_SLAVE, err};
    }

    err = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave(reg)", "error 12 i2c_master_write_byte: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_WRITE_BYTE | ERROR_MASTER_READ_SLAVE, err};
    }

    err = i2c_master_write_byte(cmd, reg, true);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave(reg)", "error 13 i2c_master_write_byte: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_WRITE_BYTE | ERROR_MASTER_READ_SLAVE, err};
    }

    err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave(reg)", "error 14 i2c_master_start_error: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_START | ERROR_MASTER_READ_SLAVE, err};
    }

    err = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave(reg)", "error 15 i2c_master_write_byte: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_WRITE_BYTE | ERROR_MASTER_READ_SLAVE, err};
    }

    if (size > 1) {
        err = i2c_master_read(cmd, data, size - 1, i2c_ack_type_t::I2C_MASTER_ACK);
        if (err != ESP_OK) {
            ESP_LOGE("homer_i2c::read_from_slave(reg)", "error 16 i2c_master_read: %d", err);
            i2c_cmd_link_delete(cmd);
            return {ERROR_I2C_MASTER_READ | ERROR_MASTER_READ_SLAVE, err};
        }
    }

    err = i2c_master_read_byte(cmd, data + size - 1, i2c_ack_type_t::I2C_MASTER_NACK);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave(reg)", "error 17 i2c_master_read_byte: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_READ_BYTE | ERROR_MASTER_READ_SLAVE, err};
    }

    err = i2c_master_stop(cmd);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave(reg)", "error 18 i2c_master_stop: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_STOP | ERROR_MASTER_READ_SLAVE, err};
    }

    ESP_LOGI("DELAY", "DEL: %lu", this->delay);
    err = i2c_master_cmd_begin(this->i2c_num, cmd, this->delay);
    if (err != ESP_OK) {
        ESP_LOGE("homer_i2c::read_from_slave(reg)", "error 19 i2c_master_cmd_begin: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_CMD_BEGIN | ERROR_MASTER_READ_SLAVE, err};
    }

    i2c_cmd_link_delete(cmd);
    return HwErr::make_ok();
}


HwErr Device::read_int16(uint8_t addr,
                         const uint8_t reg,
                         int16_t& value) const
{
    auto err = this->write_to_slave(addr, &reg, 1);

    if (err.is_ok()) {
        uint8_t data[2] = {0};
        err = this->read_from_slave(addr, data, 2);

        if (err.is_ok()) {
            value = static_cast<int16_t>((data[0] << 8) | data[1]);
            return err;
        }
    }

    ESP_LOGE("homer_i2c::read_int16", "read [0x%02x]::int16 failed", reg);
    return err;
}

HwErr Device::read_uint16(uint8_t addr,
                          const uint8_t reg,
                          uint16_t& value) const
{
    auto err = this->write_to_slave(addr, &reg, 1);

    if (err.is_ok()) {
        uint8_t data[2] = {0};
        err = this->read_from_slave(addr, data, 2);

        if (err.is_ok()) {
            value = static_cast<uint16_t>((data[0] << 8) | data[1]);
            return err;
        }
    }

    ESP_LOGE("homer_i2c::read_int16", "read [0x%02x]::uint16 failed", reg);
    return err;
}

HwErr Device::read_uint32(uint8_t addr,
                          const uint8_t reg,
                          uint32_t& value) const
{
    auto err = this->write_to_slave(addr, &reg, 1);

    if (err.is_ok()) {
        uint8_t data[3] = {0};
        err = this->read_from_slave(addr, data, 3);

        if (err.is_ok()) {
            value = static_cast<uint32_t>((data[0] << 16) | (data[1] << 8) | data[2]);
            return err;
        }
    }

    ESP_LOGE("homer_i2c::read_int16", "read [0x%02x]::uint16 failed", reg);
    return err;
}

HwErr Device::write(uint8_t addr,
                    const uint8_t a,
                    const uint8_t b) const
{
    const uint8_t data[] = {a, b};
    const auto err = this->write_to_slave(addr, data, 2);

    if (err.is_ok())
        return err;

    ESP_LOGE("homer_i2c::write", "write 0x%02x-0x%02x failed", a, b);
    return err;
}


void Device::ensure_delay_set() const
{
    if (this->delay <= 0)
        throw std::runtime_error("i2c delay not set");
}

void Device::set_delay(TickType_t new_delay) noexcept
{
    this->delay = new_delay;
}

}

}
