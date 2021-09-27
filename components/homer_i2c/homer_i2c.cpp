#include <cstdint>
#include <cstddef>

#include "esp_log.h"

#include "homer_util.hpp"
#include "homer_i2c.hpp"


namespace homer1 {

namespace {

const char* TAG = "homer_i2c";

}

namespace i2c {

Device::Device(const i2c_port_t i2c_num,
               const uint8_t addr,
               const TickType_t delay) noexcept:
        i2c_num{i2c_num},
        addr{addr},
        delay{delay}
{
}

Device::Device(Device&& other) noexcept:
        i2c_num{other.i2c_num},
        addr{other.addr},
        delay{other.delay}
{
}

Device& Device::operator=(Device&& other) noexcept
{
    if (this == &other)
        return *this;

    this->i2c_num = other.i2c_num;
    this->addr = other.addr;
    this->delay = other.delay;

    return *this;
}


HwErr Device::write_to_slave(const uint8_t* data,
                             size_t size) const noexcept
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    esp_err_t err;

    err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c_master_start_error: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_START | ERROR_MASTER_WRITE_SLAVE, err};
    }

    err = i2c_master_write_byte(cmd, (this->addr << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c_master_write_byte: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_WRITE_BYTE | ERROR_MASTER_WRITE_SLAVE, err};
    }

    err = i2c_master_write(cmd, data, size, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c_master_write: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_WRITE | ERROR_MASTER_WRITE_SLAVE, err};
    }

    err = i2c_master_stop(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c_master_stop: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_STOP | ERROR_MASTER_WRITE_SLAVE, err};
    }

    err = i2c_master_cmd_begin(this->i2c_num, cmd, this->delay);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c_master_cmd_begin: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_CMD_BEGIN | ERROR_MASTER_WRITE_SLAVE, err};
    }

    i2c_cmd_link_delete(cmd);
    return HwErr::make_ok();
}

HwErr Device::read_from_slave(uint8_t* data,
                              size_t size) const noexcept
{
    if (size == 0)
        return HwErr::make_ok();

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    esp_err_t err;

    err = i2c_master_start(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c_master_start_error: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_START | ERROR_MASTER_READ_SLAVE, err};
    }

    err = i2c_master_write_byte(cmd, (this->addr << 1) | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c_master_write_byte: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_WRITE_BYTE | ERROR_MASTER_READ_SLAVE, err};
    }

    if (size > 1) {
        err = i2c_master_read(cmd, data, size - 1, i2c_ack_type_t::I2C_MASTER_ACK);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "error i2c_master_read: %d", err);
            i2c_cmd_link_delete(cmd);
            return {ERROR_I2C_MASTER_READ | ERROR_MASTER_READ_SLAVE, err};
        }
    }

    err = i2c_master_read_byte(cmd, data + size - 1, i2c_ack_type_t::I2C_MASTER_NACK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c_master_read_byte: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_READ_BYTE | ERROR_MASTER_READ_SLAVE, err};
    }

    err = i2c_master_stop(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c_master_stop: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_STOP | ERROR_MASTER_READ_SLAVE, err};
    }

    err = i2c_master_cmd_begin(this->i2c_num, cmd, this->delay);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "error i2c_master_cmd_begin: %d", err);
        i2c_cmd_link_delete(cmd);
        return {ERROR_I2C_MASTER_CMD_BEGIN | ERROR_MASTER_READ_SLAVE, err};
    }

    i2c_cmd_link_delete(cmd);
    return HwErr::make_ok();
}


HwErr Device::read_int16(const uint8_t reg,
                         int16_t& value) const noexcept
{
    auto err = this->write_to_slave(&reg, 1);

    if (err.is_ok()) {
        uint8_t data[2] = {0};
        err = this->read_from_slave(data, 2);

        if (err.is_ok()) {
            value = static_cast<int16_t>((data[0] << 8) | data[1]);
            return err;
        }
    }

    ESP_LOGE(TAG, "read [0x%02x]::int16 failed", reg);
    return err;
}

HwErr Device::read_uint16(const uint8_t reg,
                          uint16_t& value) const noexcept
{
    auto err = this->write_to_slave(&reg, 1);

    if (err.is_ok()) {
        uint8_t data[2] = {0};
        err = this->read_from_slave(data, 2);

        if (err.is_ok()) {
            value = static_cast<uint16_t>((data[0] << 8) | data[1]);
            return err;
        }
    }

    ESP_LOGE(TAG, "read [0x%02x]::uint16 failed", reg);
    return err;
}

HwErr Device::read_uint32(const uint8_t reg,
                          uint32_t& value) const noexcept
{
    auto err = this->write_to_slave(&reg, 1);

    if (err.is_ok()) {
        uint8_t data[3] = {0};
        err = this->read_from_slave(data, 3);

        if (err.is_ok()) {
            value = static_cast<uint32_t>((data[0] << 16) | (data[1] << 8) | data[2]);
            return err;
        }
    }

    ESP_LOGE(TAG, "read [0x%02x]::uint16 failed", reg);
    return err;
}

HwErr Device::write(const uint8_t a,
                    const uint8_t b) const noexcept
{
    const uint8_t data[] = {a, b};
    const auto err = this->write_to_slave(data, 2);

    if (err.is_ok())
        return err;

    ESP_LOGE(TAG, "write 0x%02x-0x%02x failed", a, b);
    return err;
}

}

}
