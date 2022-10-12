#pragma once

#include <cstdint>
#include <cstddef>
#include <utility>

#include "driver/i2c.h"

using std::size_t;
using std::int16_t;
using std::uint8_t;
using std::uint16_t;
using std::uint32_t;
using std::uint64_t;


namespace homer1::i2c {

const uint64_t ERROR_I2C_MASTER_START = ERROR_NO_DATA_AVAILABLE << 1;
const uint64_t ERROR_I2C_MASTER_WRITE_BYTE = ERROR_I2C_MASTER_START << 1;
const uint64_t ERROR_I2C_MASTER_WRITE = ERROR_I2C_MASTER_WRITE_BYTE << 1;
const uint64_t ERROR_I2C_MASTER_STOP = ERROR_I2C_MASTER_WRITE << 1;
const uint64_t ERROR_I2C_MASTER_CMD_BEGIN = ERROR_I2C_MASTER_STOP << 1;
const uint64_t ERROR_I2C_MASTER_READ = ERROR_I2C_MASTER_CMD_BEGIN << 1;
const uint64_t ERROR_I2C_MASTER_READ_BYTE = ERROR_I2C_MASTER_READ << 1;
//---
const uint64_t ERROR_MASTER_READ_SLAVE = ERROR_I2C_MASTER_READ_BYTE << 1;
const uint64_t ERROR_MASTER_WRITE_SLAVE = ERROR_MASTER_READ_SLAVE << 1;
//---
const uint64_t ERROR_READ_INT16 = ERROR_MASTER_WRITE_SLAVE << 1;
const uint64_t ERROR_READ_UINT16 = ERROR_READ_INT16 << 1;
const uint64_t ERROR_READ_UINT32 = ERROR_READ_UINT16 << 1;
const uint64_t ERROR_WRITE_REG = ERROR_READ_UINT32 << 1;
//---
const uint64_t ERROR_MAX = ERROR_WRITE_REG;

// TODO is this used?
const char* err_to_string(uint64_t err) noexcept;

class Device final
{
public:
    Device& operator=(const Device& other) = delete;

    Device(const Device& other) = delete;


    Device& operator=(Device&& other) noexcept;

    Device(Device&& other) noexcept;;

    explicit Device(i2c_port_t i2c_num) noexcept;

    ~Device() noexcept = default;


    void set_delay(TickType_t delay) noexcept;


    [[nodiscard]] HwErr write_to_slave(uint8_t addr,
                                       const uint8_t* data,
                                       size_t size) const;

    [[nodiscard]] HwErr read_from_slave(uint8_t addr,
                                        uint8_t* data,
                                        size_t size) const;

    [[nodiscard]] HwErr read_from_slave(uint8_t addr,
                                        uint8_t reg,
                                        uint8_t* data,
                                        size_t size) const;


    [[nodiscard]] HwErr write(uint8_t addr,
                              uint8_t,
                              uint8_t) const;

    [[nodiscard]] HwErr read_int16(uint8_t addr,
                                   uint8_t reg,
                                   int16_t& value) const;

    [[nodiscard]] HwErr read_uint16(uint8_t addr,
                                    uint8_t reg,
                                    uint16_t& value) const;

    [[nodiscard]] HwErr read_uint32(uint8_t addr,
                                    uint8_t reg,
                                    uint32_t& value) const;

private:

    void ensure_delay_set() const;

    i2c_port_t i2c_num;
    TickType_t delay;
};

}
