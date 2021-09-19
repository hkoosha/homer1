#pragma once

#include <cstdint>
#include <cstddef>
#include "driver/i2c.h"

using std::size_t;
using std::int16_t;
using std::uint8_t;
using std::uint16_t;
using std::uint32_t;
using std::uint64_t;

namespace homer1 {
namespace i2c {

const uint64_t ERROR_NONE = 0;
//---
const uint64_t ERROR_I2C_MASTER_START = 1;
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

class Device final
{
public:

    Device& operator=(const Device& other) = delete;

    Device& operator=(Device&& other) = delete;


    Device(const Device& other) noexcept = default;

    Device(Device&& other) noexcept = default;

    explicit Device(i2c_port_t i2c_num,
                    uint8_t addr,
                    TickType_t delay) noexcept;

    ~Device() noexcept = default;


    uint64_t write_to_slave(const uint8_t* data,
                            size_t size) const noexcept;

    uint64_t read_from_slave(uint8_t* data,
                             size_t size) const noexcept;

    uint64_t write(uint8_t,
                   uint8_t) const noexcept;

    uint64_t read_int16(uint8_t reg,
                        int16_t& value) const noexcept;

    uint64_t read_uint16(uint8_t reg,
                         uint16_t& value) const noexcept;

    uint64_t read_uint32(uint8_t reg,
                         uint32_t& value) const noexcept;

private:

    const i2c_port_t i2c_num;
    const uint8_t addr;
    const TickType_t delay;
};

}
}
