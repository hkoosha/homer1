#pragma once

#include <cstddef>


#include "driver/uart.h"


using std::size_t;


class MyS8 final {
public:

    MyS8(const MyS8& other) = delete;
    MyS8& operator=(const MyS8& other) = delete;
    MyS8& operator=(MyS8&& other) = delete;
    MyS8(MyS8&& other) = delete;

    explicit MyS8(const uart_port_t port) noexcept;

    ~MyS8() noexcept;


    int16_t get_co2() noexcept;
    int16_t get_abc_days() noexcept;
    int32_t get_sensor_id() noexcept;
    int16_t get_sensor_fw() noexcept;

private:

    int16_t read_reg(const unsigned char* data) noexcept;
    bool read_and_write_8_bytes(const unsigned char* bytes) noexcept;
    bool write_8_bytes(const unsigned char* bytes) const noexcept;
    bool read_7_bytes() noexcept;


    const uart_port_t port;

    char *uart_buffer;
};