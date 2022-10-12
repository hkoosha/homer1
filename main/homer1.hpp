#pragma once

namespace homer1cfg {

const uint32_t PRINT_INITIAL_DELAY = 5000;

const int UART_BUFFER_SIZE = 1024;
const uart_port_t PMS5003_UART_PORT = UART_NUM_1;
const uart_port_t S8_UART_PORT = UART_NUM_2;

const uint32_t MY_I2C_CLOCK_SPEED = 100000;
const i2c_port_t SENSOR_I2C_PORT = I2C_NUM_0;

}

