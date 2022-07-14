#pragma once

namespace homer1cfg {

const uint32_t MEASUREMENT_DELAY = 1000;
const uint32_t PRINT_DELAY = 30000;
const uint32_t PRINT_INITIAL_DELAY = 5000;
const uint32_t PUSH_DELAY = 5000;
const uint32_t PUSH_INITIAL_DELAY = 5000;

const char* const PROMETHEUS_HANDLER_URI = "/metrics";

const int UART_BUFFER_SIZE = 1024;
const uint32_t MY_I2C_CLOCK_SPEED = 100000;

const uart_port_t PMS5003_UART_PORT = UART_NUM_1;
const int PMS5003_UART_TX_PIN = 25;
const int PMS5003_UART_RX_PIN = 26;

const uart_port_t S8_UART_PORT = UART_NUM_2;
const int S8_UART_TX_PIN = 13;
const int S8_UART_RX_PIN = 27;

const i2c_port_t BMP180_I2C_PORT = I2C_NUM_0;
const int BMP180_I2C_SDA_PIN = 33;
const int BMP180_I2C_SCL_PIN = 32;

const i2c_port_t SHT3X_I2C_PORT = I2C_NUM_1;
const int SHT3X_I2C_SDA_PIN = 17;
const int SHT3X_I2C_SCL_PIN = 18;

const i2c_port_t CCS811_I2C_PORT = I2C_NUM_1;
const int CCS811_I2C_SDA_PIN = 17;
const int CCS811_I2C_SCL_PIN = 18;

}

