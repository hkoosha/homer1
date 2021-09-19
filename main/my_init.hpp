#pragma once

#include "driver/uart.h"
#include "driver/i2c.h"


void my_nvs_init();


void my_wifi_init(const char* ssid,
                  const char* password);


void my_uart_init_8n1(uart_port_t,
                      int tx_pin,
                      int rx_pin,
                      uint16_t baudrate = 9600);


const uint32_t MY_I2C_CLOCK_SPEED = 100000;

void my_i2c_init(i2c_port_t,
                 int sda_pin,
                 int scl_pin,
                 uint32_t clock_speed = MY_I2C_CLOCK_SPEED);
