#pragma once

#include "driver/uart.h"


void my_nvs_init(void);


void my_wifi_init(void);


void my_uart_init_8n1_9600(const uart_port_t,
                           const int rx_pim,
                           const int rx_pin);

