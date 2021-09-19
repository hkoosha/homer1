#pragma once

#include "driver/uart.h"


void my_sleep_millis(const uint32_t);


int my_uart_say(const uart_port_t,
                const char*);

