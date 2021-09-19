#include <thread>
#include <chrono>
#include <cstring>


#include "driver/uart.h"



void my_sleep_millis(const uint32_t d)
{
    std::this_thread::sleep_for(std::chrono::milliseconds{ d });
}



int my_uart_say(const uart_port_t port,
                const char* str)
{
    return uart_write_bytes(port, str, std::strlen(str));
}
