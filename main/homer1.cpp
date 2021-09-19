#include <iostream>
#include <cstdint>
// #include <cstddef>


#include "my_init.hpp"
#include "my_util.hpp"
#include "my_s8.hpp"


using std::cout;
using std::cin;
using std::endl;


const char* fucker = "Go fuck yourself: ";
const char* ending = "\r\n";


void init(void) {
    
    cout << "init..." << endl;

    my_nvs_init();
    // my_wifi_init();
    my_uart_init_8n1_9600(UART_NUM_2, 13, 12);
}

void work(void) {
    uint32_t i = 0;

    MyS8 s8 { UART_NUM_2 };

    do {
        cout << "f: " << i++ << endl;

        const auto co2 = s8.get_co2();
        cout << "co2: " << co2 << endl;

        const auto days = s8.get_abc_days();
        cout << "abc: " << days << endl;

        const auto id = s8.get_sensor_id();
        cout << "sid: 0x" << std::uppercase << std::hex << id << std::dec << std::nouppercase << endl;
        
        const auto fw = s8.get_sensor_fw();
        cout << "sfw: 0x" << std::uppercase << std::hex << fw << std::dec << std::nouppercase;
        cout << " (" << fw << ")" << endl;

        my_sleep_millis(1000);
    } while(true);
}

extern "C" void app_main(void) {
    init();

    cout << "going to fuck myself." << endl;

    work();
}
