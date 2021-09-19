#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <sstream>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

using std::size_t;
using std::uint32_t;
using std::uint64_t;

namespace homer1 {

inline void my_sleep_ticks(const size_t ticks) noexcept
{
    TickType_t xDelay = ticks / portTICK_PERIOD_MS;
    if (xDelay == 0)
        xDelay = 1;
    vTaskDelay(xDelay);
}

inline void my_sleep_millis(const uint32_t d) noexcept
{
    vTaskDelay(pdMS_TO_TICKS(d));
}

inline uint64_t now_millis() noexcept
{
    const auto uptime = esp_timer_get_time();
    return uptime <= 0
           ? 0
           : static_cast<uint64_t>(uptime / 1000);
}


std::string uint64_to_bin(uint64_t n,
                          bool expand_zero = false,
                          bool prefix = true) noexcept;

void print_sensor_dump_header(std::stringstream& ss) noexcept;

}
