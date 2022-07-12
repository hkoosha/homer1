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

const uint64_t ERROR_NONE = 0;
const uint64_t ERROR_NO_DATA_AVAILABLE = 1;

const char* err_to_string(uint64_t err) noexcept;

}

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


void print_sensor_dump_header(std::stringstream& ss) noexcept;

std::string uint64_to_bin(uint64_t n,
                          bool expand_zero = false,
                          bool prefix = true) noexcept;

}

namespace homer1 {

class HwErr final
{
public:
    HwErr& operator=(const HwErr& other) noexcept;

    HwErr& operator=(HwErr&& other) noexcept;

    HwErr(const HwErr& other) noexcept = default;

    HwErr(HwErr&& other) noexcept;

    HwErr(uint64_t sensor_err,
          esp_err_t hardware_err) noexcept;

    ~HwErr() noexcept = default;

    static HwErr make_ok() noexcept;

    static HwErr make_no_data() noexcept;

    static HwErr make_sensor_err(uint64_t err) noexcept;

    static HwErr make_hardware_err(esp_err_t err) noexcept;

    static HwErr make(uint64_t sensor_err,
                      esp_err_t hardware_err) noexcept;


    [[nodiscard]] uint64_t sensor_err() const noexcept;

    [[nodiscard]] esp_err_t hardware_err() const noexcept;

    [[nodiscard]] bool is_ok() const noexcept;

    [[nodiscard]] bool has_error() const noexcept;

    [[nodiscard]] bool is_ok_or_no_data() const noexcept;

    [[nodiscard]] bool has_sensor_err() const noexcept;

    [[nodiscard]] bool has_hardware_err() const noexcept;

    [[nodiscard]] bool has_sensor_err(uint64_t err) const noexcept;

    [[nodiscard]] bool has_hardware_err(esp_err_t err) const noexcept;


    void add_sensor_err(uint64_t err) noexcept;

    void set_hardware_err(esp_err_t err) noexcept;

    void merge_from(const HwErr& other) noexcept;

    void clear_sensor_error() noexcept;

    void clear_hardware_error() noexcept;

    void mark_ok_no_data() noexcept;

    void mark_ok_has_data() noexcept;


private:
    uint64_t _sensor_err;
    esp_err_t _hardware_err;
};

}
