#include <cstdint>
#include <sstream>
#include <string>
#include <algorithm>

#include "esp_log.h"

#include "homer_util.hpp"

using std::uint32_t;
using std::uint64_t;


namespace homer1 {

std::string uint64_to_bin(uint64_t n,
                          const bool expand_zero,
                          const bool prefix) noexcept
{
    std::string bin;

    if (n == 0 && !expand_zero) {
        bin += '0';
        return bin;
    }

    for (int i = 0; i < 32; i++) {
        bin += n & 1 ? '1' : '0';
        n >>= 1;
    }

    if (prefix) {
        bin += 'b';
        bin += '0';
    }

    std::reverse(bin.begin(), bin.end());

    return bin;
}

void print_sensor_dump_header(std::stringstream& ss) noexcept
{
    ss << std::endl << "=================================== ";

    const auto uptime = now_millis();
    const auto uptime_seconds = uptime / 1000;
    const uint32_t seconds = uptime_seconds % 60;
    const uint32_t minutes_all = (uptime_seconds - seconds) / 60;
    const uint32_t minutes = minutes_all % 60;
    const uint32_t hours = (minutes_all - minutes) / 60;

    // TODO use format() instead of this nonsense.
    ss << "[";
    if (hours < 10)
        ss << '0';
    ss << hours << ':';
    if (minutes < 10)
        ss << '0';
    ss << minutes << ':';
    if (seconds < 10)
        ss << '0';
    ss << seconds << "]";

    ss << std::endl;
}

const char* err_to_string(const uint64_t err) noexcept
{
    switch (err) {
        case ERROR_NO_DATA_AVAILABLE:
            return "no_data_available";

        case ERROR_NONE:
            return "NO_ERROR";

        default:
            return nullptr;
    }
}

}

namespace homer1 {

namespace {
const char* HwErrTag = "HwErr";
}

HwErr& HwErr::operator=(const HwErr& other) noexcept
{
    if (this == &other)
        return *this;

    this->_sensor_err = other._sensor_err;
    this->_hardware_err = other._hardware_err;

    return *this;
}

HwErr& HwErr::operator=(HwErr&& other) noexcept
{
    if (this == &other)
        return *this;

    this->_sensor_err = other._sensor_err;
    this->_hardware_err = other._hardware_err;

    return *this;
}

HwErr::HwErr(HwErr&& other) noexcept:
        _sensor_err{other._sensor_err},
        _hardware_err{other._hardware_err}
{
}

HwErr::HwErr(uint64_t sensor_err,
             esp_err_t hardware_err) noexcept:
        _sensor_err{sensor_err},
        _hardware_err{hardware_err}
{
}

HwErr HwErr::make_no_data() noexcept
{
    return {ERROR_NO_DATA_AVAILABLE, ESP_OK};
}

HwErr HwErr::make_sensor_err(uint64_t err) noexcept
{
    return {err, ESP_OK};
}

HwErr HwErr::make_hardware_err(esp_err_t err) noexcept
{
    return {ERROR_NONE, err};
}

HwErr HwErr::make(const uint64_t sensor_err,
                  const esp_err_t hardware_err) noexcept
{
    return {sensor_err, hardware_err};
}


uint64_t HwErr::sensor_err() const noexcept
{
    return this->_sensor_err;
}

esp_err_t HwErr::hardware_err() const noexcept
{
    return this->_hardware_err;
}

bool HwErr::is_ok() const noexcept
{
    return this->_sensor_err == ERROR_NONE &&
           this->_hardware_err == ESP_OK;
}

bool HwErr::has_error() const noexcept
{
    return !this->is_ok();
}

bool HwErr::is_ok_or_no_data() const noexcept
{
    if (this->is_ok())
        return true;

    return (!this->has_hardware_err()) && (this->sensor_err() == ERROR_NO_DATA_AVAILABLE);
}

bool HwErr::has_sensor_err(const uint64_t err) const noexcept
{
    return (this->_sensor_err & err) != ERROR_NONE;
}

bool HwErr::has_hardware_err(esp_err_t err) const noexcept
{
    return (this->_hardware_err & err) != ESP_OK;
}

bool HwErr::has_sensor_err() const noexcept
{
    return this->_sensor_err != ERROR_NONE;
}

bool HwErr::has_hardware_err() const noexcept
{
    return this->_hardware_err != ESP_OK;
}


void HwErr::add_sensor_err(const uint64_t err) noexcept
{
    this->_sensor_err |= err;
}

void HwErr::set_hardware_err(const esp_err_t err) noexcept
{
    if (this->_hardware_err != ESP_OK)
        ESP_LOGW(HwErrTag, "already have hardware err, previous=%d, new=%d", this->_hardware_err, err);

    this->_hardware_err = err;
}

void HwErr::clear_sensor_error() noexcept
{
    this->_sensor_err = ERROR_NONE;
}

void HwErr::clear_hardware_error() noexcept
{
    this->_hardware_err = ESP_OK;
}

void HwErr::mark_ok_no_data() noexcept
{
    this->clear_sensor_error();
    this->clear_hardware_error();
    this->add_sensor_err(ERROR_NO_DATA_AVAILABLE);
}

void HwErr::mark_ok_has_data() noexcept
{
    this->_sensor_err = ERROR_NONE;
    this->_hardware_err = ESP_OK;
}

HwErr HwErr::make_ok() noexcept
{
    return {ERROR_NONE, ESP_OK};
}

void HwErr::merge_from(const HwErr& other) noexcept
{
    this->_sensor_err |= other.sensor_err();
    this->set_hardware_err(other.hardware_err());
}

}
