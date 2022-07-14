#pragma once

#include <limits>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "homer_util.hpp"

using std::uint64_t;


namespace homer1 {

const char* const SENSOR_ATTR_TIME_TO_READ = "TTR";
const char* const SENSOR_ATTR_SENSOR_ERR = "err_v_sensor";
const char* const SENSOR_ATTR_HW_ERR = "err_v_hw";
const char* const SENSOR_ATTR_SENSOR_ERR_MSG = "err_msg_sensor";
const char* const SENSOR_ATTR_HW_ERR_MSG = "err_msg_hw";
const char* const SENSOR_ATTR_SENSOR_ERR_BIN = "err_bin_sensor";
const char* const SENSOR_ATTR_HW_ERR_BIN = "err_bin_hw";

const char* const EMPTY = "";

using HomerSensorDump = std::map<std::string, std::string>;

// Because clion goes crazy if I make this insert call on site.
template<typename V>
inline void insert(HomerSensorDump& map,
                   const char* name,
                   const V& value) noexcept
{
    map.insert({name, std::to_string(value)});
}

// Because clion goes crazy if I make this insert call on site.
inline void insert_str(HomerSensorDump& map,
                       const char* name,
                       const char* value) noexcept
{
    map.insert({name, value});
}

// Because clion goes crazy if I make this insert call on site.
inline void insert_str(HomerSensorDump& map,
                       const std::string& name,
                       const std::string& value) noexcept
{
    map.insert({name, value});
}

// Because clion goes crazy if I make this insert call on site.
template<typename T>
inline void push_back(std::vector<T>& vector,
                      const T& value) noexcept
{
    vector.push_back(value);
}

}

namespace homer1 {

template<typename DATA>
class HomerSensor;

class HomerSensorData
{
public:
    HomerSensorData& operator=(const HomerSensorData& other) noexcept;

    HomerSensorData& operator=(HomerSensorData&& other) noexcept;

    HomerSensorData(const HomerSensorData& other) noexcept = default;

    HomerSensorData(HomerSensorData&& other) noexcept;


    virtual ~HomerSensorData() noexcept = default;


    void dump(std::stringstream& ss) const noexcept;

    [[nodiscard]] HomerSensorDump dump(bool prefix_with_sensor_name = true) const noexcept;

    void influxdb(std::vector<std::string>& measurements) const noexcept;

    void prometheus(std::stringstream& ss) const noexcept;


    [[nodiscard]] const HwErr& get_error() const noexcept;

    HwErr& _get_error() noexcept;


    const char* hw_err_to_str(const HwErr* err = nullptr) const noexcept;

    const char* sensor_err_to_str(const HwErr* err = nullptr) const noexcept;

    [[nodiscard]] const char* name() const noexcept;

protected:
    explicit HomerSensorData(const char* name) noexcept;

    HomerSensorData(HwErr&& error,
                    uint64_t time_to_read,
                    uint64_t read_start,
                    const char* name) noexcept;


    virtual void do_dump(HomerSensorDump& map) const noexcept = 0;

    virtual void do_dump(std::stringstream& ss) const noexcept = 0;

    [[nodiscard]] virtual const char* do_hw_err_to_str(esp_err_t err) const noexcept;

    [[nodiscard]] virtual const char* do_sensor_err_to_str(uint64_t err) const noexcept = 0;


    virtual void invalidate() noexcept = 0;

    HwErr error;

    uint64_t _time_to_read;
    uint64_t _read_start;
    const char* _name;

    template<typename DATA>
    friend
    class HomerSensor;

private:
    void dump_header(std::stringstream& ss) const noexcept;

    void start_read() noexcept;

    void end_read() noexcept;
};

template<typename DATA>
class HomerSensor
{
public:
    HomerSensor& operator=(const HomerSensor& other) = delete;

    HomerSensor(const HomerSensor& other) = delete;

    HomerSensor(HomerSensor&& other) noexcept = delete;


    HomerSensor& operator=(HomerSensor&& other) noexcept
    {
        if (this == &other)
            return *this;

        this->_refresh_every = other._refresh_every;
        this->_last_update = other._last_update;

        return *this;
    }


    virtual ~HomerSensor() noexcept = default;


    const DATA& read_data() noexcept
    {
        DATA& d = this->get_raw_data();
        HomerSensorData& hsd = d;

        if (hsd.error.has_error() || (now_millis() - this->_last_update) > this->_refresh_every) {
            hsd.invalidate();
            hsd._get_error().mark_ok_no_data();
            hsd.start_read();
            this->refresh_data();
            hsd.end_read();
            if (hsd.error.has_error())
                hsd.invalidate();
            this->_last_update = now_millis();
        }

        return d;
    }

protected:
    explicit HomerSensor(uint64_t refresh_every) noexcept:
            _refresh_every{refresh_every},
            _last_update{0}
    {
    }

    HomerSensor(uint64_t refresh_every,
                uint64_t last_update) noexcept:
            _refresh_every{refresh_every},
            _last_update{last_update}
    {
    }


    virtual void refresh_data() noexcept = 0;

    virtual DATA& get_raw_data() noexcept = 0;

    uint64_t _refresh_every;
    uint64_t _last_update;
};

}
