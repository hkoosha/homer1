#pragma once

#include <limits>
#include <cstdint>
#include <unordered_map>
#include <string>

#include "homer_util.hpp"

using std::uint64_t;


namespace homer1 {

const char* const SENSOR_ATTR_TIME_TO_READ = "TTR";
const char* const SENSOR_ATTR_SENSOR_ERR = "sensor_err";
const char* const SENSOR_ATTR_HW_ERR = "hw_err";

const char* const EMPTY = "";

using HomerSensorDump = std::unordered_map<const char*, std::string>;

// Because clion goes crazy if I make this insert call on site.
template<typename V>
inline void insert(HomerSensorDump& map,
                   const char* name,
                   const V& value) noexcept
{
    map.insert({name, std::to_string(value)});
}

template<typename DATA>
class HomerSensor;

class HomerSensorData
{
public:
    HomerSensorData& operator=(const HomerSensorData& other) noexcept;

    HomerSensorData& operator=(HomerSensorData&& other) noexcept;

    HomerSensorData(const HomerSensorData& other) noexcept;

    HomerSensorData(HomerSensorData&& other) noexcept;


    virtual ~HomerSensorData() noexcept = default;


    void dump(std::stringstream& ss) const noexcept;

    HomerSensorDump dump() const noexcept;

    const HwErr& get_error() const noexcept;

    HwErr& _get_error() noexcept;


    const char* hw_err_to_str(const HwErr* err = nullptr) noexcept
    {
        if (err == nullptr)
            err = &this->get_error();

        const auto msg = this->do_hw_err_to_str(err->hardware_err());
        return msg == nullptr ? EMPTY : msg;
    }

    const char* sensor_err_to_str(const HwErr* err = nullptr) noexcept
    {
        if (err == nullptr)
            err = &this->get_error();

        if (err->sensor_err() == ERROR_NONE || err->sensor_err() == ERROR_NO_DATA_AVAILABLE)
            return ::homer1::err_to_string(err->sensor_err());

        const auto msg = this->do_sensor_err_to_str(err->sensor_err());
        return msg == nullptr ? EMPTY : msg;
    }


protected:
    HomerSensorData() noexcept;

    virtual void do_dump(HomerSensorDump& map) const noexcept = 0;

    virtual void do_dump(std::stringstream& ss) const noexcept = 0;

    virtual const char* do_hw_err_to_str(esp_err_t err) const noexcept;

    virtual const char* do_sensor_err_to_str(uint64_t err) const noexcept = 0;


    virtual void invalidate() noexcept = 0;

    HwErr error;

    template<typename DATA>
    friend
    class HomerSensor;

private:
    void dump_header(std::stringstream& ss) const noexcept;

    void start_read() noexcept;

    void end_read() noexcept;

    uint64_t time_to_read;
    uint64_t read_start;
};

template<typename DATA>
class HomerSensor
{
public:
    HomerSensor& operator=(const HomerSensor& other) = delete;

    HomerSensor(const HomerSensor& other) = delete;


    HomerSensor& operator=(HomerSensor&& other) noexcept
    {
        if (this == &other)
            return *this;

        this->refresh_every = other.refresh_every;
        this->last_update = other.last_update;

        return *this;
    }

    HomerSensor(HomerSensor&& other) noexcept:
            refresh_every{other.refresh_every},
            last_update{other.last_update}
    {
    }

    virtual ~HomerSensor() noexcept = default;


    const DATA& read_data() noexcept
    {
        DATA& d = this->get_raw_data();
        HomerSensorData& hsd = d;

        if (hsd.error.has_error() || (now_millis() - this->last_update) > this->refresh_every) {
            hsd.invalidate();
            hsd._get_error().mark_ok_no_data();
            hsd.start_read();
            this->refresh_data();
            hsd.end_read();
            if (hsd.error.has_error())
                hsd.invalidate();
            this->last_update = now_millis();
        }

        return d;
    }

protected:
    explicit HomerSensor(uint64_t refresh_every) noexcept:
            refresh_every{refresh_every},
            last_update{0}
    {
    }

    virtual void refresh_data() noexcept = 0;

    virtual DATA& get_raw_data() noexcept = 0;

private:
    uint64_t refresh_every;
    uint64_t last_update;
};

}