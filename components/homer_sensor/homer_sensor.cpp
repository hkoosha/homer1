#include <sstream>
#include <cstdint>

#include "homer_util.hpp"
#include "homer_sensor.hpp"

using std::uint64_t;


namespace homer1 {

void HomerSensorData::dump_header(std::stringstream& ss) const noexcept
{
    ss << "ERR: "
       << uint64_to_bin(this->error.hardware_err())
       << " / "
       << uint64_to_bin(this->error.sensor_err())
       << std::endl;
    ss << "TTR: " << this->time_to_read << std::endl;
}

void HomerSensorData::dump(std::stringstream& ss) const noexcept
{
    this->dump_header(ss);
    this->do_dump(ss);
}

HomerSensorDump HomerSensorData::dump() const noexcept
{
    auto map = HomerSensorDump{};

    insert(map, SENSOR_ATTR_TIME_TO_READ, this->time_to_read);
    insert(map, SENSOR_ATTR_HW_ERR, this->error.hardware_err());
    insert(map, SENSOR_ATTR_SENSOR_ERR, this->error.sensor_err());
    insert(map, SENSOR_ATTR_TIME_TO_READ, this->time_to_read);

    if (this->error.is_ok())
        this->do_dump(map);

    return map;
}


void HomerSensorData::start_read() noexcept
{
    this->read_start = now_millis();
}

void HomerSensorData::end_read() noexcept
{
    assert(this->read_start > 0);
    this->time_to_read = now_millis() - this->read_start;
}


const HwErr& HomerSensorData::get_error() const noexcept
{
    return this->error;
}

HwErr& HomerSensorData::_get_error() noexcept
{
    return this->error;
}

const char* HomerSensorData::do_hw_err_to_str(esp_err_t err) const noexcept
{
    return esp_err_to_name(err);
}

const char* HomerSensorData::hw_err_to_str(const HwErr* err) noexcept
{
    if (err == nullptr)
        err = &this->get_error();

    const auto msg = this->do_hw_err_to_str(err->hardware_err());
    return msg == nullptr ? EMPTY : msg;
}

const char* HomerSensorData::sensor_err_to_str(const HwErr* err) noexcept
{
    if (err == nullptr)
        err = &this->get_error();

    if (err->sensor_err() == ERROR_NONE || err->sensor_err() == ERROR_NO_DATA_AVAILABLE)
        return ::homer1::err_to_string(err->sensor_err());

    const auto msg = this->do_sensor_err_to_str(err->sensor_err());
    return msg == nullptr ? EMPTY : msg;
}


HomerSensorData& HomerSensorData::operator=(const HomerSensorData& other) noexcept
{
    if (this == &other)
        return *this;

    this->error = other.error;
    this->time_to_read = other.time_to_read;
    this->read_start = other.read_start;

    return *this;
}

HomerSensorData& HomerSensorData::operator=(HomerSensorData&& other) noexcept
{
    if (this == &other)
        return *this;

    this->error = std::move(other.error);
    this->time_to_read = other.time_to_read;
    this->read_start = other.read_start;

    return *this;
}

HomerSensorData::HomerSensorData() noexcept:
        error{HwErr::make_no_data()},
        time_to_read{std::numeric_limits<uint64_t>::max()},
        read_start{0}
{
}

HomerSensorData::HomerSensorData(const HomerSensorData& other) noexcept:
        error{other.error},
        time_to_read{other.time_to_read},
        read_start{other.read_start}
{
    this->error = other.error;
    this->time_to_read = other.time_to_read;
    this->read_start = other.read_start;
}

HomerSensorData::HomerSensorData(HomerSensorData&& other) noexcept:
        error{std::move(other.error)},
        time_to_read{other.time_to_read},
        read_start{other.read_start}
{
}

}