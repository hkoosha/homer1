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
    HomerSensorDump as_map{};

    this->do_dump(as_map);

    return as_map;
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