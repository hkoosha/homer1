#include <sstream>
#include <cstdint>
#include <utility>

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
    ss << "TTR: " << this->_time_to_read << std::endl;
}

void HomerSensorData::dump(std::stringstream& ss) const noexcept
{
    this->dump_header(ss);
    this->do_dump(ss);
}

HomerSensorDump HomerSensorData::dump(const bool prefix_with_sensor_name) const noexcept
{
    auto map = HomerSensorDump{};

    if (this->_time_to_read != std::numeric_limits<uint64_t>::max())
        insert(map, SENSOR_ATTR_TIME_TO_READ, this->_time_to_read);

    insert(map, SENSOR_ATTR_HW_ERR, this->error.hardware_err());
    insert(map, SENSOR_ATTR_SENSOR_ERR, this->error.sensor_err());
    insert_str(map, SENSOR_ATTR_SENSOR_ERR_MSG, this->sensor_err_to_str());
    insert_str(map, SENSOR_ATTR_HW_ERR_MSG, this->hw_err_to_str());
    insert_str(map, SENSOR_ATTR_SENSOR_ERR_BIN, uint64_to_bin(this->error.sensor_err(), true));
    insert_str(map, SENSOR_ATTR_HW_ERR_BIN, uint64_to_bin(this->error.hardware_err(), true));

    if (this->error.is_ok())
        this->do_dump(map);

    if (!prefix_with_sensor_name)
        return map;

    const std::string UNDERSCORE = "_";
    auto named_map = HomerSensorDump{};
    for (const auto& item: map)
        insert_str(named_map, this->name() + UNDERSCORE + item.first, item.second);

    return named_map;
}

void HomerSensorData::influxdb(std::vector<std::string>& measurements) const noexcept
{
    for (const auto& item: this->dump(false)) {
        const auto len = item.second.length();
        const auto has_value = len > 0;
        const auto first_is_alpha = has_value && isalpha(item.second[0]);
        const auto is_binary_repr = len > 1 && item.second[0] == '0' && item.second[1] == 'b';

        std::string value = !has_value || first_is_alpha || is_binary_repr
                            ? '"' + item.second + '"'
                            : item.second;

        std::string str = item.first     // measurement name
                          + ",sensor="   // tag
                          + this->_name  // tag value
                          + " "
                          + "value="     // field
                          + value; // field value

        push_back(measurements, str);
    }
}

void HomerSensorData::prometheus(std::stringstream& ss) const noexcept
{
    ss << "# HELP time_to_read_seconds Time taken to read the sensor\n"
          "# TYPE time_to_read_seconds gauge\n"
          "time_to_read_seconds{sensor=\""
       << this->_name
       << "\"} "
       << (static_cast<double>(this->_time_to_read) / 1000)
       << "\n";

    const auto dump = this->dump(false);
    for (const auto& item: dump)
        if (item.first == "TTR" ||
            item.first == "err_bin_hw" || item.first == "err_bin_sensor" ||
            item.first == "err_msg_hw" || item.first == "err_msg_sensor")
            continue;
        else
            ss << "# HELP "
               << item.first
               << " sensor value\n# TYPE "
               << item.first
               << " gauge\n"
               << item.first
               << "{sensor=\""
               << this->_name
               << "\"} "
               << item.second
               << "\n";
}


void HomerSensorData::start_read() noexcept
{
    this->_read_start = now_millis();
}

void HomerSensorData::end_read() noexcept
{
    assert(this->_read_start > 0);
    this->_time_to_read = now_millis() - this->_read_start;
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

const char* HomerSensorData::hw_err_to_str(const HwErr* err) const noexcept
{
    if (err == nullptr)
        err = &this->get_error();

    const auto msg = this->do_hw_err_to_str(err->hardware_err());
    return msg == nullptr ? EMPTY : msg;
}

const char* HomerSensorData::sensor_err_to_str(const HwErr* err) const noexcept
{
    if (err == nullptr)
        err = &this->get_error();

    if (err->sensor_err() == ERROR_NONE || err->sensor_err() == ERROR_NO_DATA_AVAILABLE)
        return ::homer1::err_to_string(err->sensor_err());

    const auto msg = this->do_sensor_err_to_str(err->sensor_err());
    return msg == nullptr ? EMPTY : msg;
}

const char* HomerSensorData::name() const noexcept
{
    return this->_name;
}


HomerSensorData& HomerSensorData::operator=(const HomerSensorData& other) noexcept
{
    if (this == &other)
        return *this;

    this->error = other.error;
    this->_time_to_read = other._time_to_read;
    this->_read_start = other._read_start;
    this->_name = other._name;

    return *this;
}

HomerSensorData& HomerSensorData::operator=(HomerSensorData&& other) noexcept
{
    if (this == &other)
        return *this;

    this->error = std::move(other.error);
    this->_time_to_read = other._time_to_read;
    this->_read_start = other._read_start;
    this->_name = other._name;

    return *this;
}

HomerSensorData::HomerSensorData(const char* _name) noexcept:
        error{HwErr::make_no_data()},
        _time_to_read{std::numeric_limits<uint64_t>::max()},
        _read_start{0},
        _name{_name}
{
}

HomerSensorData::HomerSensorData(HomerSensorData&& other) noexcept:
        error{std::move(other.error)},
        _time_to_read{other._time_to_read},
        _read_start{other._read_start},
        _name{other._name}
{
}

HomerSensorData::HomerSensorData(HwErr&& error,
                                 uint64_t time_to_read,
                                 uint64_t read_start,
                                 const char* name) noexcept:
        error{std::move(error)},
        _time_to_read{time_to_read},
        _read_start{read_start},
        _name{name}
{
}


}