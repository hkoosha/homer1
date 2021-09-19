#include <cstdint>
#include <sstream>
#include <string>
#include <algorithm>

#include "util.hpp"

using std::uint32_t;
using std::uint64_t;
using std::int32_t;
using std::endl;


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
    ss << endl << "=================================== ";

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

    ss << endl;
}

}
