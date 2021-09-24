#include <stdexcept>
#include <cstdint>
#include <cstddef>
#include <iostream>

#include "esp_log.h"
#include "driver/uart.h"

#include "pms5003.hpp"

using std::size_t;
using std::uint16_t;
using std::int32_t;

using std::endl;


namespace homer1 {

namespace {
const char* TAG = "Sensor";

const auto UART_READ_WAIT = 500;
const auto UART_READ_WAIT_TICKS = UART_READ_WAIT / portTICK_RATE_MS;

const auto MAX_RETRIES = 10;

const auto MAGIC0 = 0x42;
const auto MAGIC1 = 0x4D;
}

namespace Pms5003 {

Sensor::Sensor(const uart_port_t port) :
        port{port}
{
    this->uart_buffer = new uint8_t[32];
    if (!this->uart_buffer)
        throw std::runtime_error("could not allocate buffer");
}

Sensor::~Sensor() noexcept
{
    delete[] this->uart_buffer;
}

SensorData Sensor::read_data() noexcept
{
    SensorData data{};
    data.pm10_standard = -1;
    data.pm25_standard = -1;
    data.pm100_standard = -1;
    data.pm10_env = -1;
    data.pm25_env = -1;
    data.pm100_env = -1;
    data.particles_03 = -1;
    data.particles_05 = -1;
    data.particles_10 = -1;
    data.particles_25 = -1;
    data.particles_50 = -1;
    data.particles_100 = -1;
    data.error = ERROR_NONE;
    data.time_to_read = 0;

    const auto then = now_millis();

    uint64_t err;

    err = this->read_frame();
    if (err != ERROR_NONE) {
        data.error = err;
        data.time_to_read = now_millis() - then;
        return data;
    }

    if (!this->checksum_check()) {
        ESP_LOGE(TAG, "corrupt frame, invalid checksum");
        data.error = ERROR_BAD_CHECKSUM;
        data.time_to_read = now_millis() - then;
        return data;
    }

    this->put_buffer_into_data(data);
    data.time_to_read = now_millis() - then;
    return data;
}

// Haha, I'm going to use goto all over the place.
// TODO This could probably use a ring buffer.
uint64_t Sensor::read_frame() noexcept
{
    size_t loop = 0;

    start_over:
    ESP_LOGV(TAG, "trying to receive from sensor, try=%d", loop);

    for (size_t i = 0; i < 32; i++)
        this->uart_buffer[i] = 0;

    const auto err = uart_flush_input(this->port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to flush buffer");
        return ERROR_UART_BUFFER_FLUSH;
    }

    size_t remaining = 32;
    while (this->uart_buffer[0] != MAGIC0 || this->uart_buffer[1] != MAGIC1 || remaining) {
        loop++;
        if (loop > MAX_RETRIES) {
            ESP_LOGE(TAG, "attempts exhausted while trying to read from sensor");
            return ERROR_READ_ATTEMPTS_EXHAUSTED;
        }

        const auto from = 32 - remaining;
        const auto len = uart_read_bytes(this->port,
                                         this->uart_buffer + from,
                                         remaining,
                                         UART_READ_WAIT_TICKS);
        if (len < 0) {
            ESP_LOGE(TAG, "uart read error: %d", len);
            return ERROR_UART_READ;
        }

        bool found_magic = false;
        size_t magic_index = 0;
        for (; magic_index < 31; magic_index++)
            if (this->uart_buffer[magic_index] == MAGIC0 &&
                this->uart_buffer[magic_index + 1] == MAGIC1) {
                found_magic = true;
                break;
            }
        if (!found_magic)
            goto start_over;

        if (magic_index > 0)
            for (size_t i = magic_index; i < 32; i++)
                this->uart_buffer[i - magic_index] = this->uart_buffer[i];

        remaining -= len;
    }

    if (this->uart_buffer[0] != MAGIC0 || this->uart_buffer[1] != MAGIC1) {
        ESP_LOGE(TAG, "wrong magic0, expecting=%02x-%02x got=%02x-%02x",
                 MAGIC0,
                 MAGIC1,
                 this->uart_buffer[0],
                 this->uart_buffer[1]);
        return ERROR_BAD_MAGIC_VALUE;
    }

    return ERROR_NONE;
}

void Sensor::put_buffer_into_data(SensorData& data) const noexcept
{
    data.pm10_standard = this->uart_buffer[4] * 256 + this->uart_buffer[5];
    data.pm25_standard = this->uart_buffer[6] * 256 + this->uart_buffer[7];
    data.pm100_standard = this->uart_buffer[8] * 256 + this->uart_buffer[9];

    data.pm10_env = this->uart_buffer[10] * 256 + this->uart_buffer[11];
    data.pm25_env = this->uart_buffer[12] * 256 + this->uart_buffer[13];
    data.pm100_env = this->uart_buffer[14] * 256 + this->uart_buffer[15];

    data.particles_03 = this->uart_buffer[16] * 256 + this->uart_buffer[17];
    data.particles_05 = this->uart_buffer[18] * 256 + this->uart_buffer[19];
    data.particles_10 = this->uart_buffer[20] * 256 + this->uart_buffer[21];
    data.particles_25 = this->uart_buffer[22] * 256 + this->uart_buffer[23];
    data.particles_50 = this->uart_buffer[24] * 256 + this->uart_buffer[25];
    data.particles_100 = this->uart_buffer[26] * 256 + this->uart_buffer[27];
}

bool Sensor::checksum_check() const noexcept
{
    const uint16_t checksum = this->uart_buffer[30] * 256 + this->uart_buffer[31];

    uint16_t sum = 0;
    for (size_t i = 0; i < 30; i++)
        sum += this->uart_buffer[i];

    if (sum != checksum)
        ESP_LOGE(TAG, "sum=%d, checksum=%d", sum, checksum);

    return sum == checksum;
}


}

}
