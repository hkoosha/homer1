#include "esp_log.h"
#include <cstring>
#include <cstddef>
#include <cstdint>


#include "my_s8.hpp"


using std::size_t;
using std::uint8_t;
using std::uint16_t;


// =============================================================================

static const char* TAG = "MyS8";

static const unsigned char ID_HI[]   = {0xFE, 0x04, 0x00, 0x1D, 0x00, 0x01, 0xB5, 0xC3};
static const unsigned char ID_LO[]   = {0xFE, 0x04, 0x00, 0x1E, 0x00, 0x01, 0x45, 0xC3};
static const unsigned char ABC_REQ[] = {0xFE, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xA1, 0XC3};
static const unsigned char FW_REQ[]  = {0xFE, 0x04, 0x00, 0x1C, 0x00, 0x01, 0xE4, 0x03};
static const unsigned char CO2_REQ[] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};

static const auto UART_READ_WAIT = 500;



// https://github.com/SFeli/ESP32_S8/blob/master/ESP32_S8_01.ino
static uint16_t modbus_crc(const char* const buf, 
                           const size_t len)
{
  uint16_t crc = 0xFFFF;

  for (size_t pos = 0; pos < len; pos++) {
    crc ^= static_cast<uint16_t>(buf[pos]);         // XOR byte into least sig. byte of crc
    for (uint8_t i = 8; i > 0; i--) {   // Loop over each bit
      if ((crc & 0x0001) != 0) {        // If the LSB is set
        crc >>= 1;                      // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else {                            // else LSB is not set
        crc >>= 1;                      // Just shift right
      }
    }
  }  

  // Note, this number has low and high bytes swapped,
  // so use it accordingly (or swap bytes)
  return crc;  
}


// =============================================================================

MyS8::MyS8(const uart_port_t port) noexcept: 
    port { port }
{
    this->uart_buffer = new char[8];
    assert(this->uart_buffer);
}

MyS8::~MyS8() noexcept
{
    delete[] this->uart_buffer;
}


// =============================================================================

int16_t MyS8::get_co2() noexcept
{
    const auto value = this->read_reg(CO2_REQ);

    if(value < 0)
        ESP_LOGE(TAG, "error reading co2");
    
    return value;
}

int16_t MyS8::get_abc_days() noexcept
{
    const auto value = this->read_reg(ABC_REQ);

    if(value < 0)
        ESP_LOGE(TAG, "error reading abc");
    
    return value;
}

int32_t MyS8::get_sensor_id() noexcept
{
    const auto id_hi = this->read_reg(ID_HI);
    if(id_hi < 0) {
        ESP_LOGE(TAG, "error reading id_hi");
        return id_hi;
    }

    const auto id_lo = this->read_reg(ID_LO);
    if(id_lo < 0) {
        ESP_LOGE(TAG, "error reading id_lo");
        return id_lo;
    }

    const auto hi = static_cast<int32_t>(id_hi);
    const auto lo = static_cast<int32_t>(id_lo);
    return hi * 256 * 256 + lo;
}

int16_t MyS8::get_sensor_fw() noexcept
{
    const auto value = this->read_reg(FW_REQ);

    if(value < 0)
        ESP_LOGE(TAG, "error reading fw");
    
    return value;
}


int16_t MyS8::read_reg(const unsigned char* data) noexcept {
    if(!this->read_and_write_8_bytes(data))
        return -1;

    const auto hi = static_cast<uint16_t>(this->uart_buffer[3]);
    const auto lo = static_cast<uint16_t>(this->uart_buffer[4]);
    const auto co2 = hi * 256 + lo;
    return static_cast<int16_t>(co2);
}

bool MyS8::read_and_write_8_bytes(const unsigned char* data) noexcept {
    if(!this->write_8_bytes(data))
        return false;

    if(!this->read_7_bytes())
        return false;
    
    return true;
}

bool MyS8::write_8_bytes(const unsigned char* data) const noexcept 
{
    const auto len = uart_write_bytes(this->port, data, 8);

    if(len < 0) {
        ESP_LOGE(TAG, "uart write error, code=%d", len);
        return false;
    }
    else if (len > 8) {
        ESP_LOGE(TAG, "uart write assetion error, too much data written, expected=8, written=%d", len);
        return false;
    }
    else if (len != 8) {
        ESP_LOGE(TAG, "uart write error, could not write all data, expected to write=8, written=%d", len);
        return false;
    }
    else {
        ESP_LOGV(TAG, "wrote 8 bytes to uart");
        return true;
    }
}

bool MyS8::read_7_bytes() noexcept 
{
    std::memset(this->uart_buffer, 0, 8);

    const auto len = uart_read_bytes(this->port, this->uart_buffer, 8, UART_READ_WAIT / portTICK_RATE_MS);

    if (len < 0) {
        ESP_LOGE(TAG, "uart read error, code=%d", len);
        return false;
    }
    else if(len > 7) {
        ESP_LOGE(TAG, "too much data in uart, by now memory is corrupt.");
        return false;
    }
    else if(len != 7) {
        ESP_LOGE(TAG, "uart read error, could not read all data, expected to read=7, actual read=%d", len);
        return false;
    }

    const auto crc_hi = static_cast<uint16_t>(this->uart_buffer[6]) * 256;
    const auto crc_lo = static_cast<uint16_t>(this->uart_buffer[5]);
    const auto read_crc = crc_hi + crc_lo;
    const auto calc_crc = modbus_crc(this->uart_buffer, 5);

    if(read_crc != calc_crc) {
        ESP_LOGE(TAG, "uart read error, crc mismatch.");
        return false;
    }

    return true;
}
