#include <iostream>
#include <cstdint>

#include "esp_log.h"

#include "my_init.hpp"
#include "util.hpp"
#include "s8.hpp"
#include "pms5003.hpp"
#include "bmp180.hpp"
#include "sht3x.hpp"

using namespace homer1;

using std::uint32_t;

static const char* MY_TAG = "homer1";
static const uint32_t MEASUREMENT_DELAY = 1000;
static const uint32_t PRINT_DELAY = 500;


struct SensorData final
{
    Pms5003::SensorData pms5003;
    Bmp180::SensorData bmp180;
    S8::SensorData s8;
    Sht3x::SensorData sht3x;

    SensorData() noexcept:
            pms5003{},
            bmp180{},
            s8{},
            sht3x{}
    {
    }
};

struct SensorPeripheral final
{
    S8::Sensor s8;
    Pms5003::Sensor pms5003;
    Bmp180::Sensor bmp180;
    Sht3x::Sensor sht3x;
};

struct Sensor final
{
    SensorData data;
    SensorPeripheral peripheral;
    SemaphoreHandle_t mutex;

    void lock() noexcept
    {
        xSemaphoreTake(this->mutex, portMAX_DELAY);
    }

    void unlock() noexcept
    {
        xSemaphoreGive(this->mutex);
    }

    void update_s8() noexcept
    {
        const auto new_data = this->peripheral.s8.read_data();

        this->lock();
        this->data.s8 = new_data;
        this->unlock();
    }

    void update_sht3x() noexcept
    {
        const auto new_data = this->peripheral.sht3x.read_data();

        this->lock();
        this->data.sht3x = new_data;
        this->unlock();
    }

    void update_pms5003() noexcept
    {
        const auto new_data = this->peripheral.pms5003.read_data();

        this->lock();
        this->data.pms5003 = new_data;
        this->unlock();
    }

    void update_bmp180() noexcept
    {
        Bmp180::SensorData new_data{};
        if (this->peripheral.bmp180.is_initialized() || (this->peripheral.bmp180.init() == Bmp180::ERROR_NONE))
            new_data = this->peripheral.bmp180.read_data();

        this->lock();
        this->data.bmp180 = new_data;
        this->unlock();
    }

    void dump_all(std::stringstream& ss) noexcept
    {
        this->lock();
        if (this->data.pms5003.has_data())
            this->data.pms5003.dump(ss);
        ss << std::endl;
        if (this->data.bmp180.has_data())
            this->data.bmp180.dump(ss);
        ss << std::endl;
        if (this->data.s8.has_data())
            this->data.s8.dump(ss);
        ss << std::endl;
        if (this->data.sht3x.has_data())
            this->data.sht3x.dump(ss);
        this->unlock();
    }
};


[[noreturn]] void work_read_s8(void* arg)
{
    auto* sensor = static_cast<Sensor*>(arg);

    do {
        sensor->update_s8();
        my_sleep_millis(MEASUREMENT_DELAY);
    } while (true);
}

[[noreturn]] void work_read_sht3x(void* arg)
{
    auto* sensor = static_cast<Sensor*>(arg);

    do {
        sensor->update_sht3x();
        my_sleep_millis(MEASUREMENT_DELAY);
    } while (true);
}

[[noreturn]] void work_read_pms5003(void* arg)
{
    auto* sensor = static_cast<Sensor*>(arg);

    do {
        sensor->update_pms5003();
        my_sleep_millis(MEASUREMENT_DELAY);
    } while (true);
}

[[noreturn]] void work_read_bmp180(void* arg)
{
    auto* sensor = static_cast<Sensor*>(arg);

    do {
        sensor->update_bmp180();
        my_sleep_millis(MEASUREMENT_DELAY);
    } while (true);
}

[[noreturn]] void work_dump_all(void* arg)
{
    auto* sensor = static_cast<Sensor*>(arg);

    do {
        std::stringstream ss;
        print_sensor_dump_header(ss);
        sensor->dump_all(ss);
        std::cout << ss.str();
        my_sleep_millis(PRINT_DELAY);
    } while (true);
}


extern "C" void app_main(void)
{
    ESP_LOGI(MY_TAG, "init...");
    my_nvs_init();
    // my_wifi_init(CONFIG_MY_WIFI_SSID, CONFIG_MY_WIFI_PASSWORD);
    my_uart_init_8n1(UART_NUM_1, 25, 26);
    my_uart_init_8n1(UART_NUM_2, 13, 27);
    my_i2c_init(I2C_NUM_0, 33, 32);
    my_i2c_init(I2C_NUM_1, 17, 18);

    auto sensor = new Sensor{
            .data{},
            .peripheral{
                    .s8 {S8::Sensor{UART_NUM_2}},
                    .pms5003 {Pms5003::Sensor{UART_NUM_1}},
                    .bmp180 {Bmp180::Sensor{i2c::Device{
                            I2C_NUM_0,
                            Bmp180::I2C_ADDR,
                            Bmp180::I2C_DELAY
                    }}},
                    .sht3x {Sht3x::Sensor{i2c::Device{
                            I2C_NUM_1,
                            Sht3x::I2C_ADDR,
                            Sht3x::I2C_DELAY
                    }}}
            },
            .mutex{}
    };
    sensor->mutex = xSemaphoreCreateMutex();
    if (!sensor->mutex)
        throw std::runtime_error("could not allocate mutex");

    xTaskCreate(
            work_read_bmp180,
            "work_read_bmp180",
            1024 * 3,
            sensor,
            10,
            nullptr
    );
    xTaskCreate(
            work_read_pms5003,
            "work_read_pms5003",
            1024,
            sensor,
            10,
            nullptr
    );
    xTaskCreate(
            work_read_s8,
            "work_read_s8",
            1024,
            sensor,
            10,
            nullptr
    );
    xTaskCreate(
            work_read_sht3x,
            "work_read_sht3x",
            1024,
            sensor,
            10,
            nullptr
    );
    xTaskCreate(
            work_dump_all,
            "work_dump_all",
            1024 * 4,
            sensor,
            10,
            nullptr
    );
}
