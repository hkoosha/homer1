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

    bool has_pms5003;
    bool has_bmp180;
    bool has_s8;
    bool has_sht3x;
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
};


[[noreturn]] void work_read_all(void* arg)
{
    auto* sensor = static_cast<Sensor*>(arg);
    do {
        if (sensor->peripheral.bmp180.is_initialized() || (sensor->peripheral.bmp180.init() == Bmp180::ERROR_NONE)) {
            sensor->data.bmp180 = sensor->peripheral.bmp180.read_data(Bmp180::SEA_LEVEL_PRESSURE);
            sensor->data.has_bmp180 = true;
        }
        else {
            sensor->data.has_bmp180 = false;
        }

        sensor->data.s8 = sensor->peripheral.s8.read_data();
        sensor->data.has_s8 = true;

        sensor->data.pms5003 = sensor->peripheral.pms5003.read_data();
        sensor->data.has_pms5003 = true;

        sensor->data.sht3x = sensor->peripheral.sht3x.read_data();
        sensor->data.has_sht3x = true;

        my_sleep_millis(MEASUREMENT_DELAY);
    } while (true);
}


[[noreturn]] void work_read_s8(void* arg)
{
    auto* sensor = static_cast<Sensor*>(arg);

    do {
        const auto data = sensor->peripheral.s8.read_data();

        xSemaphoreTake(sensor->mutex, portMAX_DELAY);
        sensor->data.s8 = data;
        sensor->data.has_s8 = true;
        xSemaphoreGive(sensor->mutex);

        my_sleep_millis(MEASUREMENT_DELAY);
    } while (true);
}

[[noreturn]] void work_read_sht3x(void* arg)
{
    auto* sensor = static_cast<Sensor*>(arg);

    do {
        const auto data = sensor->peripheral.sht3x.read_data();

        xSemaphoreTake(sensor->mutex, portMAX_DELAY);
        sensor->data.sht3x = data;
        sensor->data.has_sht3x = true;
        xSemaphoreGive(sensor->mutex);

        my_sleep_millis(MEASUREMENT_DELAY);
    } while (true);
}

[[noreturn]] void work_read_pms5003(void* arg)
{
    auto* sensor = static_cast<Sensor*>(arg);

    do {
        const auto data = sensor->peripheral.pms5003.read_data();

        xSemaphoreTake(sensor->mutex, portMAX_DELAY);
        sensor->data.pms5003 = data;
        sensor->data.has_pms5003 = true;
        xSemaphoreGive(sensor->mutex);

        my_sleep_millis(MEASUREMENT_DELAY);
    } while (true);
}

[[noreturn]] void work_read_bmp180(void* arg)
{
    auto* sensor = static_cast<Sensor*>(arg);

    do {
        bool has_data;
        Bmp180::SensorData data{};

        if (sensor->peripheral.bmp180.is_initialized() || (sensor->peripheral.bmp180.init() == Bmp180::ERROR_NONE)) {
            data = sensor->peripheral.bmp180.read_data(Bmp180::SEA_LEVEL_PRESSURE);
            has_data = true;
        }
        else {
            has_data = false;
        }

        xSemaphoreTake(sensor->mutex, portMAX_DELAY);
        sensor->data.bmp180 = data;
        sensor->data.has_bmp180 = has_data;
        xSemaphoreGive(sensor->mutex);

        my_sleep_millis(MEASUREMENT_DELAY);
    } while (true);
}


[[noreturn]] void work_dump_all(void* arg)
{
    auto* sensor = static_cast<Sensor*>(arg);
    do {
        xSemaphoreTake(sensor->mutex, portMAX_DELAY);
        const auto pms5003 = Pms5003::copy(sensor->data.pms5003);
        const auto bmp180 = Bmp180::copy(sensor->data.bmp180);
        const auto s8 = S8::copy(sensor->data.s8);
        const auto sht3x = Sht3x::copy(sensor->data.sht3x);
        const auto has_pms5003 = sensor->data.has_pms5003;
        const auto has_bmp180 = sensor->data.has_bmp180;
        const auto has_s8 = sensor->data.has_s8;
        const auto has_sht3x = sensor->data.has_sht3x;
        xSemaphoreGive(sensor->mutex);

        std::stringstream ss;
        print_sensor_dump_header(ss);

        if (has_bmp180)
            Bmp180::dump(bmp180, ss);

        ss << std::endl;
        if (has_s8)
            S8::dump(s8, ss);

        ss << std::endl;
        if (has_pms5003)
            Pms5003::dump(pms5003, ss);

        ss << std::endl;
        if (has_sht3x)
            Sht3x::dump(sht3x, ss);

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
                            Sht3x::ADDR_1,
                            Sht3x::DELAY
                    }}}
            },
            .mutex{}
    };
    sensor->data.has_bmp180 = false;
    sensor->data.has_sht3x = false;
    sensor->data.has_pms5003 = false;
    sensor->data.has_s8 = false;
    sensor->mutex = xSemaphoreCreateMutex();
    if(!sensor->mutex)
        throw std::runtime_error("could not allocate mutex");

    // xTaskCreate(
    //         work_read_all,
    //         "work_read_all",
    //         1024 * 5,
    //         sensor,
    //         10,
    //         nullptr
    // );
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
