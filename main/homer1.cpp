#include <iostream>
#include <cstdint>

#include "esp_log.h"

#include "my_init.hpp"
#include "homer_util.hpp"
#include "s8.hpp"
#include "pms5003.hpp"
#include "bmp180.hpp"
#include "sht3x.hpp"

using namespace homer1;

using std::uint32_t;


namespace {

const char* MY_TAG = "homer1";
const uint32_t MEASUREMENT_DELAY = 1000;
const uint32_t PRINT_DELAY = 1000;


class SensorData final
{
public:

    Pms5003::SensorData pms5003;
    Bmp180::SensorData bmp180;
    S8::SensorData s8;
    Sht3x::SensorData sht3x;


    SensorData(const SensorData& other) = delete;

    SensorData& operator=(const SensorData& other) = delete;

    SensorData& operator=(SensorData&& other) = delete;

    SensorData(SensorData&& other) = delete;

    SensorData() noexcept:
            pms5003{},
            bmp180{},
            s8{},
            sht3x{}
    {
    }
};

class SensorPeripheral final
{
public:

    S8::Sensor s8;
    Pms5003::Sensor pms5003;
    Bmp180::Sensor bmp180;
    Sht3x::Sensor sht3x;

    SensorPeripheral(const SensorPeripheral& other) = delete;

    SensorPeripheral& operator=(const SensorPeripheral& other) = delete;

    SensorPeripheral& operator=(SensorPeripheral&& other) = delete;

    SensorPeripheral(SensorPeripheral&& other) = delete;
};

class Sensor final
{
public:
    SensorData data;
    SensorPeripheral peripheral;
    SemaphoreHandle_t mutex;
    volatile bool loop;


    Sensor(const Sensor& other) = delete;

    Sensor& operator=(const Sensor& other) = delete;

    Sensor& operator=(Sensor&& other) = delete;

    Sensor(Sensor&& other) = delete;


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
        if (this->data.pms5003.has_data()) {
            ss << "[PMS5003]" << std::endl;
            this->data.pms5003.dump(ss);
        }
        if (this->data.bmp180.has_data()) {
            ss << std::endl << "[BMP180]" << std::endl;
            this->data.bmp180.dump(ss);
        }
        if (this->data.s8.has_data()) {
            ss << std::endl << "[S8]" << std::endl;
            this->data.s8.dump(ss);
        }
        if (this->data.sht3x.has_data()) {
            ss << std::endl << "[SHT3X]" << std::endl;
            this->data.sht3x.dump(ss);
        }
        this->unlock();
    }

private:
    void lock() noexcept
    {
        xSemaphoreTake(this->mutex, portMAX_DELAY);
    }

    void unlock() noexcept
    {
        xSemaphoreGive(this->mutex);
    }
};


Sensor* make_sensor()
{
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
            .mutex{},
            .loop = true,
    };
    sensor->mutex = xSemaphoreCreateMutex();
    if (!sensor->mutex) {
        ESP_LOGE(MY_TAG, "could not allocate mutex");
        throw std::runtime_error("could not allocate mutex");
    }

    return sensor;
}

void read_sensors(Sensor* sensor) noexcept
{
    xTaskCreate(
            [](void* arg) {
                auto* sensor0 = static_cast<Sensor*>(arg);
                do {
                    sensor0->update_bmp180();
                    my_sleep_millis(MEASUREMENT_DELAY);
                } while (sensor0->loop);
            },
            "work_read_bmp180",
            1024 * 3,
            sensor,
            10,
            nullptr
    );
    xTaskCreate(
            [](void* arg) {
                auto* sensor0 = static_cast<Sensor*>(arg);
                do {
                    sensor0->update_pms5003();
                    my_sleep_millis(MEASUREMENT_DELAY);
                } while (sensor0->loop);
            },
            "work_read_pms5003",
            1024,
            sensor,
            10,
            nullptr
    );
    xTaskCreate(
            // work_read_s8,
            [](void* arg) {
                auto* sensor0 = static_cast<Sensor*>(arg);
                do {
                    sensor0->update_s8();
                    my_sleep_millis(MEASUREMENT_DELAY);
                } while (sensor0->loop);
            },
            "work_read_s8",
            1024,
            sensor,
            10,
            nullptr
    );
    xTaskCreate(
            [](void* arg) {
                auto* sensor0 = static_cast<Sensor*>(arg);
                do {
                    sensor0->update_sht3x();
                    my_sleep_millis(MEASUREMENT_DELAY);
                } while (sensor0->loop);
            },
            "work_read_sht3x",
            1024,
            sensor,
            10,
            nullptr
    );
}

void dump_sensors(Sensor* sensor) noexcept
{
    xTaskCreate(
            [](void* arg) {
                auto* sensor0 = static_cast<Sensor*>(arg);
                do {
                    std::stringstream ss;
                    print_sensor_dump_header(ss);
                    sensor0->dump_all(ss);
                    std::cout << ss.str();
                    my_sleep_millis(PRINT_DELAY);
                } while (sensor0->loop);
            },
            "work_dump_all",
            1024 * 4,
            sensor,
            10,
            nullptr
    );
}

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

    auto sensor = make_sensor();

    read_sensors(sensor);
    dump_sensors(sensor);
}
