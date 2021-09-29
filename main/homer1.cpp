#include <iostream>
#include <cstdint>
#include <map>
#include <vector>
#include <algorithm>

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


    SensorData& operator=(const SensorData& other) = delete;

    SensorData& operator=(SensorData&& other) = delete;

    SensorData(const SensorData& other) = delete;


    SensorData(SensorData&& other) noexcept:
            pms5003{std::move(other.pms5003)},
            bmp180{std::move(other.bmp180)},
            s8{std::move(other.s8)},
            sht3x{std::move(other.sht3x)}
    {
    }

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


    SensorPeripheral& operator=(const SensorPeripheral& other) = delete;

    SensorPeripheral& operator=(SensorPeripheral&& other) = delete;

    SensorPeripheral(const SensorPeripheral& other) = delete;


    SensorPeripheral(SensorPeripheral&& other) noexcept:
            s8{std::move(other.s8)},
            pms5003{std::move(other.pms5003)},
            bmp180{std::move(other.bmp180)},
            sht3x{std::move(other.sht3x)}
    {
    }

    SensorPeripheral(S8::Sensor&& s8,
                     Pms5003::Sensor&& pms5003,
                     Bmp180::Sensor&& bmp180,
                     Sht3x::Sensor&& sht3x) noexcept:
            s8{std::move(s8)},
            pms5003{std::move(pms5003)},
            bmp180{std::move(bmp180)},
            sht3x{std::move(sht3x)}
    {
    }
};

class Sensor final
{
public:
    SensorData data;
    SensorPeripheral peripheral;
    SemaphoreHandle_t mutex;
    volatile bool loop;


    Sensor& operator=(const Sensor& other) = delete;

    Sensor& operator=(Sensor&& other) = delete;

    Sensor(const Sensor& other) = delete;

    Sensor(Sensor&& other) = delete;


    Sensor(SensorData&& data,
           SensorPeripheral&& peripheral,
           SemaphoreHandle_t&& mutex) noexcept:
            data{std::move(data)},
            peripheral{std::move(peripheral)},
            mutex{mutex},
            loop{true}
    {
    }


    void update_s8() noexcept
    {
        const auto& new_data = this->peripheral.s8.read_data();

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
        if (this->peripheral.bmp180.is_initialized() || this->peripheral.bmp180.init().is_ok())
            new_data = this->peripheral.bmp180.read_data();

        this->lock();
        this->data.bmp180 = new_data;
        this->unlock();
    }


    void dump_pretty_print(std::stringstream& ss) noexcept
    {
        this->lock();

        ss << "[PMS5003]" << std::endl;
        if (this->data.pms5003.get_error().is_ok()) {
            this->data.pms5003.dump(ss);
        }
        else {
            ss << "HW: " << this->data.pms5003.hw_err_to_str() << std::endl;
            ss << "SN: " << this->data.pms5003.sensor_err_to_str() << std::endl;
        }

        ss << std::endl << "[BMP180]" << std::endl;
        if (this->data.bmp180.get_error().is_ok()) {
            this->data.bmp180.dump(ss);
        }
        else {
            ss << "HW: " << this->data.bmp180.hw_err_to_str() << std::endl;
            ss << "SN: " << this->data.bmp180.sensor_err_to_str() << std::endl;
        }

        ss << std::endl << "[S8]" << std::endl;
        if (this->data.s8.get_error().is_ok()) {
            this->data.s8.dump(ss);
        }
        else {
            ss << "HW: " << this->data.s8.hw_err_to_str() << std::endl;
            ss << "SN: " << this->data.s8.sensor_err_to_str() << std::endl;
        }

        ss << std::endl << "[SHT3X]" << std::endl;
        if (this->data.sht3x.get_error().is_ok()) {
            this->data.sht3x.dump(ss);
        }
        else {
            ss << "HW: " << this->data.sht3x.hw_err_to_str() << std::endl;
            ss << "SN: " << this->data.sht3x.sensor_err_to_str() << std::endl;
        }

        this->unlock();
    }

    void dump_map(std::stringstream& ss) noexcept
    {
        this->lock();
        const HomerSensorDump pms5003_dump = this->data.pms5003.dump();
        const HomerSensorDump bmp180_dump = this->data.bmp180.dump();
        const HomerSensorDump s8_dump = this->data.s8.dump();
        const HomerSensorDump sht3x_dump = this->data.sht3x.dump();
        this->unlock();

        ss << "[PMS5003]" << std::endl;
        for (const auto& item: pms5003_dump)
            ss << item.first << "=" << item.second << std::endl;

        ss << "[BMP180]" << std::endl;
        for (const auto& item: bmp180_dump)
            ss << item.first << "=" << item.second << std::endl;

        ss << "[S8]" << std::endl;
        for (const auto& item: s8_dump)
            ss << item.first << "=" << item.second << std::endl;

        ss << "[SHT3X]" << std::endl;
        for (const auto& item: sht3x_dump)
            ss << item.first << "=" << item.second << std::endl;
    }

    void dump_influxdb(std::stringstream& ss) noexcept
    {
        std::vector<std::string> measurements{};
        this->lock();
        this->data.pms5003.influxdb(measurements);
        this->data.bmp180.influxdb(measurements);
        this->data.s8.influxdb(measurements);
        this->data.sht3x.influxdb(measurements);
        this->unlock();

        std::sort(measurements.begin(), measurements.end());

        for (const auto& item: measurements)
            ss << item << std::endl;
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
            {},
            SensorPeripheral{
                    {S8::Sensor{UART_NUM_2}},
                    {Pms5003::Sensor{UART_NUM_1}},
                    {Bmp180::Sensor{
                            i2c::Device{
                                    I2C_NUM_0,
                                    Bmp180::I2C_ADDR,
                                    Bmp180::I2C_DELAY
                            },
                            Bmp180::SEA_LEVEL_PRESSURE,
                            Bmp180::OVERSAMPLING_ULTRA_HIGH_RES
                    }},
                    {Sht3x::Sensor{
                            i2c::Device{
                                    I2C_NUM_1,
                                    Sht3x::I2C_ADDR,
                                    Sht3x::I2C_DELAY
                            }
                    }}
            },
            nullptr
    };

    if (!sensor) {
        ESP_LOGE(MY_TAG, "could not allocate sensor");
        throw std::runtime_error("could not allocate sensor");
    }

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
                    sensor0->update_s8();
                    my_sleep_millis(MEASUREMENT_DELAY);
                } while (sensor0->loop);
            },
            "work_read_s8",
            1024 * 4,
            sensor,
            10,
            nullptr
    );
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
            1024 * 3,
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
            1024 * 3,
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
                    sensor0->dump_influxdb(ss);
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
