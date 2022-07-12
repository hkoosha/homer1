#include <iostream>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>
#include <algorithm>
#include <string>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"

#include "homer_util.hpp"
#include "s8.hpp"
#include "pms5003.hpp"
#include "bmp180.hpp"
#include "sht3x.hpp"
#include "homer_helper.c"
#include "homer1.hpp"

using namespace homer1;

using std::uint16_t;
using std::uint32_t;

using namespace homer1cfg;

// SENSORS
namespace {

const char* const MY_TAG = "homer1";
const char* const PUSHER_TAG = "influx_pusher";

class Sensor;

httpd_handle_t prometheus_http_server(Sensor* sensor);


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
    volatile bool loop{true};

    Sensor& operator=(const Sensor& other) = delete;

    Sensor& operator=(Sensor&& other) = delete;

    Sensor(const Sensor& other) = delete;

    Sensor(Sensor&& other) = delete;

    ~Sensor() noexcept
    {
        const auto err = httpd_stop(this->prometheus_server);
        if (err != ESP_OK)
            ESP_LOGE(MY_TAG, "failed to close httpd server: %s", esp_err_to_name(err));
        this->prometheus_server = nullptr;
    }


    Sensor(SensorData&& data,
           SensorPeripheral&& peripheral) :
            data{std::move(data)},
            peripheral{std::move(peripheral)}
    {
        this->mutex = xSemaphoreCreateMutex();
        if (!this->mutex)
            throw std::runtime_error("could not allocate mutex");

        if (my_is_wifi_enabled() && my_is_prometheus_enabled()) {
            this->prometheus_server = prometheus_http_server(this);
            if (!this->prometheus_server)
                throw std::runtime_error("could not create server");
        }
        else {
            ESP_LOGW(MY_TAG, "Prometheus endpoint disabled.");
        }
    }


    void update_s8() noexcept
    {
        assert(my_is_s8_enabled());

        const auto& new_data = this->peripheral.s8.read_data();

        this->lock();
        this->data.s8 = new_data;
        this->unlock();
    }

    void update_sht3x() noexcept
    {
        assert(my_is_sht3x_enabled());

        const auto new_data = this->peripheral.sht3x.read_data();

        this->lock();
        this->data.sht3x = new_data;
        this->unlock();
    }

    void update_pms5003() noexcept
    {
        assert(my_is_pms5003_enabled());

        const auto new_data = this->peripheral.pms5003.read_data();

        this->lock();
        this->data.pms5003 = new_data;
        this->unlock();
    }

    void update_bmp180() noexcept
    {
        assert(my_is_bmp180_enabled());

        Bmp180::SensorData new_data{};
        if (this->peripheral.bmp180.is_initialized() || this->peripheral.bmp180.init().is_ok())
            new_data = this->peripheral.bmp180.read_data();

        this->lock();
        this->data.bmp180 = new_data;
        this->unlock();
    }


    void dump_pretty_print(std::stringstream& ss) const noexcept
    {
        this->lock();

        if (my_is_pms5003_enabled()) {
            ss << "[PMS5003]" << std::endl;
            if (this->data.pms5003.get_error().is_ok()) {
                this->data.pms5003.dump(ss);
            }
            else {
                ss << "HW: " << this->data.pms5003.hw_err_to_str() << std::endl;
                ss << "SN: " << this->data.pms5003.sensor_err_to_str() << std::endl;
            }
        }
        if (my_is_bmp180_enabled()) {
            ss << std::endl << "[BMP180]" << std::endl;
            if (this->data.bmp180.get_error().is_ok()) {
                this->data.bmp180.dump(ss);
            }
            else {
                ss << "HW: " << this->data.bmp180.hw_err_to_str() << std::endl;
                ss << "SN: " << this->data.bmp180.sensor_err_to_str() << std::endl;
            }
        }
        if (my_is_s8_enabled()) {
            ss << std::endl << "[S8]" << std::endl;
            if (this->data.s8.get_error().is_ok()) {
                this->data.s8.dump(ss);
            }
            else {
                ss << "HW: " << this->data.s8.hw_err_to_str() << std::endl;
                ss << "SN: " << this->data.s8.sensor_err_to_str() << std::endl;
            }
        }
        if (my_is_sht3x_enabled()) {
            ss << std::endl << "[SHT3X]" << std::endl;
            if (this->data.sht3x.get_error().is_ok()) {
                this->data.sht3x.dump(ss);
            }
            else {
                ss << "HW: " << this->data.sht3x.hw_err_to_str() << std::endl;
                ss << "SN: " << this->data.sht3x.sensor_err_to_str() << std::endl;
            }
        }

        this->unlock();
    }

    __attribute__((unused)) void dump_map(std::stringstream& ss) const noexcept
    {
        this->lock();

        if (my_is_pms5003_enabled()) {
            ss << "[PMS5003]" << std::endl;
            const HomerSensorDump pms5003_dump = this->data.pms5003.dump();
            for (const auto& item: pms5003_dump)
                ss << item.first << "=" << item.second << std::endl;
        }
        if (my_is_bmp180_enabled()) {
            ss << "[BMP180]" << std::endl;
            const HomerSensorDump bmp180_dump = this->data.bmp180.dump();
            for (const auto& item: bmp180_dump)
                ss << item.first << "=" << item.second << std::endl;
        }
        if (my_is_s8_enabled()) {
            ss << "[S8]" << std::endl;
            const HomerSensorDump s8_dump = this->data.s8.dump();
            for (const auto& item: s8_dump)
                ss << item.first << "=" << item.second << std::endl;
        }
        if (my_is_sht3x_enabled()) {
            ss << "[SHT3X]" << std::endl;
            const HomerSensorDump sht3x_dump = this->data.sht3x.dump();
            for (const auto& item: sht3x_dump)
                ss << item.first << "=" << item.second << std::endl;
        }

        this->unlock();
    }

    void dump_influxdb(std::stringstream& ss) const noexcept
    {
        std::vector<std::string> measurements{};

        this->lock();
        if (my_is_pms5003_enabled())
            this->data.pms5003.influxdb(measurements);
        if (my_is_bmp180_enabled())
            this->data.bmp180.influxdb(measurements);
        if (my_is_s8_enabled())
            this->data.s8.influxdb(measurements);
        if (my_is_sht3x_enabled())
            this->data.sht3x.influxdb(measurements);
        this->unlock();

        // Not needed for InfluxDB but makes cURLing it user-friendly.
        std::sort(measurements.begin(), measurements.end());

        for (const auto& item: measurements)
            ss << item << std::endl;
    }

    [[nodiscard]] std::stringstream dump_prometheus() const noexcept
    {
        std::stringstream ss{};

        this->lock();

        if (my_is_pms5003_enabled())
            this->data.pms5003.prometheus(ss);
        if (my_is_bmp180_enabled())
            this->data.bmp180.prometheus(ss);
        if (my_is_s8_enabled())
            this->data.s8.prometheus(ss);
        if (my_is_sht3x_enabled())
            this->data.sht3x.prometheus(ss);

        this->unlock();

        return ss;
    }

private:
    SensorPeripheral peripheral;
    SemaphoreHandle_t mutex;
    httpd_handle_t prometheus_server;

    void lock() const noexcept
    {
        xSemaphoreTake(this->mutex, portMAX_DELAY);
    }

    void unlock() const noexcept
    {
        xSemaphoreGive(this->mutex);
    }
};


Sensor* make_sensor()
{
    auto sensor = new Sensor{
            {},
            SensorPeripheral{
                    {S8::Sensor{S8_UART_PORT}},
                    {Pms5003::Sensor{PMS5003_UART_PORT}},
                    {Bmp180::Sensor{
                            i2c::Device{
                                    BMP180_I2C_PORT,
                                    Bmp180::I2C_DELAY
                            },
                            Bmp180::SEA_LEVEL_PRESSURE,
                            Bmp180::OVERSAMPLING_ULTRA_HIGH_RES
                    }},
                    {Sht3x::Sensor{
                            i2c::Device{
                                    SHT3X_I2C_PORT,
                                    Sht3x::I2C_DELAY
                            }
                    }}
            }
    };

    if (!sensor) {
        ESP_LOGE(MY_TAG, "could not allocate sensor");
        throw std::runtime_error("could not allocate sensor");
    }

    return sensor;
}

void read_sensors(Sensor* sensor) noexcept
{
    xTaskCreate(
            [](void* arg) {
                auto* sensor0 = static_cast<Sensor*>(arg);
                while (sensor0->loop) {
                    if(my_is_s8_enabled())
                        sensor0->update_s8();
                    my_sleep_millis(MEASUREMENT_DELAY);
                }
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
                while (sensor0->loop) {
                    if (my_is_bmp180_enabled())
                        sensor0->update_bmp180();
                    my_sleep_millis(MEASUREMENT_DELAY);
                }
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
                while (sensor0->loop) {
                    if(my_is_pms5003_enabled())
                        sensor0->update_pms5003();
                    my_sleep_millis(MEASUREMENT_DELAY);
                }
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
                while (sensor0->loop) {
                    if(my_is_sht3x_enabled())
                        sensor0->update_sht3x();
                    my_sleep_millis(MEASUREMENT_DELAY);
                }
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
                my_sleep_millis(PRINT_INITIAL_DELAY);

                auto* sensor0 = static_cast<Sensor*>(arg);
                while (sensor0->loop) {
                    std::stringstream ss;
                    print_sensor_dump_header(ss);
                    sensor0->dump_pretty_print(ss);
                    std::cout << ss.str();

                    my_sleep_millis(PRINT_DELAY);
                }
            },
            "work_dump_all",
            1024 * 4,
            sensor,
            10,
            nullptr
    );
}

bool push(const char* const url,
          const char* const token_header,
          std::stringstream& ss)
{
    const auto body = ss.str();
    const auto len = body.length();

    if (len < 1) {
        ESP_LOGW(PUSHER_TAG, "nothing to push");
        return false;
    }

    const auto len_int = static_cast<int>(body.length());
    if ((len_int - len) != 0) {
        ESP_LOGE(PUSHER_TAG, "body too large");
        return false;
    }

    esp_http_client_config_t config;
    std::memset(&config, 0, sizeof(esp_http_client_config_t));
    config.url = url;
    config.disable_auto_redirect = false;
    config.max_redirection_count = 10;
    config.keep_alive_enable = false;

    auto client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(PUSHER_TAG, "http client creation failed");
        return false;
    }

    esp_err_t err;

    ESP_ERROR_CHECK(esp_http_client_set_method(client, HTTP_METHOD_POST));
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "Authorization", token_header));
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "Content-Type", "text/plain; charset=utf-8"));
    ESP_ERROR_CHECK(esp_http_client_set_header(client, "Accept", "application/json"));

    ESP_LOGI(PUSHER_TAG, "sending http post request to url=%s", url);
    ESP_ERROR_CHECK(err = esp_http_client_set_post_field(client, body.c_str(), len_int));

    const auto then = now_millis();
    err = esp_http_client_perform(client);
    const auto spent = now_millis() - then;
    const auto status_code = esp_http_client_get_status_code(client);
    ESP_LOGI(PUSHER_TAG, "Http request duration: %llu" "ms" " response_status_code=%d", spent, status_code);

    ESP_ERROR_CHECK(esp_http_client_cleanup(client));

    if (err != ESP_OK || status_code < 200 || 299 < status_code) {
        ESP_LOGE(PUSHER_TAG, "HTTP REQUEST FAILED!, err=%s status_code=%d", esp_err_to_name(err), status_code);
        return false;
    }
    else {
        ESP_LOGD(PUSHER_TAG, "wrote bytes=%d", len);
        return true;
    }
}

void push_influxdb(Sensor* sensor) noexcept
{
    if (!my_is_wifi_enabled()) {
        ESP_LOGW(PUSHER_TAG, "Wifi not enabled, disabled pushing metrics.");
        return;
    }
    if (strlen(CONFIG_MY_INFLUXDB_URL) < 1) {
        ESP_LOGW(PUSHER_TAG, "InfluxDB URL not set, disabled pushing metrics.");
        return;
    }

    xTaskCreate(
            [](void* arg) {
                my_sleep_millis(PUSH_INITIAL_DELAY);

                auto* sensor0 = static_cast<Sensor*>(arg);

                std::string url{CONFIG_MY_INFLUXDB_URL};
                url += "/api/v2/write?org=";
                url += CONFIG_MY_INFLUXDB_ORG;
                url += "&bucket=";
                url += CONFIG_MY_INFLUXDB_BUCKET;
                url += "&precision=ms";

                const auto* url_str = url.c_str();

                std::string token_header{"Token "};
                token_header += CONFIG_MY_INFLUXDB_TOKEN;
                const auto* token_header_str = token_header.c_str();

                while (sensor0->loop) {
                    std::stringstream ss;
                    sensor0->dump_influxdb(ss);

                    ss << "free_rtos_tasks,device=ESP32 value="
                       << uxTaskGetNumberOfTasks()
                       << "\nwifi_rssi,device=ESP32 value=";

                    wifi_ap_record_t ap_rec;
                    if (esp_wifi_sta_get_ap_info(&ap_rec) == ESP_OK) {
                        ss << std::to_string(ap_rec.rssi);
                    }
                    else {
                        ESP_LOGV(MY_TAG, "could not get wifi info.");
                        ss << "NaN";
                    }
                    ss << "\n";

                    multi_heap_info_t heap_info;
                    heap_caps_get_info(&heap_info, MALLOC_CAP_DEFAULT);
                    ss << "heap_free_bytes,device=ESP32 value="
                       << heap_info.total_free_bytes
                       << "\n";
                    ss << "heap_alloc_bytes,device=ESP32 value="
                       << heap_info.total_allocated_bytes
                       << "\n";

                    push(url_str, token_header_str, ss);

                    my_sleep_millis(PUSH_DELAY);
                }
            },
            "work_dump_all",
            1024 * 4,
            sensor,
            10,
            nullptr
    );
}


esp_err_t prometheus_http_server_handler(httpd_req_t* req) noexcept
{
    const auto* sensor = static_cast<Sensor*>(req->user_ctx);

    auto ss = sensor->dump_prometheus();

    ss << "# HELP free_rtos_tasks FreeRTOS total tasks\n"
          "# TYPE free_rtos_tasks gauge\n"
          "free_rtos_tasks "
       << uxTaskGetNumberOfTasks()
       << "\n";

    ss << "# HELP uptime_seconds uptime\n"
          "# TYPE uptime_seconds gauge\n"
          "uptime_seconds "
       << (now_millis() * 1000)
       << "\n";

    ss << "# HELP wifi_rssi Wifi strength\n"
          "# TYPE wifi_rssi gauge\n"
          "wifi_rssi ";
    wifi_ap_record_t ap_rec;
    if (esp_wifi_sta_get_ap_info(&ap_rec) == ESP_OK) {
        ss << std::to_string(ap_rec.rssi);
    }
    else {
        ESP_LOGV(MY_TAG, "could not get wifi info.");
        ss << "NaN";
    }
    ss << "\n";

    multi_heap_info_t heap_info;
    heap_caps_get_info(&heap_info, MALLOC_CAP_DEFAULT);
    ss << "# HELP heap_free_bytes Heap free bytes\n"
          "# TYPE heap_free_bytes gauge\n"
          "heap_free_bytes "
       << heap_info.total_free_bytes;
    ss << "\n# HELP heap_alloc_bytes Heap allocated bytes\n"
          "# TYPE heap_alloc_bytes gauge\n"
          "heap_alloc_bytes "
       << heap_info.total_allocated_bytes
       << "\n";

    const auto httpd_err = httpd_resp_send(req, ss.str().c_str(), HTTPD_RESP_USE_STRLEN);
    if (httpd_err == ESP_OK)
        ESP_LOGI(MY_TAG, "http request served on /metrics");
    else
        ESP_LOGW(MY_TAG, "http request failed on /metrics, err=%s", esp_err_to_name(httpd_err));
    return httpd_err;
}

httpd_handle_t prometheus_http_server(Sensor* sensor)
{
    esp_err_t err;

    auto config = my_get_httpd_config(CONFIG_MY_PROMETHEUS_PORT);
    httpd_handle_t server = nullptr;
    err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(MY_TAG, "registering http server failed=%s", esp_err_to_name(err));
        throw std::runtime_error{"registering http server failed"};
    }

    httpd_uri_t prometheus_handler_uri{};
    prometheus_handler_uri.uri = PROMETHEUS_HANDLER_URI;
    prometheus_handler_uri.method = HTTP_GET;
    prometheus_handler_uri.user_ctx = sensor;
    prometheus_handler_uri.handler = prometheus_http_server_handler;
    err = httpd_register_uri_handler(server, &prometheus_handler_uri);
    if (err != ESP_OK) {
        ESP_LOGE(MY_TAG, "registering http handler failed=%s", esp_err_to_name(err));

        err = httpd_stop(server);
        if (err != ESP_OK)
            ESP_LOGE(MY_TAG, "stopping http server failed=%s", esp_err_to_name(err));

        throw std::runtime_error{"registering http handler failed"};
    }

    ESP_LOGI(MY_TAG, "http server started on port: %d", CONFIG_MY_PROMETHEUS_PORT);
    return server;
}

}

// WIFI
namespace {

const char* const WIFI_TAG = "my_scan";

void my_wifi_event_handler(__attribute__((unused)) void* arg,
                           esp_event_base_t event_base,
                           int32_t event_id,
                           void* event_data) noexcept
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(WIFI_TAG, "start");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(WIFI_TAG, "disconnect");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const auto* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
    else {
        ESP_LOGI(WIFI_TAG, "unknown event");
    }
}

void my_wifi_init(const char* ssid,
                  const char* password) noexcept
{
    if (strlen(ssid) == 0) {
        ESP_LOGW(WIFI_TAG, "SSID not set, not enabling wifi.");
        return;
    }

    ESP_LOGI(WIFI_TAG, "wifi init...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    auto cfg = my_get_wifi_cfg();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &my_wifi_event_handler, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, &my_wifi_event_handler, nullptr, nullptr));

    // Initialize default station as network interface instance (esp-netif)
    esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // Initialize and start WiFi
    wifi_config_t wifi_config;
    std::memset(&wifi_config, 0, sizeof(wifi_config));

    wifi_config.sta.scan_method = WIFI_FAST_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.rssi = -127;
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    std::memcpy(wifi_config.sta.ssid, ssid, std::strlen(ssid));
    std::memcpy(wifi_config.sta.password, password, std::strlen(password));

    ESP_LOGI(WIFI_TAG, "SSID: %s;", wifi_config.sta.ssid);
    ESP_LOGI(WIFI_TAG, "PASS: %s;", wifi_config.sta.password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

}

// UART & I2C
namespace {

const char* const UART_TAG = "my_uart";
const char* const I2C_TAG = "my_uart";

void my_uart_init_8n1(const uart_port_t uart_num,
                      const int tx_pin,
                      const int rx_pin,
                      const uint16_t baudrate = 9600) noexcept
{
    ESP_LOGI(UART_TAG, "UART num=%d tx=%d rx=%d baudrate=%d 8N1 buffer=%d",
             uart_num, tx_pin, rx_pin, baudrate, UART_BUFFER_SIZE);

    uart_config_t uart_config;
    std::memset(&uart_config, 0, sizeof(uart_config));

    uart_config.baud_rate = baudrate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 122;

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(
            uart_num,
            tx_pin,
            rx_pin,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE
    ));

    ESP_ERROR_CHECK(uart_driver_install(
            uart_num,
            UART_BUFFER_SIZE,
            UART_BUFFER_SIZE,
            10,
            nullptr,
            0
    ));
}

void my_i2c_init(const i2c_port_t port,
                 const int sda_pin,
                 const int scl_pin,
                 const uint32_t clock_speed) noexcept
{
    ESP_LOGI(I2C_TAG, "I2C num=%d sda=%d scl=%d clock=%d", port, sda_pin, scl_pin, clock_speed);

    i2c_config_t i2c_conf;
    std::memset(&i2c_conf, 0, sizeof(i2c_conf));
    i2c_conf.mode = I2C_MODE_MASTER;
    i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf.sda_io_num = sda_pin;
    i2c_conf.scl_io_num = scl_pin;
    i2c_conf.master.clk_speed = clock_speed;

    ESP_ERROR_CHECK(i2c_param_config(port, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0));
}

}

// NVS
namespace {

const char* const NVS_TAG = "my_nvs";

void my_nvs_init() noexcept
{
    ESP_LOGI(NVS_TAG, "NVS init...");

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
}

}


extern "C" void app_main(void)
{
    ESP_LOGI(MY_TAG, "init...");
    my_nvs_init();

    if (my_is_wifi_enabled())
        my_wifi_init(CONFIG_MY_WIFI_SSID, CONFIG_MY_WIFI_PASSWORD);
    else
        ESP_LOGW(MY_TAG, "Wifi disabled");

    ESP_LOGI(MY_TAG, "peripherals...");

    if (my_is_pms5003_enabled())
        my_uart_init_8n1(PMS5003_UART_PORT, PMS5003_UART_TX_PIN, PMS5003_UART_RX_PIN);
    else
        ESP_LOGW(MY_TAG, "PMS5003 disabled");

    if (my_is_s8_enabled())
        my_uart_init_8n1(S8_UART_PORT, S8_UART_TX_PIN, S8_UART_RX_PIN);
    else
        ESP_LOGW(MY_TAG, "s8 disabled");

    if (my_is_bmp180_enabled())
        my_i2c_init(BMP180_I2C_PORT, BMP180_I2C_SDA_PIN, BMP180_I2C_SCL_PIN, MY_I2C_CLOCK_SPEED);
    else
        ESP_LOGW(MY_TAG, "bmp180 disabled");

    if (my_is_sht3x_enabled())
        my_i2c_init(SHT3X_I2C_PORT, SHT3X_I2C_SDA_PIN, SHT3X_I2C_SCL_PIN, MY_I2C_CLOCK_SPEED);
    else
        ESP_LOGW(MY_TAG, "sht3x disabled");

    ESP_LOGI(MY_TAG, "sensors...");
    auto sensor = make_sensor();

    ESP_LOGI(MY_TAG, "start...");
    read_sensors(sensor);
    dump_sensors(sensor);
    push_influxdb(sensor);
}
