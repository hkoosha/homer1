#include <cstdint>
#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/i2c.h"

using std::uint16_t;
using std::uint32_t;


// ======================================================================== WIFI

static const char* MY_WIFI_TAG = "my_scan";

static void my_wifi_event_handler(__attribute__((unused)) void* arg,
                                  esp_event_base_t event_base,
                                  int32_t event_id,
                                  void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(MY_WIFI_TAG, "start");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(MY_WIFI_TAG, "disconnect");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(MY_WIFI_TAG, "got ip:"
        IPSTR, IP2STR(&event->ip_info.ip));
    }
    else {
        ESP_LOGI(MY_WIFI_TAG, "unknown event");
    }
}

void my_wifi_init(const char* ssid,
                  const char* password)
{
    ESP_LOGI(MY_WIFI_TAG, "wifi init...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT()
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

    ESP_LOGI(MY_WIFI_TAG, "SSID: %s;", wifi_config.sta.ssid);
    ESP_LOGI(MY_WIFI_TAG, "PASS: %s;", wifi_config.sta.password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}


// ========================================================================= NVS

static const char* MY_NVS_TAG = "my_nvs";

void my_nvs_init()
{
    ESP_LOGI(MY_NVS_TAG, "NVS init...");

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
}


// ======================================================================== UART

static const char* MY_UART_TAG = "my_uart";
static const int MY_UART_BUFFER_SIZE = 1024 * 2;

void my_uart_init_8n1(const uart_port_t uart_num,
                      const int tx_pin,
                      const int rx_pin,
                      const uint16_t baudrate = 9600)
{
    ESP_LOGI(MY_UART_TAG, "UART num=%d tx=%d rx=%d baudrate=%d 8N1 buffer=%d",
             uart_num, tx_pin, rx_pin, baudrate, MY_UART_BUFFER_SIZE);

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
            MY_UART_BUFFER_SIZE,
            MY_UART_BUFFER_SIZE,
            10,
            nullptr,
            0
    ));
}


// ========================================================================= I2c

static const char* MY_I2C_TAG = "my_uart";


void my_i2c_init(const i2c_port_t port,
                 const int sda_pin,
                 const int scl_pin,
                 const uint32_t clock_speed)
{
    ESP_LOGI(MY_I2C_TAG, "I2C sda=%d scl=%d", sda_pin, scl_pin);

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
