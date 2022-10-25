#pragma once

#include <cstdint>

#include "esp_wifi.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

int32_t my_print_delay()
{
    return CONFIG_MY_PRINT_DELAY;
}

// ============================================================================

inline bool my_enable_write_to_esp()
{
#ifdef CONFIG_MY_ENABLE_WRITE_TO_DISPLAY_ESP
    return CONFIG_MY_ENABLE_WRITE_TO_DISPLAY_ESP;
#else
    return false;
#endif
}

inline gpio_num_t my_write_to_esp_gpio_pin()
{
    return (gpio_num_t) CONFIG_MY_WRITE_TO_DISPLAY_GPIO_INTERRUPT_PIN;
}

// ============================================================================

inline wifi_init_config_t my_get_wifi_cfg()
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    return cfg;
}

inline httpd_config_t my_get_httpd_config(const uint16_t port)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    return config;
}

inline bool my_is_prometheus_enabled()
{
    return CONFIG_MY_PROMETHEUS_PORT > 0;
}

inline const char* my_prometheus_uri()
{
    return CONFIG_MY_PROMETHEUS_URI;
}

inline bool my_is_influxdb_enabled()
{
    return strlen(CONFIG_MY_INFLUXDB_URL) >= 1;
}

inline uint32_t my_influxdb_push_delay()
{
    return CONFIG_MY_INFLUXDB_PUSH_DELAY;
}

inline bool my_is_wifi_enabled()
{
#ifdef CONFIG_MY_WIFI_ENABLED
    return CONFIG_MY_WIFI_ENABLED;
#else
    return false;
#endif
}

// ============================================================================

inline bool my_is_pms5003_enabled()
{
#ifdef CONFIG_MY_SENSOR_ENABLED_PMS5003
    return CONFIG_MY_SENSOR_ENABLED_PMS5003;
#else
    return false;
#endif
}

inline bool my_is_sht3x_enabled()
{
#ifdef CONFIG_MY_SENSOR_ENABLED_SHT3X
    return CONFIG_MY_SENSOR_ENABLED_SHT3X;
#else
    return false;
#endif
}

inline bool my_is_s8_enabled()
{
#ifdef CONFIG_MY_SENSOR_ENABLED_S8
    return CONFIG_MY_SENSOR_ENABLED_S8;
#else
    return false;
#endif
}

inline bool my_is_bmp180_enabled()
{
#ifdef CONFIG_MY_SENSOR_ENABLED_BMP180
    return CONFIG_MY_SENSOR_ENABLED_BMP180;
#else
    return false;
#endif
}

inline bool my_is_sgp30_enabled()
{
#ifdef CONFIG_MY_SENSOR_ENABLED_SGP30
    return CONFIG_MY_SENSOR_ENABLED_SGP30;
#else
    return false;
#endif
}


inline bool my_is_any_sensor_enabled()
{
    return my_is_pms5003_enabled()
           || my_is_sht3x_enabled()
           || my_is_s8_enabled()
           || my_is_bmp180_enabled()
           || my_is_sgp30_enabled();
}

// ============================================================================

inline bool my_needs_sensor_i2c()
{
    return my_is_bmp180_enabled()
           || my_is_sht3x_enabled()
           || my_is_sgp30_enabled();
}

inline uint32_t my_measurement_delay_millis()
{
    return CONFIG_MY_MEASUREMENT_DELAY_MILLIS;
}

// ============================================================================

inline int my_sensors_i2c_sda_pin()
{
    return CONFIG_MY_SENSORS_I2C_SDA_PIN;
}

inline int my_sensors_i2c_scl_pin()
{
    return CONFIG_MY_SENSORS_I2C_SCL_PIN;
}

inline int my_s8_rx_pin()
{
    return CONFIG_MY_S8_RX_PIN;
}

inline int my_s8_tx_pin()
{
    return CONFIG_MY_S8_TX_PIN;
}

inline int my_pms5003_rx_pin()
{
    return CONFIG_MY_PMS5003_RX_PIN;
}

inline int my_pms5003_tx_pin()
{
    return CONFIG_MY_PMS5003_TX_PIN;
}

#ifdef __cplusplus
}
#endif
