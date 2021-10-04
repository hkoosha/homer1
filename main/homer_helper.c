#include <stdint.h>

#include "esp_wifi.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// Clion goes crazy if I do it in cpp file.
inline wifi_init_config_t my_get_wifi_cfg()
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT()
    return cfg;
}

// Clion goes crazy if I do it in cpp file.
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

inline bool my_is_wifi_enabled()
{
    return CONFIG_MY_WIFI_ENABLED;
}


inline bool my_is_pms5003_enabled()
{
    return CONFIG_MY_SENSOR_ENABLED_PMS5003;
}

inline bool my_is_sht3x_enabled()
{
    return CONFIG_MY_SENSOR_ENABLED_SHT3X;
}

inline bool my_is_s8_enabled()
{
    return CONFIG_MY_SENSOR_ENABLED_S8;
}

inline bool my_is_bmp180_enabled()
{
    return CONFIG_MY_SENSOR_ENABLED_BMP180;
}


#ifdef __cplusplus
}
#endif
