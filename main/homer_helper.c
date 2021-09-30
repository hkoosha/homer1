#include <stdint.h>

#include "esp_wifi.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// Clion goes crazy if I do it in cpp file.
wifi_init_config_t my_get_wifi_cfg()
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT()
    return cfg;
}

// Clion goes crazy if I do it in cpp file.
httpd_config_t my_get_httpd_config(const uint16_t port)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    return config;
}

#ifdef __cplusplus
}
#endif
