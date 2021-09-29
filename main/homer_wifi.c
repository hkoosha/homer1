#include <esp_wifi.h>

#ifdef __cplusplus
extern "C" {
#endif

// Clion goes crazy if I do it in cpp file.
wifi_init_config_t my_get_wifi_cfg()
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT()
    return cfg;
}

#ifdef __cplusplus
}
#endif
