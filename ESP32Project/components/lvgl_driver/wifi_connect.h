#ifndef WIFI_CONNECT_H_
#define WIFI_CONNECT_H_

#include "esp_err.h"

esp_err_t connect_init(void);
esp_err_t wifi_connect(char* wifi_ssid, char* wifi_password);
esp_err_t tutorial_disconnect(void);
esp_err_t tutorial_deinit(void);

#endif
