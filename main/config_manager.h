#pragma once
#include <stdint.h>
#include "esp_err.h"

/* Reads WiFi/BT config from NVS namespace "preset".
 * Falls back to station_config.h defaults if keys are absent. */
esp_err_t config_manager_init(void);

const char    *config_get_wifi_ssid(void);
const char    *config_get_wifi_pass(void);
const uint8_t *config_get_bt_mac(void);   /* 6-byte BDA, big-endian order */
