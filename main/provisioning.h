#pragma once
#include "esp_err.h"

/**
 * provisioning.h — UART provisioning for WiFi/BT config
 *
 * Listens on UART0 (115200 baud) for provisioning commands:
 *   PROVISION:{"ssid":"...","pass":"...","btmac":"..."}\n
 *
 * Saves to NVS namespace "preset" and reboots on success.
 */

esp_err_t provisioning_start(void);
void provisioning_stop(void);