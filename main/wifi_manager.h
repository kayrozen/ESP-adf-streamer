#pragma once
#include "esp_err.h"

/**
 * Initialize WiFi in STA mode and register event handlers.
 * Must be called once before wifi_manager_connect().
 */
esp_err_t wifi_manager_init(void);

/**
 * Connect to the given SSID and block until an IP is obtained or timeout.
 * @param ssid  WiFi network name
 * @param pass  WiFi password (NULL or "" for open network)
 * @return ESP_OK on connected, ESP_ERR_TIMEOUT if no IP within 15s
 */
esp_err_t wifi_manager_connect(const char *ssid, const char *pass);

/** Return true if WiFi is currently connected and has an IP. */
bool wifi_manager_is_connected(void);
