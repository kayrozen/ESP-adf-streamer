#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * Initialize the Classic Bluetooth stack (Bluedroid).
 * Must be called before pipeline_init().
 * @param device_name  BT device name advertised to peers
 */
esp_err_t bt_manager_init(const char *device_name);

/**
 * Start GAP inquiry to find A2DP sinks.
 * When a sink is found, bt_manager_get_peer_bda() returns its address.
 * If peer_bda is non-zero, skips scan and uses the given address directly.
 * Blocks for up to timeout_s seconds.
 * @return ESP_OK if a peer was found/set
 */
esp_err_t bt_manager_find_peer(const uint8_t peer_bda[6], uint32_t timeout_s);

/**
 * Returns the BDA of the found/configured peer (valid after bt_manager_find_peer).
 */
const uint8_t *bt_manager_get_peer_bda(void);

/** True if A2DP connection is currently established. */
bool bt_manager_is_a2dp_connected(void);
