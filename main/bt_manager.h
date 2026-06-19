#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_a2dp_api.h"

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

/**
 * A2DP connection-state observer.  The ADF a2dp_stream element owns the
 * esp_a2d callback; pass this as a2dp_stream_config_t.user_callback.user_a2d_cb
 * so bt_manager can track connection state (esp_a2d_cb_t signature).
 */
void bt_manager_a2dp_state_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

/**
 * Retry esp_a2d_source_connect() to the configured peer BDA.
 * Safe to call while the pipeline is running; the BT stack ignores
 * duplicate connect requests when already connected.
 */
esp_err_t bt_manager_reconnect_a2dp(void);
