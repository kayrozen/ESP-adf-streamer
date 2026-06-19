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
 * Fire a single esp_a2d_source_connect() to the configured peer BDA.
 * Returns ESP_OK if the request was accepted (the link comes up asynchronously,
 * observed via bt_manager_a2dp_state_cb).  Safe to call while connected — it
 * skips the request and returns ESP_OK.  Use for runtime reconnect attempts.
 * Requires a2dp_stream_init() (esp_a2d_source_init) to have run first.
 */
esp_err_t bt_manager_connect_a2dp(void);

/**
 * Page the configured sink and BLOCK until A2DP is CONNECTED, retrying up to
 * max_attempts times (each attempt waits out the BR/EDR page timeout).  Call
 * this BEFORE starting the audio pipeline so the page happens while WiFi is
 * idle — streaming concurrently with the page starves it under BT/WiFi
 * coexistence and the page times out (SDP conn cnf 0x4).
 * Requires a2dp_stream_init() (esp_a2d_source_init) to have run first.
 * @return ESP_OK once connected, ESP_ERR_TIMEOUT if all attempts failed.
 */
esp_err_t bt_manager_connect_a2dp_blocking(int max_attempts);
