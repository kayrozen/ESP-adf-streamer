#pragma once
#include "esp_err.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"

/**
 * pipeline.h — Audio pipeline management
 *
 * Pipeline topology:
 *   http_stream → [mp3|aac]_decoder → passthrough_el → a2dp_stream
 *
 * http_stream handles both plain HTTP/Icecast and HLS (.m3u8) transparently.
 * The passthrough_el is the Phase D custom element (byte counter).
 * a2dp_stream writes PCM to the Bluetooth A2DP source stack.
 */

typedef enum {
    PIPELINE_CODEC_AUTO = 0,  /* detect from Content-Type (default) */
    PIPELINE_CODEC_MP3,
    PIPELINE_CODEC_AAC,
} pipeline_codec_t;

/**
 * Initialize pipeline elements and register them.
 * Must be called after bt_manager_init() and wifi_manager_connect().
 * @param peer_bda   Bluetooth address of A2DP sink (from bt_manager)
 */
esp_err_t pipeline_init(const uint8_t peer_bda[6]);

/**
 * Start streaming from the given URL.
 * Caller must ensure A2DP is connected (bt_manager_is_a2dp_connected()) before
 * calling to avoid ring-buffer overflow while the BT sink is not yet ready.
 */
esp_err_t pipeline_start(const char *url);

/**
 * Change station: stop current stream, swap URL, restart — keeps A2DP alive.
 * Phase D: validates hot-swap without dropping BT connection.
 */
esp_err_t pipeline_change_station(const char *new_url);

/**
 * Stop the pipeline gracefully.
 */
esp_err_t pipeline_stop(void);

/**
 * Destroy pipeline and free all elements.
 */
void pipeline_deinit(void);

/**
 * Return the event interface for the pipeline event loop.
 */
audio_event_iface_handle_t pipeline_get_event_iface(void);

/**
 * Get the passthrough element handle (for Phase D stats).
 */
audio_element_handle_t pipeline_get_passthrough_el(void);
