#pragma once
#include "esp_err.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"

/**
 * pipeline.h — Audio pipeline management
 *
 * Pipeline topology:
 *   http_stream → [mp3|aac]_decoder → a2dp_stream
 *
 * http_stream handles both plain HTTP/Icecast and HLS (.m3u8) transparently.
 * a2dp_stream writes PCM to the Bluetooth A2DP source stack.
 * The decoder's ring buffer (24KB, PSRAM) acts as the A2DP jitter cushion.
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
 * @param boot_url   URL of the first station to play. Used to create the initial
 *                   decoder matching the first stream's codec, so the first
 *                   playback does not incur a MP3->AAC hot-swap. May be NULL
 *                   (falls back to an MP3 initial decoder).
 */
esp_err_t pipeline_init(const uint8_t peer_bda[6], const char *boot_url);

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
 * Get the decoder element handle (for stall detection via audio_element_getinfo).
 * The decoder's byte_pos advances as PCM is produced and freezes on both a
 * network stall (input starvation) and an A2DP stall (downstream backpressure),
 * making it a reliable liveness signal for the pipeline.
 */
audio_element_handle_t pipeline_get_decoder_el(void);

/**
 * Log a one-line playback diagnostic: the decoder's actual output sample rate /
 * channels, recent PCM throughput, and the dec/rsp ring-buffer fill levels.
 * Call periodically (e.g. every ~2 s) from the event loop while streaming to
 * distinguish BT starvation (rsp buffer draining) from over-production (buffer
 * full / resampler bottleneck) from clean-delivery-but-bad-content. No-op when
 * the decoder is not running.
 */
void pipeline_log_diag(void);
