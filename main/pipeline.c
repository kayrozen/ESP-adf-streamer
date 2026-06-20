#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "audio_pipeline.h"
#include "audio_element.h"
#include "audio_event_iface.h"
#include "http_stream.h"
#include "mp3_decoder.h"
#include "aac_decoder.h"
#include "esp_peripherals.h"  /* must precede a2dp_stream.h (defines esp_periph_handle_t) */
#include "a2dp_stream.h"
#include "bt_manager.h"
#include "pipeline.h"

static const char *TAG = "pipeline";

/* PCM-side jitter buffer feeding a2dp_stream — THE buffer that rides out input
 * delivery stalls. The decoder eagerly fills it until full then blocks, so it
 * stays near-full and the BT sink drains it in real time. Its depth sets how
 * long a no-data gap on the source the audio can survive without a dropout.
 *
 *   24KB  (original) ≈ 0.13s @ 48kHz/stereo/16-bit (192KB/s) — too short for the
 *                       200-500ms BT/WiFi coexistence stalls (logs 21-23 chop).
 *   256KB (PR pre-D)  ≈ 1.33s — fixed the MP3 baseline (plaintext HTTP, smooth
 *                       server pacing); gap-free in logs 35/36.
 *   512KB (now)       ≈ 2.67s — needed for the HTTPS AAC/HLS streams. Log 47
 *                       (steady AAC from icecast.radiofrance.fr) showed the TLS
 *                       socket delivering NOTHING for 1.38s, 1.35s and 2.08s at
 *                       a time, then bursting 30-57KB/s to catch up. TLS hands
 *                       data over in burstier encrypted records and its per-
 *                       record decrypt adds Core-0 pressure (http 23%, IDLE0 8%
 *                       in the log), so the server<->socket pacing develops
 *                       multi-second gaps the plaintext MP3 path never had. A
 *                       1.38s gap just exceeds the 256KB (1.33s) buffer, draining
 *                       it to a brief dropout every ~12s — the reported AAC chop.
 *                       512KB absorbs the observed 2.08s worst case with margin.
 *
 * Lives in PSRAM (>16KB SPIRAM_MALLOC_ALWAYSINTERNAL threshold), ~3.5MB free, so
 * it costs no internal DRAM. Only one decoder is alive at a time except a brief
 * hot-swap overlap (2x512KB = 1MB peak), well within PSRAM headroom. Adds ~2.7s
 * startup/seek latency, irrelevant for internet radio. */
#define PCM_JITTER_RB_SIZE  (512 * 1024)

/* Pipeline handles */
static audio_pipeline_handle_t   s_pipeline     = NULL;
static audio_element_handle_t    s_http_el      = NULL;
static audio_element_handle_t    s_decoder_el   = NULL;
static audio_element_handle_t    s_a2dp_el      = NULL;
static audio_event_iface_handle_t s_evt         = NULL;

/* Peer BDA cache */
static uint8_t s_peer_bda[6] = {0};

/* Decoder type tracking for dynamic selection */
static pipeline_codec_t s_current_codec = PIPELINE_CODEC_AUTO;

/* Mutex for protecting pipeline state during station changes */
static SemaphoreHandle_t s_pipeline_mutex = NULL;

/* Forward declaration for static function used in pipeline_start */
static esp_err_t pipeline_recreate_decoder(pipeline_codec_t new_codec);

/* ---- helpers ---- */

static audio_element_handle_t create_http_stream(void)
{
    http_stream_cfg_t cfg = HTTP_STREAM_CFG_DEFAULT();
    cfg.type              = AUDIO_STREAM_READER;
    cfg.enable_playlist_parser = true;   /* HLS playlist support */
    cfg.task_stack        = 6 * 1024;   /* TLS handshake (HTTPS AAC/HLS stations) runs on this task — do not shrink */
    cfg.task_prio         = 23;
    /* HTTP_STREAM_TASK_CORE defaults to 0 via Kconfig — move to Core 1.
     *
     * Log 32 CPU table: Core 0 runs at 92% busy (IDLE0 = 8%). The load
     * breakdown is BTC_TASK 20% + btController 13% + WiFi 13% + BTU_TASK 9%
     * + hciT 3% = 58% for BT/WiFi, plus http 18% competing on the same core.
     * Core 1 runs at 37% busy (dec 34%, IDLE1 63%), so it has ample headroom.
     *
     * Pinning http to Core 1 moves ~18% off Core 0. That headroom goes to
     * BTC_TASK / btController, which need it to encode SBC frames on time —
     * the root cause of the L2CAP is_cong bursts that throttle throughput
     * below real-time. http and dec share Core 1 in natural turn: http writes
     * to the ring buffer, dec drains it, so they mostly alternate rather than
     * compete. */
    cfg.task_core         = 1;
    /* Compressed-side jitter buffer. 64KB ≈ 4s of 128kbps MP3. Lives in PSRAM
     * (>16KB SPIRAM_MALLOC_ALWAYSINTERNAL threshold), so it costs no internal
     * DRAM — and moving it out of internal actually frees the old 4KB. Lets the
     * decoder burst-refill from the socket backlog faster than real-time after
     * a BT/WiFi coexistence stall, instead of starving on a 4KB input. */
    cfg.out_rb_size       = 64 * 1024;
    return http_stream_init(&cfg);
}

static audio_element_handle_t create_mp3_decoder(void)
{
    mp3_decoder_cfg_t cfg = DEFAULT_MP3_DECODER_CONFIG();
    /* Pin the decoder to Core 1. Core 0 runs the WiFi driver (core=0 in the
     * logs), the BT controller, Bluedroid, and BT/WiFi coexistence — at 160 MHz
     * the MP3 decode competed with all of that and could only sustain ~108 KB/s
     * PCM (log 26), below the 192 KB/s real-time rate → choppy. Core 1 only runs
     * the app/monitor tasks, so the decoder gets a near-dedicated core; combined
     * with the 240 MHz bump this clears the real-time decode budget with margin. */
    cfg.task_core         = 1;
    cfg.task_prio         = 23;
    cfg.out_rb_size       = PCM_JITTER_RB_SIZE;  /* see PCM_JITTER_RB_SIZE note */
    return mp3_decoder_init(&cfg);
}

static audio_element_handle_t create_aac_decoder(void)
{
    aac_decoder_cfg_t cfg = DEFAULT_AAC_DECODER_CONFIG();
    /* Pin to Core 1 — same rationale as create_mp3_decoder(): keep CPU-heavy
     * decode off Core 0, which is saturated by WiFi + BT + coexistence. */
    cfg.task_core         = 1;
    cfg.task_prio         = 23;
    cfg.out_rb_size       = PCM_JITTER_RB_SIZE;  /* see PCM_JITTER_RB_SIZE note */
    return aac_decoder_init(&cfg);
}

/* Create decoder based on codec type */
static audio_element_handle_t create_decoder(pipeline_codec_t codec)
{
    switch (codec) {
        case PIPELINE_CODEC_AAC:
            return create_aac_decoder();
        case PIPELINE_CODEC_MP3:
        case PIPELINE_CODEC_AUTO:
        default:
            return create_mp3_decoder();
    }
}

/* Detect codec from URL */
static pipeline_codec_t detect_codec_from_url(const char *url)
{
    if (!url) return PIPELINE_CODEC_MP3;
    
    if (strstr(url, ".aac") || strstr(url, "AAC") || strstr(url, "aac")) {
        return PIPELINE_CODEC_AAC;
    }
    if (strstr(url, ".m3u8") || strstr(url, "hls") || strstr(url, "HLS")) {
        /* HLS typically uses AAC */
        return PIPELINE_CODEC_AAC;
    }
    return PIPELINE_CODEC_MP3;
}

static audio_element_handle_t create_a2dp_stream(void)
{
    /* ADF v2.8: type is a2dp_stream_config_t; no A2DP_STREAM_CFG_DEFAULT macro.
     * task_stack / task_prio are not fields of this struct. */
    a2dp_stream_config_t cfg = {
        .type          = AUDIO_STREAM_WRITER,  /* source: we push PCM to BT */
        .user_callback = {
            /* a2dp_stream owns the esp_a2d callback; it invokes this on
             * connection-state events so bt_manager can track link state. */
            .user_a2d_cb = bt_manager_a2dp_state_cb,
        },
        .audio_hal     = NULL,
    };
    return a2dp_stream_init(&cfg);
}

/* ---- public API ---- */

esp_err_t pipeline_init(const uint8_t peer_bda[6], const char *boot_url)
{
    memcpy(s_peer_bda, peer_bda, 6);

    if (!s_pipeline_mutex) {
        s_pipeline_mutex = xSemaphoreCreateMutex();
        if (!s_pipeline_mutex) {
            ESP_LOGE(TAG, "Failed to create pipeline mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    s_pipeline = audio_pipeline_init(&pipeline_cfg);
    if (!s_pipeline) {
        ESP_LOGE(TAG, "pipeline_init failed");
        return ESP_FAIL;
    }

    /* Create the initial decoder matching the first station's codec so the first
     * playback runs on a freshly-created decoder, not one reached via a live
     * MP3->AAC hot-swap. This isolates the decoder from the swap mechanism when
     * diagnosing AAC audio corruption. */
    pipeline_codec_t boot_codec = boot_url ? detect_codec_from_url(boot_url)
                                           : PIPELINE_CODEC_MP3;
    s_http_el     = create_http_stream();
    s_decoder_el  = create_decoder(boot_codec);
    s_a2dp_el     = create_a2dp_stream();

    if (!s_http_el || !s_decoder_el || !s_a2dp_el) {
        ESP_LOGE(TAG, "Failed to create one or more pipeline elements");
        if (s_http_el)    audio_element_deinit(s_http_el);
        if (s_decoder_el) audio_element_deinit(s_decoder_el);
        if (s_a2dp_el)    audio_element_deinit(s_a2dp_el);
        audio_pipeline_deinit(s_pipeline);
        s_pipeline = NULL;
        return ESP_FAIL;
    }

    esp_err_t ret = audio_pipeline_register(s_pipeline, s_http_el, "http");
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Register http failed: %d", ret); goto err_cleanup; }
    ret = audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Register dec failed: %d", ret); goto err_cleanup; }
    ret = audio_pipeline_register(s_pipeline, s_a2dp_el, "bt");
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Register bt failed: %d", ret); goto err_cleanup; }

    const char *link_tag[] = {"http", "dec", "bt"};
    ret = audio_pipeline_link(s_pipeline, link_tag, 3);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Pipeline link failed: %d", ret); goto err_cleanup; }

    /* Event interface: listen to pipeline + element events */
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    s_evt = audio_event_iface_init(&evt_cfg);
    if (!s_evt) {
        ESP_LOGE(TAG, "Failed to create event interface");
        goto err_cleanup;
    }
    audio_pipeline_set_listener(s_pipeline, s_evt);

    /* NOTE: the A2DP connection is intentionally NOT initiated here.  a2dp_stream
     * has now run esp_a2d_source_init (so the source profile is ready), but the
     * actual page must happen while WiFi is idle — app_main calls
     * bt_manager_connect_a2dp_blocking() before pipeline_start() so the BR/EDR
     * page does not contend with the HTTP stream under BT/WiFi coexistence. */

    s_current_codec = boot_codec;
    ESP_LOGI(TAG, "Pipeline initialized (boot codec %d)", boot_codec);
    return ESP_OK;

err_cleanup:
    if (s_evt) {
        audio_event_iface_destroy(s_evt);
        s_evt = NULL;
    }
    if (s_pipeline) {
        audio_pipeline_unregister(s_pipeline, s_http_el);
        audio_pipeline_unregister(s_pipeline, s_decoder_el);
        audio_pipeline_unregister(s_pipeline, s_a2dp_el);
        audio_pipeline_deinit(s_pipeline);
        s_pipeline = NULL;
    }
    if (s_http_el)    { audio_element_deinit(s_http_el);    s_http_el = NULL; }
    if (s_decoder_el) { audio_element_deinit(s_decoder_el); s_decoder_el = NULL; }
    if (s_a2dp_el)    { audio_element_deinit(s_a2dp_el);    s_a2dp_el = NULL; }
    return ESP_FAIL;
}

esp_err_t pipeline_start(const char *url)
{
    if (!s_pipeline || !url) return ESP_ERR_INVALID_STATE;

    /* Detect codec from URL and switch decoder if needed */
    pipeline_codec_t new_codec = detect_codec_from_url(url);
    if (new_codec != s_current_codec) {
        ESP_LOGI(TAG, "Codec change detected: %d -> %d", s_current_codec, new_codec);
        esp_err_t ret = pipeline_recreate_decoder(new_codec);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    audio_element_set_uri(s_http_el, url);
    ESP_LOGI(TAG, "Starting pipeline -> %s", url);

    esp_err_t ret = audio_pipeline_run(s_pipeline);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "audio_pipeline_run failed: %d", ret);
    }
    return ret;
}

/* Recreate decoder element for new codec type */
static esp_err_t pipeline_recreate_decoder(pipeline_codec_t new_codec)
{
    if (new_codec == s_current_codec) {
        return ESP_OK;  /* No change needed */
    }

    audio_element_handle_t new_decoder = create_decoder(new_codec);
    if (!new_decoder) {
        ESP_LOGE(TAG, "Failed to create new decoder for codec %d", new_codec);
        return ESP_FAIL;
    }

    /* Keep reference to old decoder for potential rollback.
     * Do NOT deinit old_decoder until new one is fully linked. */
    audio_element_handle_t old_decoder = s_decoder_el;

    /* Stop pipeline to safely swap decoder */
    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);

    /* Unlink pipeline before swapping elements */
    audio_pipeline_unlink(s_pipeline);

    /* Unregister old decoder (but don't deinit yet - keep for rollback) */
    audio_pipeline_unregister(s_pipeline, s_decoder_el);
    s_decoder_el = new_decoder;
    esp_err_t ret = audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register new decoder: %d", ret);
        /* Rollback: restore old decoder (still valid, not deinit'd) */
        s_decoder_el = old_decoder;
        audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
        const char *link_tag[] = {"http", "dec", "bt"};
        audio_pipeline_link(s_pipeline, link_tag, 3);
        audio_pipeline_run(s_pipeline);
        /* Now safe to deinit failed new decoder */
        audio_element_deinit(new_decoder);
        return ret;
    }

    const char *link_tag[] = {"http", "dec", "bt"};
    ret = audio_pipeline_link(s_pipeline, link_tag, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to relink pipeline after decoder swap: %d", ret);
        /* Rollback: restore old decoder (still valid, not deinit'd) */
        s_decoder_el = old_decoder;
        audio_pipeline_unregister(s_pipeline, new_decoder);
        audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
        audio_pipeline_link(s_pipeline, link_tag, 3);
        audio_pipeline_run(s_pipeline);
        /* Deinit failed new decoder only — old decoder was restored, not replaced */
        audio_element_deinit(new_decoder);
        return ret;
    }

    /* Success: new decoder is linked. Now safe to deinit old decoder. */
    audio_element_deinit(old_decoder);
    s_current_codec = new_codec;
    ESP_LOGI(TAG, "Decoder recreated for codec %d", new_codec);
    return ESP_OK;
}

esp_err_t pipeline_change_station(const char *new_url)
{
    if (!s_pipeline || !new_url) return ESP_ERR_INVALID_STATE;

    /* Take mutex to prevent concurrent access with event loop */
    if (xSemaphoreTake(s_pipeline_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take pipeline mutex for station change");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Changing station -> %s", new_url);

    /* Detect codec and recreate decoder if needed */
    pipeline_codec_t new_codec = detect_codec_from_url(new_url);
    esp_err_t ret = pipeline_recreate_decoder(new_codec);
    if (ret != ESP_OK) {
        xSemaphoreGive(s_pipeline_mutex);
        return ret;
    }

    /* Stop the pipeline but keep elements initialized to preserve A2DP connection.
     * Do NOT call audio_pipeline_terminate() — that deinitializes all elements
     * including the A2DP stream, which drops the Bluetooth connection. */
    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);

    audio_element_set_uri(s_http_el, new_url);
    audio_pipeline_reset_ringbuffer(s_pipeline);
    audio_pipeline_reset_elements(s_pipeline);
    audio_pipeline_change_state(s_pipeline, AEL_STATE_INIT);

    ret = audio_pipeline_run(s_pipeline);
    xSemaphoreGive(s_pipeline_mutex);
    return ret;
}

esp_err_t pipeline_stop(void)
{
    if (!s_pipeline) return ESP_OK;
    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);
    audio_pipeline_terminate(s_pipeline);
    ESP_LOGI(TAG, "Pipeline stopped");
    return ESP_OK;
}

void pipeline_deinit(void)
{
    if (!s_pipeline) return;
    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);
    audio_pipeline_terminate(s_pipeline);
    audio_pipeline_unregister(s_pipeline, s_http_el);
    audio_pipeline_unregister(s_pipeline, s_decoder_el);
    audio_pipeline_unregister(s_pipeline, s_a2dp_el);
    audio_pipeline_deinit(s_pipeline);
    audio_element_deinit(s_http_el);
    audio_element_deinit(s_decoder_el);
    audio_element_deinit(s_a2dp_el);
    audio_event_iface_destroy(s_evt);

    s_pipeline = NULL;
    s_http_el = NULL;
    s_decoder_el = NULL;
    s_a2dp_el = NULL;
    s_evt = NULL;

    if (s_pipeline_mutex) {
        vSemaphoreDelete(s_pipeline_mutex);
        s_pipeline_mutex = NULL;
    }
    ESP_LOGI(TAG, "Pipeline destroyed");
}

audio_event_iface_handle_t pipeline_get_event_iface(void)
{
    return s_evt;
}

audio_element_handle_t pipeline_get_decoder_el(void)
{
    return s_decoder_el;
}
