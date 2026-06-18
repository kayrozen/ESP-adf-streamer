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
#include "esp_a2dp_api.h"
#include "passthrough_el.h"
#include "pipeline.h"

static const char *TAG = "pipeline";

/* Pipeline handles */
static audio_pipeline_handle_t   s_pipeline     = NULL;
static audio_element_handle_t    s_http_el      = NULL;
static audio_element_handle_t    s_decoder_el   = NULL;
static audio_element_handle_t    s_passthrough  = NULL;
static audio_element_handle_t    s_a2dp_el      = NULL;
static audio_event_iface_handle_t s_evt         = NULL;

/* Peer BDA cache */
static uint8_t s_peer_bda[6] = {0};

/* Decoder type tracking for dynamic selection */
static pipeline_codec_t s_current_codec = PIPELINE_CODEC_AUTO;

/* Mutex for protecting pipeline state during station changes */
static SemaphoreHandle_t s_pipeline_mutex = NULL;

/* ---- helpers ---- */

static audio_element_handle_t create_http_stream(void)
{
    http_stream_cfg_t cfg = HTTP_STREAM_CFG_DEFAULT();
    cfg.type              = AUDIO_STREAM_READER;
    cfg.enable_playlist_parser = true;   /* HLS playlist support */
    cfg.task_stack        = 6 * 1024;
    cfg.task_prio         = 23;
    return http_stream_init(&cfg);
}

static audio_element_handle_t create_mp3_decoder(void)
{
    mp3_decoder_cfg_t cfg = DEFAULT_MP3_DECODER_CONFIG();
    cfg.task_core         = 0;
    cfg.task_prio         = 23;
    return mp3_decoder_init(&cfg);
}

static audio_element_handle_t create_aac_decoder(void)
{
    aac_decoder_cfg_t cfg = DEFAULT_AAC_DECODER_CONFIG();
    cfg.task_core         = 0;
    cfg.task_prio         = 23;
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
        .user_callback = { 0 },
        .audio_hal     = NULL,
    };
    return a2dp_stream_init(&cfg);
}

/* ---- public API ---- */

esp_err_t pipeline_init(const uint8_t peer_bda[6])
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

    s_http_el     = create_http_stream();
    s_decoder_el  = create_decoder(PIPELINE_CODEC_MP3);  /* default, will be updated on first stream */
    s_passthrough = passthrough_el_init();
    s_a2dp_el     = create_a2dp_stream();

    if (!s_http_el || !s_decoder_el || !s_passthrough || !s_a2dp_el) {
        ESP_LOGE(TAG, "Failed to create one or more pipeline elements");
        if (s_http_el)    audio_element_deinit(s_http_el);
        if (s_decoder_el) audio_element_deinit(s_decoder_el);
        if (s_passthrough)audio_element_deinit(s_passthrough);
        if (s_a2dp_el)    audio_element_deinit(s_a2dp_el);
        audio_pipeline_deinit(s_pipeline);
        s_pipeline = NULL;
        return ESP_FAIL;
    }

    esp_err_t ret = audio_pipeline_register(s_pipeline, s_http_el, "http");
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Register http failed: %d", ret); goto err_cleanup; }
    ret = audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Register dec failed: %d", ret); goto err_cleanup; }
    ret = audio_pipeline_register(s_pipeline, s_passthrough, "pass");
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Register pass failed: %d", ret); goto err_cleanup; }
    ret = audio_pipeline_register(s_pipeline, s_a2dp_el, "bt");
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Register bt failed: %d", ret); goto err_cleanup; }

    const char *link_tag[] = {"http", "dec", "pass", "bt"};
    ret = audio_pipeline_link(s_pipeline, link_tag, 4);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Pipeline link failed: %d", ret); goto err_cleanup; }

    /* Event interface: listen to pipeline + element events */
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    s_evt = audio_event_iface_init(&evt_cfg);
    if (!s_evt) {
        ESP_LOGE(TAG, "Failed to create event interface");
        goto err_cleanup;
    }
    audio_pipeline_set_listener(s_pipeline, s_evt);

    /* Connect A2DP to the peer speaker */
    ESP_LOGI(TAG, "Connecting A2DP to %02X:%02X:%02X:%02X:%02X:%02X",
             s_peer_bda[5], s_peer_bda[4], s_peer_bda[3],
             s_peer_bda[2], s_peer_bda[1], s_peer_bda[0]);
    esp_a2d_source_connect(s_peer_bda);

    s_current_codec = PIPELINE_CODEC_MP3;
    ESP_LOGI(TAG, "Pipeline initialized");
    return ESP_OK;

err_cleanup:
    if (s_evt) {
        audio_event_iface_destroy(s_evt);
        s_evt = NULL;
    }
    if (s_pipeline) {
        audio_pipeline_unregister(s_pipeline, s_http_el);
        audio_pipeline_unregister(s_pipeline, s_decoder_el);
        audio_pipeline_unregister(s_pipeline, s_passthrough);
        audio_pipeline_unregister(s_pipeline, s_a2dp_el);
        audio_pipeline_deinit(s_pipeline);
        s_pipeline = NULL;
    }
    if (s_http_el)    { audio_element_deinit(s_http_el);    s_http_el = NULL; }
    if (s_decoder_el) { audio_element_deinit(s_decoder_el); s_decoder_el = NULL; }
    if (s_passthrough){ audio_element_deinit(s_passthrough); s_passthrough = NULL; }
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
        /* Will be handled in pipeline_change_station logic */
        s_current_codec = new_codec;
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

    /* Keep reference to old decoder for rollback on failure */
    audio_element_handle_t old_decoder = s_decoder_el;

    /* Stop pipeline to safely swap decoder */
    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);

    /* Unlink pipeline before swapping elements */
    audio_pipeline_unlink(s_pipeline);

    /* Unregister old decoder, register new one */
    audio_pipeline_unregister(s_pipeline, s_decoder_el);
    audio_element_deinit(s_decoder_el);
    s_decoder_el = new_decoder;
    esp_err_t ret = audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register new decoder: %d", ret);
        /* Rollback: restore old decoder */
        s_decoder_el = old_decoder;
        audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
        const char *link_tag[] = {"http", "dec", "pass", "bt"};
        audio_pipeline_link(s_pipeline, link_tag, 4);
        audio_pipeline_change_state(s_pipeline, AEL_STATE_RUNNING);
        return ret;
    }

    const char *link_tag[] = {"http", "dec", "pass", "bt"};
    ret = audio_pipeline_link(s_pipeline, link_tag, 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to relink pipeline after decoder swap: %d", ret);
        /* Rollback: restore old decoder */
        s_decoder_el = old_decoder;
        audio_pipeline_unregister(s_pipeline, new_decoder);
        audio_element_deinit(new_decoder);
        audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
        audio_pipeline_link(s_pipeline, link_tag, 4);
        audio_pipeline_change_state(s_pipeline, AEL_STATE_RUNNING);
        return ret;
    }

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
    audio_pipeline_unregister(s_pipeline, s_passthrough);
    audio_pipeline_unregister(s_pipeline, s_a2dp_el);
    audio_pipeline_deinit(s_pipeline);
    audio_element_deinit(s_http_el);
    audio_element_deinit(s_decoder_el);
    audio_element_deinit(s_passthrough);
    audio_element_deinit(s_a2dp_el);
    audio_event_iface_destroy(s_evt);
    
    /* Clear all handles to avoid dangling pointers */
    s_pipeline = NULL;
    s_http_el = NULL;
    s_decoder_el = NULL;
    s_passthrough = NULL;
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

audio_element_handle_t pipeline_get_passthrough_el(void)
{
    return s_passthrough;
}
