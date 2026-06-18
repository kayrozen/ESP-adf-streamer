#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

/* create_aac_decoder is used for Phase B step 2 (AAC Icecast test).
 * Suppress unused-function warning — it will be wired in when testing AAC. */
static audio_element_handle_t create_aac_decoder(void) __attribute__((unused));
static audio_element_handle_t create_aac_decoder(void)
{
    aac_decoder_cfg_t cfg = DEFAULT_AAC_DECODER_CONFIG();
    cfg.task_core         = 0;
    cfg.task_prio         = 23;
    return aac_decoder_init(&cfg);
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

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    s_pipeline = audio_pipeline_init(&pipeline_cfg);
    if (!s_pipeline) {
        ESP_LOGE(TAG, "pipeline_init failed");
        return ESP_FAIL;
    }

    s_http_el     = create_http_stream();
    /* Default decoder: MP3. pipeline_start() can swap for AAC/HLS. */
    s_decoder_el  = create_mp3_decoder();
    s_passthrough = passthrough_el_init();
    s_a2dp_el     = create_a2dp_stream();

    audio_pipeline_register(s_pipeline, s_http_el,     "http");
    audio_pipeline_register(s_pipeline, s_decoder_el,  "dec");
    audio_pipeline_register(s_pipeline, s_passthrough, "pass");
    audio_pipeline_register(s_pipeline, s_a2dp_el,     "bt");

    const char *link_tag[] = {"http", "dec", "pass", "bt"};
    audio_pipeline_link(s_pipeline, link_tag, 4);

    /* Event interface: listen to pipeline + element events */
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    s_evt = audio_event_iface_init(&evt_cfg);
    audio_pipeline_set_listener(s_pipeline, s_evt);

    /* Connect A2DP to the peer speaker */
    ESP_LOGI(TAG, "Connecting A2DP to %02X:%02X:%02X:%02X:%02X:%02X",
             s_peer_bda[5], s_peer_bda[4], s_peer_bda[3],
             s_peer_bda[2], s_peer_bda[1], s_peer_bda[0]);
    esp_a2d_source_connect(s_peer_bda);

    ESP_LOGI(TAG, "Pipeline initialized");
    return ESP_OK;
}

esp_err_t pipeline_start(const char *url)
{
    if (!s_pipeline || !url) return ESP_ERR_INVALID_STATE;

    audio_element_set_uri(s_http_el, url);
    ESP_LOGI(TAG, "Starting pipeline → %s", url);

    esp_err_t ret = audio_pipeline_run(s_pipeline);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "audio_pipeline_run failed: %d", ret);
    }
    return ret;
}

esp_err_t pipeline_change_station(const char *new_url)
{
    if (!s_pipeline || !new_url) return ESP_ERR_INVALID_STATE;

    ESP_LOGI(TAG, "Changing station → %s", new_url);

    /* Stop only the source; keep A2DP (bt element) alive to avoid BT gap */
    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);
    audio_pipeline_terminate(s_pipeline);

    audio_element_set_uri(s_http_el, new_url);
    audio_pipeline_reset_ringbuffer(s_pipeline);
    audio_pipeline_reset_elements(s_pipeline);
    audio_pipeline_change_state(s_pipeline, AEL_STATE_INIT);

    return audio_pipeline_run(s_pipeline);
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
    s_pipeline = NULL;
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
