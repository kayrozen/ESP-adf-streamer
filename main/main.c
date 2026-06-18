#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "audio_event_iface.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "wifi_manager.h"
#include "bt_manager.h"
#include "pipeline.h"
#include "monitor.h"
#include "passthrough_el.h"
#include "station_config.h"
#include "config_manager.h"

static const char *TAG = "app_main";

/* Phase D — station rotation table */
static const station_t TEST_STATIONS[NUM_TEST_STATIONS] = {
    { "MP3 Icecast",      STATION_MP3_URL        },
    { "AAC Icecast",      STATION_AAC_URL        },
    { "HLS mono-bitrate", STATION_HLS_URL        },
    { "HLS multi-bitrate",STATION_HLS_MULTI_URL  },
};

static int s_current_station = 0;

/* Pipeline event loop — handles all ADF element/pipeline events */
static void run_event_loop(void)
{
    audio_event_iface_handle_t evt = pipeline_get_event_iface();
    if (!evt) {
        ESP_LOGE(TAG, "Event interface is NULL — pipeline did not initialize");
        return;
    }

    ESP_LOGI(TAG, "Entering event loop …");
    while (true) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "audio_event_iface_listen error: %d", ret);
            continue;
        }

        /* ---- HTTP stream events ---- */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT &&
            msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
            audio_element_info_t info = {0};
            audio_element_getinfo(msg.source, &info);
            ESP_LOGI(TAG, "Stream info — codec:%d  sample_rate:%d  channels:%d  bits:%d",
                     info.codec_fmt, info.sample_rates, info.channels, info.bits);
        }

        /* ---- Pipeline finished / error ---- */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT &&
            msg.cmd == AEL_MSG_CMD_REPORT_STATUS) {
            audio_element_status_t st = (audio_element_status_t)(int)msg.data;
            if (st == AEL_STATUS_STATE_FINISHED) {
                ESP_LOGI(TAG, "Stream finished — restarting same station");
                pipeline_change_station(TEST_STATIONS[s_current_station].url);
            } else if (st == AEL_STATUS_ERROR_OPEN   ||
                       st == AEL_STATUS_ERROR_INPUT  ||
                       st == AEL_STATUS_ERROR_PROCESS) {
                ESP_LOGE(TAG, "Stream error (status=%d) — retrying in 3s", st);
                vTaskDelay(pdMS_TO_TICKS(3000));
                pipeline_change_station(TEST_STATIONS[s_current_station].url);
            }
        }
    }
}

/* Phase D — hot station change, keeps A2DP alive.
 * Only compiled when rotation is enabled to avoid unused-function error. */
#if CONFIG_PROTOTYPE_PHASE_D_ROTATION
static esp_err_t switch_to_station(int idx)
{
    if (idx < 0 || idx >= NUM_TEST_STATIONS) return ESP_ERR_INVALID_ARG;
    s_current_station = idx;
    ESP_LOGI(TAG, "Switching to station %d: %s", idx, TEST_STATIONS[idx].name);
    int64_t t0 = esp_timer_get_time();
    esp_err_t ret = pipeline_change_station(TEST_STATIONS[idx].url);
    int64_t elapsed_ms = (esp_timer_get_time() - t0) / 1000;
    ESP_LOGI(TAG, "Station switch took %" PRId64 " ms", elapsed_ms);
    return ret;
}
#endif /* CONFIG_PROTOTYPE_PHASE_D_ROTATION */

void app_main(void)
{
    /* ---- NVS ---- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ---- Runtime config (NVS "preset" namespace, fallback to station_config.h) ---- */
    ESP_ERROR_CHECK(config_manager_init());

    /* ---- Boot heap report (Phase A.1 validation) ---- */
    ESP_LOGI(TAG, "=== ESP-ADF Streamer prototype ===");
    ESP_LOGI(TAG, "Heap at boot: internal=%"PRIu32"B  SPIRAM=%"PRIu32"B",
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) < 1024 * 1024) {
        ESP_LOGE(TAG,
                 "PSRAM not detected or < 1MB free. "
                 "Check BOARD_HAS_PSRAM and SPIRAM_CACHE_WORKAROUND flags.");
    }

    /* ---- Phase A.3: WiFi ---- */
    ESP_ERROR_CHECK(wifi_manager_init());
    ret = wifi_manager_connect(config_get_wifi_ssid(), config_get_wifi_pass());
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connect failed — continuing anyway (offline test)");
    }

    /* ---- Phase A.2: Bluetooth ---- */
    ESP_ERROR_CHECK(bt_manager_init(BT_DEVICE_NAME));

    ret = bt_manager_find_peer(config_get_bt_mac(), 30);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No BT peer found — cannot start A2DP");
        ESP_LOGE(TAG, "Set BT_PEER_ADDR in station_config.h or enable a discoverable A2DP sink");
        /* Halt — without a BT peer the prototype cannot proceed */
        while (true) vTaskDelay(pdMS_TO_TICKS(5000));
    }

    /* ---- Phase B / C: Pipeline init ---- */
    ESP_ERROR_CHECK(pipeline_init(bt_manager_get_peer_bda()));  /* uses discovered/configured BDA */

    /* ---- Phase C: Resource monitoring ---- */
    monitor_start();

    /* ---- Phase B: Start first station (MP3 — simplest) ---- */
    ESP_LOGI(TAG, "Starting Phase B — MP3 Icecast baseline test");
    int64_t t_start = esp_timer_get_time();
    ESP_ERROR_CHECK(pipeline_start(TEST_STATIONS[0].url));
    ESP_LOGI(TAG, "Pipeline start latency: %" PRId64 " ms",
             (esp_timer_get_time() - t_start) / 1000);

    /*
     * Phase D smoke test — switch all stations in order after 60s each.
     * Comment this block out to stay on one station for Phase C endurance tests.
     */
#if CONFIG_PROTOTYPE_PHASE_D_ROTATION
    ESP_LOGI(TAG, "Phase D: station rotation enabled");
    for (int i = 0; i < NUM_TEST_STATIONS; i++) {
        vTaskDelay(pdMS_TO_TICKS(60 * 1000));
        switch_to_station(i);
        passthrough_stats_t stats;
        passthrough_el_get_stats(pipeline_get_passthrough_el(), &stats);
        ESP_LOGI(TAG, "Phase D passthrough: bytes=%" PRIu64 "  frames=%" PRIu32,
                 stats.bytes_passed, stats.frames_passed);
    }
#endif

    /* ---- Main event loop ---- */
    run_event_loop();

    /* Not reached in normal operation */
    pipeline_deinit();
}
