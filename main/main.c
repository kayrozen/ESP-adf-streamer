#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_coexist.h"
#include "nvs_flash.h"
#include "audio_event_iface.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "wifi_manager.h"
#include "bt_manager.h"
#include "pipeline.h"
#include "monitor.h"
#include "station_config.h"
#include "config_manager.h"
#include "provisioning.h"

static const char *TAG = "app_main";

/* Phase D — station rotation table.
 *
 * AAC Icecast is deliberately station 0 for the decoder-vs-hot-swap bisection:
 * pipeline_init() now creates the initial decoder to match station 0's codec, so
 * this AAC stream plays on a fresh AAC decoder with NO live MP3->AAC hot-swap.
 *   - If station 0 (AAC, no swap) is CLEAN but later AAC is corrupt -> the bug is
 *     in the hot-swap path.
 *   - If station 0 (AAC, no swap) is still corrupt -> the bug is the AAC decoder
 *     or the source stream itself, independent of the swap. */
static const station_t TEST_STATIONS[NUM_TEST_STATIONS] = {
    { "AAC Icecast (boot)",  STATION_AAC_URL        },
    { "MP3 Icecast",         STATION_MP3_URL        },
    { "France Inter AAC",    STATION_HLS_URL        },
    { "France Culture AAC",  STATION_HLS_MULTI_URL  },
};

static int s_current_station = 0;

/* Consecutive error count for the current station.
 * After 3 errors we skip to the next rather than looping forever — needed for
 * permanent failures like HTTP 4xx or CDN hangs (BBC 410, HLS .ts timeout). */
static int s_station_error_count = 0;

/* Decoded-PCM format watcher state (see run_event_loop). Tracks the last decoder
 * format seen so the resampler is re-armed only when the rate/channels change.
 * File-scope so restart_pipeline_station() can clear it: a station change resets
 * the pipeline elements (reverting the resampler toward its 44100 Hz default), so
 * the watcher must forget the previous format and re-push — even when the new
 * station shares the same sample rate (e.g. 48000 Hz AAC -> 48000 Hz AAC). */
static int s_fmt_logged_rate = -1;
static int s_fmt_logged_ch   = -1;

/* Format-watcher debounce: the AAC decoder reports a transient sample_rate on its
 * very first getinfo() poll (log 64: 44100 Hz before settling to 48000 Hz), which
 * triggers a spurious resampler rebuild and TLS reconnect.  Require the same rate
 * to be seen on 2 consecutive polls (~1 s at the 500 ms listen timeout) before
 * rebuilding.  Cleared on every station change so a fresh stream starts clean. */
static int s_fmt_pending_rate  = -1;
static int s_fmt_pending_ch    = -1;
static int s_fmt_pending_count =  0;
#define FMT_DEBOUNCE_POLLS 2

/* Restart streaming on a (possibly new) URL, clearing the format watcher first so
 * the resampler is reconfigured for the new stream. ALL station-change / restart
 * paths must go through this rather than calling pipeline_change_station directly,
 * otherwise a same-sample-rate transition would leave the resampler misconfigured. */
static esp_err_t restart_pipeline_station(const char *url)
{
    s_fmt_logged_rate   = -1;
    s_fmt_logged_ch     = -1;
    s_fmt_pending_rate  = -1;
    s_fmt_pending_ch    = -1;
    s_fmt_pending_count =  0;
    return pipeline_change_station(url);
}

/* Phase D — hot station change, keeps A2DP alive.
 * Must be called from the event loop thread to avoid racing with queued events. */
#if CONFIG_PROTOTYPE_PHASE_D_ROTATION
/* Pending station index set by timer callback, consumed in event loop thread */
static volatile int s_next_station_request = -1;
static esp_timer_handle_t s_rotation_timer  = NULL;
static int s_rotation_next_idx = 1;  /* skip idx 0 — already playing at start */

static void rotation_timer_cb(void *arg)
{
    if (s_rotation_next_idx < NUM_TEST_STATIONS) {
        s_next_station_request = s_rotation_next_idx++;
    }
    if (s_rotation_next_idx >= NUM_TEST_STATIONS) {
        esp_timer_stop(s_rotation_timer);
    }
}

static void start_rotation_timer(void)
{
    esp_timer_create_args_t args = {
        .callback = rotation_timer_cb,
        .name     = "phase_d_rot",
    };
    if (esp_timer_create(&args, &s_rotation_timer) == ESP_OK) {
        /* Fire every 30 s; first switch happens 30 s after start */
        esp_timer_start_periodic(s_rotation_timer, 30ULL * 1000 * 1000);
        ESP_LOGI(TAG, "Phase D: rotation timer started (30 s intervals)");
    } else {
        ESP_LOGE(TAG, "Failed to create rotation timer");
    }
}

static void do_switch_to_station(int idx)
{
    if (idx < 0 || idx >= NUM_TEST_STATIONS) return;
    ESP_LOGI(TAG, "Switching to station %d: %s", idx, TEST_STATIONS[idx].name);
    s_station_error_count = 0;
    int64_t t0 = esp_timer_get_time();
    esp_err_t ret = restart_pipeline_station(TEST_STATIONS[idx].url);
    if (ret == ESP_OK) {
        s_current_station = idx;
    }
    ESP_LOGI(TAG, "Station switch took %" PRId64 " ms", (esp_timer_get_time() - t0) / 1000);
}
#endif /* CONFIG_PROTOTYPE_PHASE_D_ROTATION */

/* Pipeline event loop — handles all ADF element/pipeline events.
 * Station rotation (Phase D) is also driven from here to avoid racing with
 * queued element events after a decoder hot-swap. */
static void run_event_loop(void)
{
    audio_event_iface_handle_t evt = pipeline_get_event_iface();
    if (!evt) {
        ESP_LOGE(TAG, "Event interface is NULL — pipeline did not initialize");
        return;
    }

    /* A2DP stall detection: if no PCM bytes flow for 20 s, the A2DP connection
     * failed silently (a2dp_stream does not propagate SDP/L2CAP errors to the
     * pipeline event queue). Retry the connection and restart the pipeline. */
    uint64_t stall_last_bytes = 0;
    int64_t  stall_since_us   = esp_timer_get_time();

    /* Decoded-PCM format watcher. The REPORT_MUSIC_INFO event only reliably
     * fires for the very first stream (it is swallowed for later stations after
     * a decoder hot-swap), so the AAC stations never logged their decoded
     * format. Corrupt audio with clean delivery (choppy / white noise but no
     * underflows) is classically a sample-rate or channel mismatch handed to
     * the SBC encoder, so log the format directly from the decoder whenever it
     * changes. State lives at file scope (s_fmt_logged_*) so it can be cleared
     * on every station change via restart_pipeline_station(). */

    ESP_LOGI(TAG, "Entering event loop …");
    while (true) {
        audio_event_iface_msg_t msg;
        /* Use a short timeout so we can service station-rotation requests from
         * the timer callback without a separate task that could race with us. */
        esp_err_t ret = audio_event_iface_listen(evt, &msg, pdMS_TO_TICKS(500));

#if CONFIG_PROTOTYPE_PHASE_D_ROTATION
        /* Check for pending station rotation (set by timer, consumed here) */
        int next = s_next_station_request;
        if (next >= 0) {
            s_next_station_request = -1;
            do_switch_to_station(next);
        }
#endif

        /* A2DP stall detection — retry if no PCM is produced for 20 s.
         * Track the decoder element's byte_pos: it advances as PCM is produced
         * and freezes on both a network stall (input starvation) and an A2DP
         * stall (downstream ring-buffer backpressure). */
        {
            audio_element_handle_t dec_el = pipeline_get_decoder_el();
            if (dec_el != NULL) {
                if (audio_element_get_state(dec_el) == AEL_STATE_RUNNING) {
                    audio_element_info_t ai = {0};
                    audio_element_getinfo(dec_el, &ai);
                    if (ai.sample_rates > 0 && ai.channels > 0 &&
                        (ai.sample_rates != s_fmt_logged_rate ||
                         ai.channels    != s_fmt_logged_ch)) {
                        /* Debounce: the AAC decoder's first getinfo() often reports
                         * a transient rate (e.g. 44100 Hz) before settling on the
                         * real one (48000 Hz).  Acting on the first poll triggers a
                         * spurious resampler rebuild + TLS reconnect (log 64).
                         * Require FMT_DEBOUNCE_POLLS consecutive polls at the same
                         * rate before pushing to the resampler. */
                        if (ai.sample_rates == s_fmt_pending_rate &&
                            ai.channels     == s_fmt_pending_ch) {
                            s_fmt_pending_count++;
                        } else {
                            s_fmt_pending_rate  = ai.sample_rates;
                            s_fmt_pending_ch    = ai.channels;
                            s_fmt_pending_count = 1;
                        }
                        if (s_fmt_pending_count >= FMT_DEBOUNCE_POLLS) {
                            s_fmt_logged_rate = ai.sample_rates;
                            s_fmt_logged_ch   = ai.channels;
                            ESP_LOGW(TAG, "Decoder PCM format — codec:%d  sample_rate:%d  channels:%d  bits:%d",
                                     ai.codec_fmt, ai.sample_rates, ai.channels, ai.bits);
                            /* Push the decoder's real output rate to the resampler.
                             * rsp_filter cannot learn it on its own (elements only
                             * exchange raw PCM via ring buffers), so without this the
                             * resampler stays at its 44100 Hz default and 1:1 passes
                             * 48000 Hz AAC through — the very mismatch this stage is
                             * meant to remove.  This polling watcher is the reliable
                             * hook: REPORT_MUSIC_INFO is swallowed after a hot-swap. */
                            pipeline_set_resample_src_info(ai.sample_rates, ai.channels);
                        }
                    }
                    uint64_t cur_bytes = (uint64_t)ai.byte_pos;
                    if (cur_bytes != stall_last_bytes) {
                        stall_last_bytes = cur_bytes;
                        stall_since_us   = esp_timer_get_time();
                    } else if ((esp_timer_get_time() - stall_since_us) > 20LL * 1000000) {
                        stall_since_us   = esp_timer_get_time();
                        stall_last_bytes = 0;
                        if (bt_manager_is_a2dp_connected()) {
                            /* A2DP up but PCM stalled — network stall or broken station.
                             * Count stalls the same way as element errors: after 3
                             * consecutive stalls on the same station assume it is
                             * permanently dead (e.g. decommissioned CDN URL) and advance
                             * to the next station instead of retrying forever (log 64:
                             * CBC Akamai hung the rotation indefinitely). */
                            s_station_error_count++;
                            if (s_station_error_count >= 3) {
                                s_station_error_count = 0;
                                s_current_station = (s_current_station + 1) % NUM_TEST_STATIONS;
                                ESP_LOGW(TAG, "Station stalled 3× — advancing to station %d: %s",
                                         s_current_station, TEST_STATIONS[s_current_station].name);
                            } else {
                                ESP_LOGW(TAG, "No PCM for 20 s (A2DP connected) — restarting pipeline (%d/3)",
                                         s_station_error_count);
                            }
                            restart_pipeline_station(TEST_STATIONS[s_current_station].url);
                        } else {
                            /* A2DP link is down — re-page the sink.  bt_manager
                             * owns the connection lifecycle, so drive a single
                             * connect attempt here (the pending guard dedups any
                             * overlap with an in-flight attempt). */
                            ESP_LOGW(TAG, "No PCM for 20 s (A2DP disconnected) — re-paging sink");
                            bt_manager_connect_a2dp();
                        }
                    }
                } else {
                    /* Reset stall timer when pipeline is not actively running */
                    stall_since_us   = esp_timer_get_time();
                    stall_last_bytes = 0;
                }
            }
        }

        if (ret != ESP_OK) {
            /* ESP_ERR_TIMEOUT is normal — just loop for next event or rotation check */
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
                s_station_error_count = 0;
                ESP_LOGI(TAG, "Stream finished — restarting same station");
                restart_pipeline_station(TEST_STATIONS[s_current_station].url);
            } else if (st == AEL_STATUS_ERROR_OPEN   ||
                       st == AEL_STATUS_ERROR_INPUT  ||
                       st == AEL_STATUS_ERROR_PROCESS) {
                s_station_error_count++;
                if (s_station_error_count >= 3) {
                    /* Permanent failure (e.g. HTTP 4xx, CDN timeout): skip to
                     * next station rather than spinning forever on a dead URL. */
                    s_station_error_count = 0;
                    s_current_station = (s_current_station + 1) % NUM_TEST_STATIONS;
                    ESP_LOGW(TAG, "Station failed 3× — advancing to station %d: %s",
                             s_current_station, TEST_STATIONS[s_current_station].name);
                } else {
                    ESP_LOGE(TAG, "Stream error (status=%d) — retrying in 3 s (%d/3)",
                             st, s_station_error_count);
                    vTaskDelay(pdMS_TO_TICKS(3000));
                }
                restart_pipeline_station(TEST_STATIONS[s_current_station].url);
            }
        }
    }
}

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

    /* ---- UART Provisioning (Web Serial after flash) ---- */
    ESP_ERROR_CHECK(provisioning_start());

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

    /* BT/WiFi coexistence arbiter — PREFER_BT.
     *
     * Log 30 (PR #24, BALANCE) was a regression: it reintroduced the BT_L2CAP
     * is_cong bursts that log 29 (PREFER_BT) had eliminated to zero (20 events
     * vs 0), while steady-state HTTP throughput stayed pinned at ~12 KB/s —
     * IDENTICAL to log 29. The coex preference does not move throughput because
     * throughput is NOT radio-arbitration-limited: it is CPU-bound. The decode
     * +SBC-encode chain scales linearly with clock (0.56x real-time at 160 MHz
     * -> 0.84x at 240 MHz), and Core 0 (WiFi + BT controller + Bluedroid SBC
     * encode + coex) is the saturated resource. BALANCE gave WiFi more Core-0
     * airtime, which only crowded out BT's SBC path -> congestion returned with
     * no throughput benefit. PREFER_BT keeps WiFi's Core-0 burden low (zero
     * congestion in log 29) while we attack the actual CPU ceiling separately.
     *
     * Must be called after bt_manager_init() so the BT controller is registered
     * with the coexistence framework before the preference takes effect. */
    ret = esp_coex_preference_set(ESP_COEX_PREFER_BT);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_coex_preference_set(BT) failed: %d (balance stays)", ret);
    }

    ret = bt_manager_find_peer(config_get_bt_mac(), 30);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No BT peer found — cannot start A2DP");
        ESP_LOGE(TAG, "Set BT_PEER_ADDR in station_config.h or enable a discoverable A2DP sink");
        /* Halt — without a BT peer the prototype cannot proceed */
        while (true) vTaskDelay(pdMS_TO_TICKS(5000));
    }

    /* ---- Phase B / C: Pipeline init ---- */
    ESP_LOGI(TAG, "Heap pre-pipeline: internal=%"PRIu32"B  SPIRAM=%"PRIu32"B",
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_ERROR_CHECK(pipeline_init(bt_manager_get_peer_bda(), TEST_STATIONS[0].url));  /* boot decoder matches station 0 */

    /* ---- Bring up A2DP BEFORE streaming ----
     * pipeline_init() has initialized the A2DP source profile (via a2dp_stream).
     * Page the speaker now, while WiFi is idle: doing the BR/EDR page
     * concurrently with the HTTP stream starves it under BT/WiFi coexistence
     * and the page times out (~5 s → SDP conn cnf 0x4 / BTA_AV_OPEN status 2).
     * Retry across the sink's intermittent page scan. */
    ret = bt_manager_connect_a2dp_blocking(10);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "A2DP not connected before streaming — starting anyway; "
                      "stall detector will keep retrying the connection");
    }

    /* ---- Phase C: Resource monitoring ---- */
    monitor_init();
    monitor_start();

    /* ---- Phase B: Start first station (AAC — boots on a fresh AAC decoder) ---- */
    ESP_LOGI(TAG, "Starting Phase B — station 0: %s", TEST_STATIONS[0].name);
    int64_t t_start = esp_timer_get_time();
    ESP_ERROR_CHECK(pipeline_start(TEST_STATIONS[0].url));
    ESP_LOGI(TAG, "Pipeline start latency: %" PRId64 " ms",
             (esp_timer_get_time() - t_start) / 1000);

    /* ---- Phase D: station rotation timer (runs inside event loop thread) ---- */
    #if CONFIG_PROTOTYPE_PHASE_D_ROTATION
    start_rotation_timer();
    #endif

    run_event_loop();
    /* Not reached in normal operation */
    pipeline_deinit();
}
