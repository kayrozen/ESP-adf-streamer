#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "bt_manager.h"

static const char *TAG = "bt_mgr";

#define BT_FOUND_BIT      BIT0
#define A2DP_CONN_BIT     BIT1
/* Set when a connect attempt fails its open/page (BTA_AV_OPEN status 2),
 * surfaced as a DISCONNECTED event while a connect is still pending.  Lets the
 * blocking connector retry the instant the stack reports failure instead of
 * waiting out the rest of its (deliberately generous) per-attempt timeout. */
#define A2DP_FAIL_BIT     BIT2

/* Per-attempt budget for a full A2DP source connection: the BR/EDR page alone
 * can take ~5.12 s (8192 slots * 0.625 ms), and SDP + AVDTP signalling follow.
 * Must comfortably exceed the whole sequence or we abort a handshake that is
 * still in flight. */
#define A2DP_CONNECT_TIMEOUT_MS   8000

static EventGroupHandle_t s_bt_event_group;
static uint8_t  s_peer_bda[6] = {0};
static uint32_t s_a2dp_connected = 0;
static uint32_t s_a2dp_connect_pending = 0;
/* esp_timer timestamp (us) when s_a2dp_connect_pending was last set.  Written
 * only by the thread that claims the pending lock; used to detect a connect
 * that reached CONNECTING but whose terminal CONNECTED/DISCONNECTED event was
 * lost, so a stuck flag can never wedge reconnection forever. */
static volatile int64_t s_a2dp_pending_since_us = 0;
/* Set while bt_manager_find_peer() is actively scanning so the GAP callback
 * knows not to interfere with any stack-internal inquiry. */
static uint32_t s_find_peer_active = 0;
/* Set by the GAP auth-fail handler when pairing with s_peer_bda fails due to a
 * link-key mismatch; consumed before the next connect so a stale bond is
 * removed only when it actually blocked us, not on every attempt. */
static uint32_t s_clear_bond_on_connect = 0;

/* ---- GAP callback ----
 * Ownership note: bt_manager owns the controller, Bluedroid, GAP and SSP.  The
 * A2DP profile (esp_a2d_source_init, the esp_a2d callback and the PCM data
 * callback) is owned by the ADF a2dp_stream element because the audio data must
 * originate from the pipeline.  bt_manager observes A2DP state through
 * bt_manager_a2dp_state_cb, which pipeline.c registers as a2dp_stream's
 * user_a2d_cb, and drives the connection lifecycle from here. */

static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    if (!param) {
        ESP_LOGW(TAG, "GAP callback with NULL param");
        return;
    }

    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        char bda_str[18];
        uint8_t *bda = param->disc_res.bda;
        if (!bda) {
            ESP_LOGW(TAG, "Disc result with NULL BDA");
            break;
        }
        snprintf(bda_str, sizeof(bda_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        ESP_LOGI(TAG, "Found device: %s", bda_str);

        /* Only act on discovery results while WE are scanning for a peer.
         * Check EIR/UUIDs for the A2DP Sink service (UUID 0x110B). */
        if (!__atomic_load_n(&s_find_peer_active, __ATOMIC_SEQ_CST)) {
            break;
        }
        for (int i = 0; i < param->disc_res.num_prop; i++) {
            esp_bt_gap_dev_prop_t *p = &param->disc_res.prop[i];
            if (p && p->type == ESP_BT_GAP_DEV_PROP_EIR && p->val) {
                uint8_t *eir  = p->val;
                uint8_t  rlen = 0;
                uint8_t *uuids = esp_bt_gap_resolve_eir_data(
                    eir, ESP_BT_EIR_TYPE_INCMPL_16BITS_UUID, &rlen);
                if (!uuids) {
                    uuids = esp_bt_gap_resolve_eir_data(
                        eir, ESP_BT_EIR_TYPE_CMPL_16BITS_UUID, &rlen);
                }
                if (uuids && rlen >= 2) {
                    for (int j = 0; j + 1 < rlen; j += 2) {
                        uint16_t u = uuids[j] | (uuids[j + 1] << 8);
                        if (u == 0x110B) {  /* A2DP Sink */
                            /* Claim-and-clear so only the first match wins; later
                             * results arriving before cancel takes effect are
                             * ignored (cancel_discovery is asynchronous). */
                            if (__atomic_exchange_n(&s_find_peer_active, 0u, __ATOMIC_SEQ_CST)) {
                                ESP_LOGI(TAG, "A2DP Sink found: %s", bda_str);
                                memcpy(s_peer_bda, bda, 6);
                                esp_bt_gap_cancel_discovery();
                                if (s_bt_event_group) {
                                    xEventGroupSetBits(s_bt_event_group, BT_FOUND_BIT);
                                }
                            }
                            return;
                        }
                    }
                }
            }
        }
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(TAG, "GAP discovery stopped");
        }
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Auth OK with: %s",
                     param->auth_cmpl.device_name ? (char *)param->auth_cmpl.device_name : "unknown");
        } else {
            ESP_LOGW(TAG, "Auth failed (stat=%d)", param->auth_cmpl.stat);
            /* Schedule a bond removal for our sink only; performed lazily before
             * the next connect so we never spam NVS on transient failures. */
            if (memcmp(param->auth_cmpl.bda, s_peer_bda, 6) == 0) {
                __atomic_store_n(&s_clear_bond_on_connect, 1u, __ATOMIC_SEQ_CST);
            }
        }
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:
        /* SSP numeric comparison — auto-accept (Just Works, no display/keyboard) */
        ESP_LOGI(TAG, "SSP confirm request — auto-accepting");
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "SSP passkey: %06lu", (unsigned long)param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "SSP passkey request — entering 0000");
        esp_bt_gap_ssp_passkey_reply(param->key_req.bda, true, 0);
        break;
    default:
        break;
    }
}

/* ---- A2DP connection-state observer ----
 * Invoked by a2dp_stream as its user_a2d_cb.  Translates esp_a2d connection
 * events into the event-group bits the connection driver waits on. */
void bt_manager_a2dp_state_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    if (!param) {
        ESP_LOGW(TAG, "A2DP callback with NULL param");
        return;
    }

    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        switch (param->conn_stat.state) {
        case ESP_A2D_CONNECTION_STATE_CONNECTING:
            ESP_LOGI(TAG, "A2DP connecting");
            __atomic_store_n(&s_a2dp_connect_pending, 1u, __ATOMIC_SEQ_CST);
            /* Fresh attempt under way — drop any failure flag left by a prior
             * attempt so the connector's wait can't trip on a stale signal. */
            if (s_bt_event_group) {
                xEventGroupClearBits(s_bt_event_group, A2DP_FAIL_BIT);
            }
            break;
        case ESP_A2D_CONNECTION_STATE_CONNECTED:
            ESP_LOGI(TAG, "A2DP connected");
            __atomic_store_n(&s_a2dp_connected, 1u, __ATOMIC_SEQ_CST);
            __atomic_store_n(&s_a2dp_connect_pending, 0u, __ATOMIC_SEQ_CST);
            if (s_bt_event_group) {
                xEventGroupSetBits(s_bt_event_group, A2DP_CONN_BIT);
            }
            break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
            ESP_LOGI(TAG, "A2DP disconnecting");
            break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
        default: {
            /* A DISCONNECTED arriving while a connect is still pending means the
             * open/page failed (BTA_AV_OPEN status 2), not a normal teardown.
             * Distinguish the two so the blocking connector can retry at once on
             * a failed open but ignore steady-state disconnects. */
            uint32_t was_pending =
                __atomic_exchange_n(&s_a2dp_connect_pending, 0u, __ATOMIC_SEQ_CST);
            ESP_LOGW(TAG, "A2DP disconnected");
            __atomic_store_n(&s_a2dp_connected, 0u, __ATOMIC_SEQ_CST);
            if (s_bt_event_group) {
                xEventGroupClearBits(s_bt_event_group, A2DP_CONN_BIT);
                if (was_pending) {
                    xEventGroupSetBits(s_bt_event_group, A2DP_FAIL_BIT);
                }
            }
            break;
        }
        }
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP audio state: %d", param->audio_stat.state);
        break;
    case ESP_A2D_PROF_STATE_EVT:
        ESP_LOGI(TAG, "A2DP profile state: %d", param->a2d_prof_stat.init_state);
        break;
    default:
        break;
    }
}

/* ---- Public API ---- */

esp_err_t bt_manager_init(const char *device_name)
{
    if (!device_name) {
        ESP_LOGE(TAG, "device_name cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_bt_event_group = xEventGroupCreate();
    if (!s_bt_event_group) {
        ESP_LOGE(TAG, "Failed to create BT event group");
        return ESP_ERR_NO_MEM;
    }

    /* Release BLE memory — Classic BT only */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    /* GAP is owned here; the A2DP profile + callbacks are owned by a2dp_stream. */
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_callback));

    /* IO capability = None → "Just Works" SSP (no PIN/confirmation). */
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    ESP_ERROR_CHECK(esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap)));

    /* Legacy PIN fallback for older PIN-based speakers. */
    esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin);

    esp_bt_gap_set_device_name(device_name);

    /* Match the canonical ESP-IDF a2dp_source example: an A2DP SOURCE always
     * initiates, so it neither advertises (discoverable) nor accepts incoming
     * pages (connectable).  Being connectable previously let the idle speaker
     * try to initiate back to us mid-page and contend for the link. */
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

    ESP_LOGI(TAG, "BT initialized as \"%s\"", device_name);
    return ESP_OK;
}

esp_err_t bt_manager_find_peer(const uint8_t peer_bda[6], uint32_t timeout_s)
{
    if (!peer_bda) {
        ESP_LOGE(TAG, "peer_bda cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    static const uint8_t zero_bda[6] = {0};

    /* Configured address provided → use it directly, no inquiry. */
    if (memcmp(peer_bda, zero_bda, 6) != 0) {
        memcpy(s_peer_bda, peer_bda, 6);
        ESP_LOGI(TAG, "Using configured peer BDA");
        xEventGroupSetBits(s_bt_event_group, BT_FOUND_BIT);
        return ESP_OK;
    }

    int discovery_duration_s = (timeout_s > 30) ? 30 : (int)timeout_s;
    uint8_t discovery_duration_units = (uint8_t)(discovery_duration_s * 1000 / 1280);
    if (discovery_duration_units == 0) discovery_duration_units = 1;
    ESP_LOGI(TAG, "Scanning for A2DP sinks (%d s, %d units)...", discovery_duration_s, discovery_duration_units);
    xEventGroupClearBits(s_bt_event_group, BT_FOUND_BIT);
    __atomic_store_n(&s_find_peer_active, 1u, __ATOMIC_SEQ_CST);
    ESP_ERROR_CHECK(esp_bt_gap_start_discovery(
        ESP_BT_INQ_MODE_GENERAL_INQUIRY,
        discovery_duration_units,
        0));

    EventBits_t bits = xEventGroupWaitBits(s_bt_event_group,
                                           BT_FOUND_BIT, pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(discovery_duration_s * 1000));
    __atomic_store_n(&s_find_peer_active, 0u, __ATOMIC_SEQ_CST);
    if (!(bits & BT_FOUND_BIT)) {
        ESP_LOGE(TAG, "No A2DP sink found within %d s", discovery_duration_s);
        esp_bt_gap_cancel_discovery();
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

const uint8_t *bt_manager_get_peer_bda(void)
{
    return s_peer_bda;
}

/* Fire a single esp_a2d_source_connect() to the configured peer.  Returns
 * ESP_OK if the request was accepted by the stack (connection completes
 * asynchronously via bt_manager_a2dp_state_cb), not that the link is up. */
esp_err_t bt_manager_connect_a2dp(void)
{
    static const uint8_t zero_bda[6] = {0};
    if (memcmp(s_peer_bda, zero_bda, 6) == 0) {
        ESP_LOGW(TAG, "connect: no peer BDA configured");
        return ESP_ERR_INVALID_STATE;
    }
    if (__atomic_load_n(&s_a2dp_connected, __ATOMIC_SEQ_CST)) {
        return ESP_OK;  /* already up */
    }
    /* Claim the pending lock.  If it was already held, only honour it while it
     * is fresh: a connect that reached CONNECTING but whose terminal event was
     * lost would otherwise leave the flag stuck at 1 forever, turning every
     * future reconnect attempt (e.g. from the stall detector) into a no-op. */
    int64_t now = esp_timer_get_time();
    if (__atomic_exchange_n(&s_a2dp_connect_pending, 1u, __ATOMIC_SEQ_CST)) {
        if ((now - s_a2dp_pending_since_us) < (int64_t)A2DP_CONNECT_TIMEOUT_MS * 1000) {
            ESP_LOGD(TAG, "A2DP connect already pending — skipping duplicate");
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Stale pending connect (lost terminal event) — re-issuing");
    }
    s_a2dp_pending_since_us = now;

    /* Re-check connected under the lock to close the window where a CONNECTED
     * callback fires between the fast-path check above and the exchange. */
    if (__atomic_load_n(&s_a2dp_connected, __ATOMIC_SEQ_CST)) {
        __atomic_store_n(&s_a2dp_connect_pending, 0u, __ATOMIC_SEQ_CST);
        return ESP_OK;
    }
    /* Clear a stale bond only if a prior attempt failed authentication. */
    if (__atomic_exchange_n(&s_clear_bond_on_connect, 0u, __ATOMIC_SEQ_CST)) {
        ESP_LOGW(TAG, "Clearing stale bond after prior auth failure");
        esp_bt_gap_remove_bond_device(s_peer_bda);
    }

    ESP_LOGI(TAG, "Connecting A2DP to %02X:%02X:%02X:%02X:%02X:%02X",
             s_peer_bda[0], s_peer_bda[1], s_peer_bda[2],
             s_peer_bda[3], s_peer_bda[4], s_peer_bda[5]);
    esp_err_t err = esp_a2d_source_connect(s_peer_bda);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_a2d_source_connect failed: %s", esp_err_to_name(err));
        __atomic_store_n(&s_a2dp_connect_pending, 0u, __ATOMIC_SEQ_CST);
    }
    return err;
}

esp_err_t bt_manager_connect_a2dp_blocking(int max_attempts)
{
    static const uint8_t zero_bda[6] = {0};
    if (memcmp(s_peer_bda, zero_bda, 6) == 0) {
        ESP_LOGE(TAG, "connect: no peer BDA configured");
        return ESP_ERR_INVALID_STATE;
    }
    if (max_attempts < 1) max_attempts = 1;

    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        if (__atomic_load_n(&s_a2dp_connected, __ATOMIC_SEQ_CST)) {
            return ESP_OK;
        }

        ESP_LOGI(TAG, "A2DP connect attempt %d/%d", attempt, max_attempts);
        xEventGroupClearBits(s_bt_event_group, A2DP_CONN_BIT | A2DP_FAIL_BIT);

        esp_err_t err = bt_manager_connect_a2dp();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            /* Request was rejected outright (e.g. busy) — back off and retry. */
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* Wake on either outcome: CONNECTED (success) or an explicit open
         * failure (FAIL).  The timeout is the backstop for a page that is still
         * silently in flight — log 33's successful page took ~6.5 s, so the
         * budget must stay generous; we just no longer sit on it after the stack
         * has already told us the open failed. */
        EventBits_t bits = xEventGroupWaitBits(s_bt_event_group,
                                               A2DP_CONN_BIT | A2DP_FAIL_BIT,
                                               pdFALSE, pdFALSE,
                                               pdMS_TO_TICKS(A2DP_CONNECT_TIMEOUT_MS));
        if (bits & A2DP_CONN_BIT) {
            ESP_LOGI(TAG, "A2DP connected on attempt %d", attempt);
            return ESP_OK;
        }

        /* Re-check the flag before tearing anything down: a CONNECTED callback
         * can race in just after xEventGroupWaitBits returns, and we must not
         * disconnect a link that just came up. */
        if (__atomic_load_n(&s_a2dp_connected, __ATOMIC_SEQ_CST)) {
            ESP_LOGI(TAG, "A2DP connected (raced past wait) on attempt %d", attempt);
            return ESP_OK;
        }

        /* Either the stack reported the open failed (FAIL bit, ~5 s page timeout
         * → SDP conn cnf 0x4 / BTA_AV_OPEN status 2) or our own budget expired.
         * Both mean no link; the sink's page scan is intermittent while it sits
         * idle.  Tear down the half-open attempt and retry.  On an explicit
         * failure, retry promptly while the sink is still warm from the page;
         * on a bare timeout, back off a little longer. */
        if (bits & A2DP_FAIL_BIT) {
            ESP_LOGW(TAG, "A2DP attempt %d open failed — re-paging now", attempt);
        } else {
            ESP_LOGW(TAG, "A2DP attempt %d timed out (no connection) — retrying", attempt);
        }
        esp_a2d_source_disconnect(s_peer_bda);
        __atomic_store_n(&s_a2dp_connect_pending, 0u, __ATOMIC_SEQ_CST);
        vTaskDelay(pdMS_TO_TICKS((bits & A2DP_FAIL_BIT) ? 800 : 1500));
    }

    ESP_LOGE(TAG, "A2DP not connected after %d attempts", max_attempts);
    return ESP_ERR_TIMEOUT;
}

bool bt_manager_is_a2dp_connected(void)
{
    return __atomic_load_n(&s_a2dp_connected, __ATOMIC_SEQ_CST);
}
