#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "bt_manager.h"

static const char *TAG = "bt_mgr";

#define BT_FOUND_BIT      BIT0
#define A2DP_CONN_BIT     BIT1

static EventGroupHandle_t s_bt_event_group;
static uint8_t  s_peer_bda[6] = {0};
static uint32_t s_a2dp_connected = 0;
static uint32_t s_a2dp_connect_pending = 0;

/* ---- AVRC controller callback (stub — we don't need remote-control events) ---- */

static void avrc_ct_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    (void)event;
    (void)param;
}

/* ---- GAP callback ---- */

static void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    if (!param) {
        ESP_LOGW(TAG, "GAP callback with NULL param");
        return;
    }

    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        /* Log every discovered device */
        char bda_str[18];
        uint8_t *bda = param->disc_res.bda;
        if (!bda) {
            ESP_LOGW(TAG, "Disc result with NULL BDA");
            break;
        }
        snprintf(bda_str, sizeof(bda_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        ESP_LOGI(TAG, "Found device: %s", bda_str);

        /* Check EIR/UUIDs for A2DP sink (UUID 0x110B).
         * IDF 5.x: esp_bt_gap_resolve_eir_data() returns a pointer to the
         * UUID list (or NULL) and writes the byte-length into rlen. */
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
                    /* 0x110B = A2DP Sink */
                    for (int j = 0; j + 1 < rlen; j += 2) {
                        uint16_t u = uuids[j] | (uuids[j + 1] << 8);
                        if (u == 0x110B) {
                            ESP_LOGI(TAG, "A2DP Sink found: %s", bda_str);
                            memcpy(s_peer_bda, bda, 6);
                            esp_bt_gap_cancel_discovery();
                            if (s_bt_event_group) {
                                xEventGroupSetBits(s_bt_event_group, BT_FOUND_BIT);
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
            ESP_LOGW(TAG, "Auth failed: %d", param->auth_cmpl.stat);
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
        /* Passkey entry — send 0000 (legacy fallback) */
        ESP_LOGI(TAG, "SSP passkey request — entering 0000");
        esp_bt_gap_ssp_passkey_reply(param->key_req.bda, true, 0);
        break;
    default:
        break;
    }
}

/* ---- A2DP callback ---- */

static void a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    if (!param) {
        ESP_LOGW(TAG, "A2DP callback with NULL param");
        return;
    }

    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
            ESP_LOGI(TAG, "A2DP connecting");
            __atomic_store_n(&s_a2dp_connect_pending, 1u, __ATOMIC_SEQ_CST);
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            ESP_LOGI(TAG, "A2DP connected");
            __atomic_store_n(&s_a2dp_connected, 1u, __ATOMIC_SEQ_CST);
            __atomic_store_n(&s_a2dp_connect_pending, 0u, __ATOMIC_SEQ_CST);
            if (s_bt_event_group) {
                xEventGroupSetBits(s_bt_event_group, A2DP_CONN_BIT);
            }
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGW(TAG, "A2DP disconnected");
            __atomic_store_n(&s_a2dp_connected, 0u, __ATOMIC_SEQ_CST);
            __atomic_store_n(&s_a2dp_connect_pending, 0u, __ATOMIC_SEQ_CST);
            if (s_bt_event_group) {
                xEventGroupClearBits(s_bt_event_group, A2DP_CONN_BIT);
            }
        }
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP audio state: %d", param->audio_stat.state);
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
        ESP_LOGI(TAG, "A2DP audio cfg: type=%d", param->audio_cfg.mcc.type);
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

    /* Release BLE memory — we only use Classic BT */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_callback));

    /* IO capability = None → forces "Just Works" SSP (no PIN/confirmation needed).
     * Must be set before any connection attempt so the speaker accepts us. */
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    ESP_ERROR_CHECK(esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap)));

    /* Legacy PIN fallback for older speakers that use PIN-based pairing.
     * esp_bt_pin_code_t is uint8_t[16]; must pass a properly-sized array. */
    esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin);

    /* AVRC must be initialized before A2DP — many speakers check for AVRC
     * in our SDP record and refuse the A2DP connection if it's missing. */
    ESP_ERROR_CHECK(esp_avrc_ct_init());
    ESP_ERROR_CHECK(esp_avrc_ct_register_callback(avrc_ct_callback));

    ESP_ERROR_CHECK(esp_a2d_register_callback(a2dp_callback));
    ESP_ERROR_CHECK(esp_a2d_source_init());

    esp_bt_gap_set_device_name(device_name);

    /* Make discoverable + connectable */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

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

    /* If a non-zero address was provided, use it directly */
    if (memcmp(peer_bda, zero_bda, 6) != 0) {
        memcpy(s_peer_bda, peer_bda, 6);
        ESP_LOGI(TAG, "Using configured peer BDA");
        xEventGroupSetBits(s_bt_event_group, BT_FOUND_BIT);
        return ESP_OK;
    }

    /* GAP discovery duration is limited to 30s by the API.
     * esp_bt_gap_start_discovery() duration parameter is in 1.28s units. */
    int discovery_duration_s = (timeout_s > 30) ? 30 : (int)timeout_s;
    /* Convert seconds to 1.28s units for the API: duration * 1000 / 1280 = duration * 0.78125 */
    uint8_t discovery_duration_units = (uint8_t)(discovery_duration_s * 1000 / 1280);
    if (discovery_duration_units == 0) discovery_duration_units = 1;
    ESP_LOGI(TAG, "Scanning for A2DP sinks (%d s, %d units)...", discovery_duration_s, discovery_duration_units);
    xEventGroupClearBits(s_bt_event_group, BT_FOUND_BIT);
    ESP_ERROR_CHECK(esp_bt_gap_start_discovery(
        ESP_BT_INQ_MODE_GENERAL_INQUIRY,
        discovery_duration_units,
        0));

    EventBits_t bits = xEventGroupWaitBits(s_bt_event_group,
                                           BT_FOUND_BIT, pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(discovery_duration_s * 1000));
    if (!(bits & BT_FOUND_BIT)) {
        ESP_LOGE(TAG, "No A2DP sink found within %d s", discovery_duration_s);
        esp_bt_gap_cancel_discovery();  // Clean up discovery on timeout
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

const uint8_t *bt_manager_get_peer_bda(void)
{
    return s_peer_bda;
}

esp_err_t bt_manager_reconnect_a2dp(void)
{
    static const uint8_t zero_bda[6] = {0};
    if (memcmp(s_peer_bda, zero_bda, 6) == 0) {
        ESP_LOGW(TAG, "reconnect_a2dp: no peer BDA configured");
        return ESP_ERR_INVALID_STATE;
    }
    /* Acquire the pending lock first; then double-check connected under the lock.
     * This closes the window where a CONNECTED callback fires between the
     * connected check and the exchange, which would leave pending=1 and trigger
     * a duplicate esp_a2d_source_connect() on an already-live link. */
    if (__atomic_exchange_n(&s_a2dp_connect_pending, 1u, __ATOMIC_SEQ_CST)) {
        ESP_LOGD(TAG, "A2DP connect already pending — skipping duplicate");
        return ESP_OK;
    }
    if (__atomic_load_n(&s_a2dp_connected, __ATOMIC_SEQ_CST)) {
        ESP_LOGD(TAG, "A2DP already connected — skipping reconnect");
        __atomic_store_n(&s_a2dp_connect_pending, 0u, __ATOMIC_SEQ_CST);
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Connecting A2DP to %02X:%02X:%02X:%02X:%02X:%02X",
             s_peer_bda[0], s_peer_bda[1], s_peer_bda[2],
             s_peer_bda[3], s_peer_bda[4], s_peer_bda[5]);
    esp_err_t err = esp_a2d_source_connect(s_peer_bda);
    if (err != ESP_OK) {
        __atomic_store_n(&s_a2dp_connect_pending, 0u, __ATOMIC_SEQ_CST);
    }
    return err;
}

bool bt_manager_is_a2dp_connected(void)
{
    return __atomic_load_n(&s_a2dp_connected, __ATOMIC_SEQ_CST);
}
