#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "wifi_manager.h"

static const char *TAG = "wifi_mgr";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_CONNECT_TIMEOUT_MS 15000

static EventGroupHandle_t s_wifi_event_group;
static bool s_initialized = false;
static bool s_connected    = false;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        wifi_event_sta_disconnected_t *ev = data;
        ESP_LOGW(TAG, "Disconnected (reason %d) — reconnecting…", ev->reason);
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi connected to AP");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_AUTHMODE_CHANGE) {
        wifi_event_sta_authmode_change_t *ev = data;
        ESP_LOGW(TAG, "Auth mode change: old=%d new=%d", ev->old_mode, ev->new_mode);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        ESP_LOGW(TAG, "Lost IP address");
        s_connected = false;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_BEACON_TIMEOUT) {
        ESP_LOGW(TAG, "Beacon timeout");
    }
}

esp_err_t wifi_manager_init(void)
{
    if (s_initialized) return ESP_OK;

    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %d", ret);
        return ret;
    }

    ret = esp_event_loop_create_default();
    /* ESP_ERR_INVALID_STATE means another component (e.g. Bluedroid) already
     * created the default loop — that is fine, we can reuse it. */
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %d", ret);
        return ret;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %d", ret);
        return ret;
    }

    ret = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WIFI_EVENT handler register failed: %d", ret);
        return ret;
    }
    ret = esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IP_EVENT handler register failed: %d", ret);
        return ret;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %d", ret);
        return ret;
    }
    s_initialized = true;
    return ESP_OK;
}

esp_err_t wifi_manager_connect(const char *ssid, const char *pass)
{
    if (!ssid) {
        ESP_LOGE(TAG, "SSID cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t wifi_cfg = {0};
    strlcpy((char *)wifi_cfg.sta.ssid,     ssid, sizeof(wifi_cfg.sta.ssid));
    strlcpy((char *)wifi_cfg.sta.password, pass ? pass : "",
            sizeof(wifi_cfg.sta.password));

    /* Set minimum auth mode threshold based on whether password is provided.
     * Open network (no pass) → WIFI_AUTH_OPEN
     * Secured network (pass given) → WIFI_AUTH_WPA_PSK (supports WPA/WPA2/WPA3) */
    if (pass && strlen(pass) > 0) {
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
    } else {
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    /* WiFi listen interval for MAX_MODEM power save.
     *
     * With WIFI_PS_MAX_MODEM and li=N, the station sleeps for N × DTIM periods
     * before waking to check for buffered data.  Our AP's DTIM period = 1 (every
     * beacon, ~102ms), so li=3 → 307ms sleep windows.
     *
     * During each 307ms sleep BT has near-exclusive 2.4 GHz access.  At 192kbps
     * stream rate we need 24KB/s = 7.4KB per 307ms window.  WiFi downloads that
     * in ~7ms at ~1MB/s burst, then sleeps again → BT gets ~97% of airtime.
     * This directly eliminates the startup L2CAP is_cong bursts (log 65: 17
     * events in the first 5s of francemusique, causing audible choppiness).
     *
     * Our ring buffers absorb the bursty WiFi pattern:
     *   HTTP ring buf   64KB  = 2.7s @ 192kbps  (64000/24000)
     *   PCM jitter buf 512KB  = 2.9s @ 176KB/s  (512000/176400)
     *
     * History: li=3 with an earlier codebase (PR #19) was replaced by PS_NONE
     * which kept WiFi awake continuously and starved BT to ~60% A2DP throughput
     * (log 24).  That was then tuned to li=1 + PS_MIN_MODEM.  With PS_MIN_MODEM,
     * the listen_interval field is IGNORED — MIN_MODEM always wakes at every DTIM
     * regardless, so li=1 vs li=3 made no difference.  The buffers are now much
     * larger (PSRAM), so li=3 + MAX_MODEM is safe and gives far more BT airtime. */
    wifi_cfg.sta.listen_interval = 3;

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %d", ret);
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %d", ret);
        return ret;
    }

    /* MAX_MODEM power save: WiFi sleeps for listen_interval (3) × DTIM periods
     * between wakes.  Combined with li=3 above, this gives ~307ms sleep windows
     * where BT has near-exclusive 2.4GHz access, eliminating the startup L2CAP
     * is_cong bursts observed in logs 64/65.
     *
     * PS_NONE (tried in PR #19) starved BT to ~60% airtime (log 24).
     * PS_MIN_MODEM (used previously) ignores listen_interval entirely — it always
     * wakes every DTIM (~102ms) — so li=1 and li=3 were equivalent under it.
     * PS_MAX_MODEM is required for listen_interval > 1 to take effect. */
    ret = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_ps(MAX_MODEM) failed: %d", ret);
    }

    ESP_LOGI(TAG, "Connecting to SSID: %s …", ssid);
    /* Clear stale bits from any previous connect attempt before waiting. */
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to %s", ssid);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "WiFi connect timeout (%d ms)", WIFI_CONNECT_TIMEOUT_MS);
    return ESP_ERR_TIMEOUT;
}

bool wifi_manager_is_connected(void)
{
    return s_connected;
}
