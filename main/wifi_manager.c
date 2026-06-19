#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_coexist.h"
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

    /* Wake on every beacon (li=1, ~102ms) instead of the default li=3 (307ms).
     * With BT/WiFi coexistence and PS_MIN_MODEM, WiFi sleeps between beacons
     * and the coexistence arbiter gives BT the radio during sleep windows.
     * li=1 lets WiFi burst at ~28 KB/s (2×MSS/102ms) — well above the 16 KB/s
     * needed for 128kbps — while sleeping ~70% of the time so BT A2DP gets
     * enough radio time to sustain 192 KB/s PCM (48kHz/stereo/16-bit).
     * li=3 (307ms) was the old default and was replaced by PS_NONE in PR #19,
     * but PS_NONE keeps WiFi awake continuously, starving BT to ~60% airtime
     * and dropping A2DP throughput below real-time regardless of buffer size. */
    wifi_cfg.sta.listen_interval = 1;

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

    /* Keep modem sleep (PS_MIN_MODEM) so WiFi yields the radio to BT between
     * beacon wakes. PS_NONE (tried in PR #19) keeps WiFi awake 100% of the
     * time; the coexistence arbiter then starves BT A2DP to ~60% throughput
     * (log 24 steady-state: 9.5 KB/s HTTP in = 114 KB/s PCM out vs 192 KB/s
     * needed, causing choppy audio despite large PSRAM jitter buffers).
     * With PS_MIN_MODEM + li=1 (102ms wakes), WiFi bursts at ~28 KB/s when
     * awake but sleeps ~70% of the time, leaving BT enough airtime for
     * glitch-free A2DP. PS_MIN_MODEM is IDF default; this call is explicit
     * for documentation and in case of later sdkconfig changes. */
    ret = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_ps(MIN_MODEM) failed: %d", ret);
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
        /* Bias the BT/WiFi coexistence arbiter toward BT so A2DP audio gets
         * priority over WiFi during radio contention. HTTP only needs 16 KB/s
         * (128kbps MP3); even a brief WiFi stall is absorbed by PSRAM jitter
         * buffers. BT underruns (heard as clicks/dropouts) cannot be buffered
         * away and must be prevented at the arbiter level. */
        ret = esp_coex_preference_set(ESP_COEX_PREFER_BT);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_coex_preference_set(BT) failed: %d (balance stays)", ret);
        }
        return ESP_OK;
    }
    ESP_LOGE(TAG, "WiFi connect timeout (%d ms)", WIFI_CONNECT_TIMEOUT_MS);
    return ESP_ERR_TIMEOUT;
}

bool wifi_manager_is_connected(void)
{
    return s_connected;
}
