#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config_manager.h"
#include "station_config.h"

static const char *TAG = "config_mgr";

#define NVS_NS "preset"

static char    s_ssid[64]  = WIFI_SSID;
static char    s_pass[64]  = WIFI_PASS;
static uint8_t s_bt_mac[6] = BT_PEER_ADDR;

static void parse_mac(const char *str, uint8_t out[6])
{
    unsigned v[6] = {0};
    if (sscanf(str, "%x:%x:%x:%x:%x:%x",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 6) {
        /* Input is MSB-first (AA:BB:CC:DD:EE:FF), but ESP32 needs LSB-first.
         * Reverse: out[0] = v[5] (LSB), out[5] = v[0] (MSB) */
        for (int i = 0; i < 6; i++) out[i] = (uint8_t)v[5 - i];
    }
}

esp_err_t config_manager_init(void)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace '" NVS_NS "' not found — using compiled-in defaults");
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open error %d — using defaults", ret);
        return ESP_OK;
    }

    size_t len;

    len = sizeof(s_ssid);
    if (nvs_get_str(h, "ssid", s_ssid, &len) == ESP_OK)
        ESP_LOGI(TAG, "NVS ssid: %s", s_ssid);

    len = sizeof(s_pass);
    nvs_get_str(h, "pass", s_pass, &len);

    char mac_str[18] = {0};
    len = sizeof(mac_str);
    if (nvs_get_str(h, "btmac", mac_str, &len) == ESP_OK) {
        parse_mac(mac_str, s_bt_mac);
        ESP_LOGI(TAG, "NVS btmac: %s", mac_str);
    }

    nvs_close(h);
    return ESP_OK;
}

const char *config_get_wifi_ssid(void) { return s_ssid; }
const char *config_get_wifi_pass(void) { return s_pass; }
const uint8_t *config_get_bt_mac(void) { return s_bt_mac; }
