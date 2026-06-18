#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "config_manager.h"

static const char *TAG = "provision";

#define UART_PORT_NUM      UART_NUM_0
#define UART_BUF_SIZE      (1024)
#define PROVISION_CMD_PREFIX "PROVISION:"
#define PROVISION_CMD_PREFIX_LEN (sizeof(PROVISION_CMD_PREFIX) - 1)

static TaskHandle_t s_provision_task = NULL;
static volatile bool s_provision_running = false;

static esp_err_t provision_save_config(const char *ssid, const char *pass, const char *btmac)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open("preset", NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %d", ret);
        return ret;
    }

    if (ssid && strlen(ssid) > 0) {
        ret = nvs_set_str(h, "ssid", ssid);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set ssid: %d", ret);
            nvs_close(h);
            return ret;
        }
    }

    if (pass) {
        ret = nvs_set_str(h, "pass", pass);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set pass: %d", ret);
            nvs_close(h);
            return ret;
        }
    }

    if (btmac && strlen(btmac) > 0) {
        ret = nvs_set_str(h, "btmac", btmac);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set btmac: %d", ret);
            nvs_close(h);
            return ret;
        }
    }

    ret = nvs_commit(h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS: %d", ret);
    } else {
        ESP_LOGI(TAG, "Configuration saved to NVS");
    }

    nvs_close(h);
    return ret;
}

static void provision_parse_and_save(const char *json_str)
{
    /* Simple JSON parser for {"ssid":"...","pass":"...","btmac":"..."} */
    char ssid[64] = {0};
    char pass[64] = {0};
    char btmac[18] = {0};

    const char *p = json_str;

    /* Find ssid */
    p = strstr(p, "\"ssid\"");
    if (p) {
        p = strchr(p, ':');
        if (p) {
            p = strchr(p + 1, '"');
            if (p) {
                const char *end = strchr(p + 1, '"');
                if (end && (end - p - 1) < sizeof(ssid)) {
                    strncpy(ssid, p + 1, end - p - 1);
                }
            }
        }
    }

    /* Find pass */
    p = strstr(json_str, "\"pass\"");
    if (p) {
        p = strchr(p, ':');
        if (p) {
            p = strchr(p + 1, '"');
            if (p) {
                const char *end = strchr(p + 1, '"');
                if (end && (end - p - 1) < sizeof(pass)) {
                    strncpy(pass, p + 1, end - p - 1);
                }
            }
        }
    }

    /* Find btmac */
    p = strstr(json_str, "\"btmac\"");
    if (p) {
        p = strchr(p, ':');
        if (p) {
            p = strchr(p + 1, '"');
            if (p) {
                const char *end = strchr(p + 1, '"');
                if (end && (end - p - 1) < sizeof(btmac)) {
                    strncpy(btmac, p + 1, end - p - 1);
                }
            }
        }
    }

    ESP_LOGI(TAG, "Parsed: ssid='%s' pass='%s' btmac='%s'", ssid, pass[0] ? "***" : "", btmac);

    esp_err_t ret = provision_save_config(
        ssid[0] ? ssid : NULL,
        pass[0] ? pass : "",
        btmac[0] ? btmac : NULL
    );

    if (ret == ESP_OK) {
        uart_write_bytes(UART_PORT_NUM, "OK\n", 3);
    } else {
        char err_msg[32];
        snprintf(err_msg, sizeof(err_msg), "ERROR:%d\n", ret);
        uart_write_bytes(UART_PORT_NUM, err_msg, strlen(err_msg));
    }
}

static void provision_task(void *arg)
{
    uint8_t *data = malloc(UART_BUF_SIZE);
    if (!data) {
        ESP_LOGE(TAG, "Failed to allocate UART read buffer");
        s_provision_running = false;
        vTaskDelete(NULL);
        return;
    }

    char line_buf[256];
    size_t line_pos = 0;

    ESP_LOGI(TAG, "Provisioning task started, listening on UART%d", UART_PORT_NUM);

    while (s_provision_running) {
        /* Read directly from UART driver */
        int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }
        data[len] = '\0';

        /* Process each byte */
        for (int i = 0; i < len; i++) {
            char c = data[i];
            if (c == '\n' || c == '\r') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';
                    ESP_LOGI(TAG, "Received: %s", line_buf);

                    if (strncmp(line_buf, PROVISION_CMD_PREFIX, PROVISION_CMD_PREFIX_LEN) == 0) {
                        provision_parse_and_save(line_buf + PROVISION_CMD_PREFIX_LEN);
                        /* Reboot to apply new config */
                        ESP_LOGI(TAG, "Provisioning complete, rebooting...");
                        vTaskDelay(pdMS_TO_TICKS(500));
                        esp_restart();
                    }
                    line_pos = 0;
                }
            } else if (line_pos < sizeof(line_buf) - 1) {
                line_buf[line_pos++] = c;
            }
        }
    }

    free(data);
    ESP_LOGI(TAG, "Provisioning task ended");
    vTaskDelete(NULL);
}

esp_err_t provisioning_start(void)
{
    if (s_provision_running) {
        ESP_LOGW(TAG, "Already running");
        return ESP_OK;
    }

    /* Install UART driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2,
                                        UART_BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    s_provision_running = true;
    BaseType_t ret = xTaskCreate(provision_task, "provision", 4096, NULL, 10, &s_provision_task);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create provisioning task");
        s_provision_running = false;
        uart_driver_delete(UART_PORT_NUM);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Provisioning started (UART%d @ 115200)", UART_PORT_NUM);
    return ESP_OK;
}

void provisioning_stop(void)
{
    s_provision_running = false;
    if (s_provision_task) {
        vTaskDelete(s_provision_task);
        s_provision_task = NULL;
    }
    uart_driver_delete(UART_PORT_NUM);
}