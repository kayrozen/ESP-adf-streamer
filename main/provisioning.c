#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "config_manager.h"
#include "cJSON.h"

static const char *TAG = "provision";

#define UART_PORT_NUM      UART_NUM_0
#define UART_BUF_SIZE      (1024)
#define PROVISION_CMD_PREFIX "PROVISION:"
#define PROVISION_CMD_PREFIX_LEN (sizeof(PROVISION_CMD_PREFIX) - 1)

static TaskHandle_t s_provision_task = NULL;
static volatile bool s_provision_running = false;
static SemaphoreHandle_t s_provision_done = NULL;

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
    char *ssid = NULL;
    char *pass = NULL;
    char *btmac = NULL;

    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse JSON: %s", cJSON_GetErrorPtr());
        uart_write_bytes(UART_PORT_NUM, "ERROR:JSON parse failed\n", 24);
        goto cleanup;
    }

    cJSON *ssid_item = cJSON_GetObjectItemCaseSensitive(root, "ssid");
    if (ssid_item && cJSON_IsString(ssid_item) && ssid_item->valuestring) {
        ssid = strdup(ssid_item->valuestring);
    }

    cJSON *pass_item = cJSON_GetObjectItemCaseSensitive(root, "pass");
    if (pass_item && cJSON_IsString(pass_item) && pass_item->valuestring) {
        pass = strdup(pass_item->valuestring);
    }

    cJSON *btmac_item = cJSON_GetObjectItemCaseSensitive(root, "btmac");
    if (btmac_item && cJSON_IsString(btmac_item) && btmac_item->valuestring) {
        btmac = strdup(btmac_item->valuestring);
    }

    ESP_LOGI(TAG, "Parsed: ssid='%s' pass='%s' btmac='%s'",
             ssid ? ssid : "", pass ? "***" : "", btmac ? btmac : "");

    esp_err_t ret = provision_save_config(
        ssid ? ssid : NULL,
        pass ? pass : "",
        btmac ? btmac : NULL
    );

    if (ret == ESP_OK) {
        uart_write_bytes(UART_PORT_NUM, "OK\n", 3);
    } else {
        char err_msg[32];
        snprintf(err_msg, sizeof(err_msg), "ERROR:%d\n", ret);
        uart_write_bytes(UART_PORT_NUM, err_msg, strlen(err_msg));
    }

cleanup:
    if (ssid) free(ssid);
    if (pass) free(pass);
    if (btmac) free(btmac);
    cJSON_Delete(root);
}

static void provision_task(void *arg)
{
    uint8_t *data     = malloc(UART_BUF_SIZE);
    char    *line_buf = malloc(UART_BUF_SIZE);
    if (!data || !line_buf) {
        ESP_LOGE(TAG, "Failed to allocate UART buffers");
        free(data);
        free(line_buf);
        s_provision_running = false;
        xSemaphoreGive(s_provision_done);
        vTaskDelete(NULL);
        return;
    }

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
            } else if (line_pos < UART_BUF_SIZE - 1) {
                line_buf[line_pos++] = c;
            }
        }
    }

    free(data);
    free(line_buf);
    ESP_LOGI(TAG, "Provisioning task ended");
    xSemaphoreGive(s_provision_done);
    vTaskDelete(NULL);
}

esp_err_t provisioning_start(void)
{
    if (s_provision_running) {
        ESP_LOGW(TAG, "Already running");
        return ESP_OK;
    }

    if (!s_provision_done) {
        s_provision_done = xSemaphoreCreateBinary();
        if (!s_provision_done) {
            ESP_LOGE(TAG, "Failed to create provisioning done semaphore");
            return ESP_ERR_NO_MEM;
        }
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

    esp_err_t ret = uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2,
                                        UART_BUF_SIZE * 2, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %d", ret);
        return ret;
    }
    ret = uart_param_config(UART_PORT_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %d", ret);
        uart_driver_delete(UART_PORT_NUM);
        return ret;
    }

    s_provision_running = true;
    BaseType_t task_ret = xTaskCreate(provision_task, "provision", 4096, NULL, 10, &s_provision_task);
    if (task_ret != pdPASS) {
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
    if (!s_provision_running && !s_provision_task) {
        return;
    }
    s_provision_running = false;
    if (s_provision_task) {
        /* Wait for task to clean up and signal done (with timeout) */
        if (s_provision_done) {
            xSemaphoreTake(s_provision_done, portMAX_DELAY);
        }
        s_provision_task = NULL;
    }
    uart_driver_delete(UART_PORT_NUM);
}