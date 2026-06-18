#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "monitor.h"

static const char *TAG = "monitor";

/* Buffer for vTaskGetRunTimeStats — allocate in PSRAM to avoid internal heap pressure */
#define STATS_BUF_SIZE (4 * 1024)

static TaskHandle_t s_monitor_task = NULL;
static volatile bool s_running     = false;
static SemaphoreHandle_t s_monitor_mutex = NULL;

static void monitor_task(void *arg)
{
    char *stats_buf = heap_caps_malloc(STATS_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!stats_buf) {
        stats_buf = malloc(STATS_BUF_SIZE);
    }

    while (true) {
        /* Sleep for the interval, but wake immediately if notified by monitor_stop().
         * ulTaskNotifyTake returns > 0 when woken by a notification (stop signal). */
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MONITOR_INTERVAL_S * 1000)) > 0) {
            break;
        }
        if (!s_running) break;

        size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        size_t free_spiram   = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        size_t min_internal  = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
        size_t min_spiram    = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);

        ESP_LOGI(TAG,
                 "HEAP  internal: %5u KB free  (min ever: %5u KB) | "
                 "PSRAM: %5u KB free  (min ever: %5u KB)",
                 (unsigned)(free_internal / 1024), (unsigned)(min_internal / 1024),
                 (unsigned)(free_spiram   / 1024), (unsigned)(min_spiram   / 1024));

        if (stats_buf) {
            vTaskGetRunTimeStats(stats_buf);
            ESP_LOGI(TAG, "CPU runtime stats:\n%s", stats_buf);
        }
    }

    if (stats_buf) free(stats_buf);
    /* Clear handle inside the task so monitor_start() can't race with a
     * still-running task after monitor_stop() returns. */
    xSemaphoreTake(s_monitor_mutex, portMAX_DELAY);
    s_monitor_task = NULL;
    xSemaphoreGive(s_monitor_mutex);
    vTaskDelete(NULL);
}

void monitor_start(void)
{
    if (!s_monitor_mutex) { ESP_LOGE(TAG, "monitor_init() not called"); return; }
    xSemaphoreTake(s_monitor_mutex, portMAX_DELAY);
    if (s_monitor_task) {
        ESP_LOGW(TAG, "Monitor already running");
        xSemaphoreGive(s_monitor_mutex);
        return;
    }
    s_running = true;
    BaseType_t ret = xTaskCreatePinnedToCore(monitor_task, "monitor", 4 * 1024,
                            NULL, 2, &s_monitor_task, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        s_monitor_task = NULL;
        s_running = false;
    } else {
        ESP_LOGI(TAG, "Monitoring started (interval %ds)", MONITOR_INTERVAL_S);
    }
    xSemaphoreGive(s_monitor_mutex);
}

void monitor_stop(void)
{
    if (!s_monitor_mutex) return;
    TaskHandle_t task = NULL;
    xSemaphoreTake(s_monitor_mutex, portMAX_DELAY);
    if (!s_monitor_task) {
        xSemaphoreGive(s_monitor_mutex);
        return;
    }
    task = s_monitor_task;
    s_running = false;
    xSemaphoreGive(s_monitor_mutex);

    if (task) {
        xTaskNotifyGive(task);
    }
    /* Task clears its own handle under mutex before deleting itself */
}

void monitor_init(void)
{
    s_monitor_mutex = xSemaphoreCreateMutex();
}
