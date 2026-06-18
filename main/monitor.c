#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "monitor.h"

static const char *TAG = "monitor";

/* Buffer for vTaskGetRunTimeStats — allocate in PSRAM to avoid internal heap pressure */
#define STATS_BUF_SIZE (4 * 1024)

static TaskHandle_t s_monitor_task = NULL;
static volatile bool s_running     = false;

static void monitor_task(void *arg)
{
    char *stats_buf = heap_caps_malloc(STATS_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!stats_buf) {
        /* Fallback to internal heap */
        stats_buf = malloc(STATS_BUF_SIZE);
    }

    TickType_t last_wake = xTaskGetTickCount();

    while (s_running) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MONITOR_INTERVAL_S * 1000));

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
    vTaskDelete(NULL);
}

void monitor_start(void)
{
    if (s_monitor_task) return;
    s_running = true;
    xTaskCreatePinnedToCore(monitor_task, "monitor", 4 * 1024,
                            NULL, 2, &s_monitor_task, 1);
    ESP_LOGI(TAG, "Monitoring started (interval %ds)", MONITOR_INTERVAL_S);
}

void monitor_stop(void)
{
    s_running = false;
    s_monitor_task = NULL;
}
