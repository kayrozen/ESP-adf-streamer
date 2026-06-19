#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "monitor.h"

static const char *TAG = "monitor";

/* Max tasks to enumerate for the per-task CPU table. */
#define STATS_MAX_TASKS 32

static TaskHandle_t s_monitor_task = NULL;
static volatile bool s_running     = false;
static SemaphoreHandle_t s_monitor_mutex = NULL;

/* Print a per-task CPU utilisation table.
 *
 * Why not vTaskGetRunTimeStats(): on ESP-IDF v5.3's SMP FreeRTOS it renders an
 * empty string (logs 26/29/30 all show a blank "CPU runtime stats:" line), so
 * we lost all CPU visibility exactly when the workload became CPU-bound.
 * uxTaskGetSystemState() is reliable. The IDLE0 / IDLE1 rows give each core's
 * idle headroom directly — that is what confirms whether Core 0 (WiFi + BT
 * controller + Bluedroid SBC encode + coex) is the saturated resource that
 * caps the decode+encode chain below real-time.
 *
 * 'tasks' is a caller-owned PSRAM buffer of STATS_MAX_TASKS entries. */
static void log_cpu_stats(TaskStatus_t *tasks)
{
    uint32_t total_runtime = 0;
    UBaseType_t n = uxTaskGetSystemState(tasks, STATS_MAX_TASKS, &total_runtime);
    if (n == 0 || total_runtime == 0) {
        ESP_LOGW(TAG, "CPU stats unavailable (n=%u, total=%u)",
                 (unsigned)n, (unsigned)total_runtime);
        return;
    }
    /* Divide by 100 up front so counter/(total/100) yields percent. */
    uint32_t total_pct = total_runtime / 100;
    if (total_pct == 0) total_pct = 1;

    ESP_LOGI(TAG, "CPU per task (IDLE0/IDLE1 = per-core idle headroom):");
    for (UBaseType_t i = 0; i < n; i++) {
        uint32_t pct = tasks[i].ulRunTimeCounter / total_pct;
        ESP_LOGI(TAG, "  %-16s %2u%%  (prio %2u, stk_hwm %5u)",
                 tasks[i].pcTaskName, (unsigned)pct,
                 (unsigned)tasks[i].uxCurrentPriority,
                 (unsigned)tasks[i].usStackHighWaterMark);
    }
}

static void monitor_task(void *arg)
{
    TaskStatus_t *tasks = heap_caps_malloc(STATS_MAX_TASKS * sizeof(TaskStatus_t),
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tasks) {
        tasks = malloc(STATS_MAX_TASKS * sizeof(TaskStatus_t));
    }

    /* Emit one snapshot immediately. The OOM crashes in logs 16/18 fire within
     * the first ~10s — before the first MONITOR_INTERVAL_S tick — so without an
     * up-front print the streaming-phase DRAM floor is never captured. */
    ESP_LOGI(TAG,
             "HEAP@start  internal: %5u KB free  (min ever: %5u KB) | "
             "PSRAM: %5u KB free  (min ever: %5u KB)",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024),
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
             (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM) / 1024));

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

        if (tasks) {
            log_cpu_stats(tasks);
        }
    }

    if (tasks) free(tasks);
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
