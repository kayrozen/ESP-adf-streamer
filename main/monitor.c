#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "monitor.h"

static const char *TAG = "monitor";

static TaskHandle_t s_monitor_task = NULL;
static volatile bool s_running     = false;
static SemaphoreHandle_t s_monitor_mutex = NULL;

/* Print a per-task CPU utilisation table.
 *
 * Why not vTaskGetRunTimeStats(): on ESP-IDF v5.3's SMP FreeRTOS it renders an
 * empty string (logs 26/29/30 all show a blank "CPU runtime stats:" line), so
 * we lost all CPU visibility exactly when we needed to diagnose the load.
 * uxTaskGetSystemState() is reliable. The IDLE0 / IDLE1 rows give each core's
 * idle headroom directly — this is how we tracked Core 0 (WiFi + BT controller
 * + Bluedroid SBC encode + coex) load across the tuning work.
 *
 * Historical note: early logs (e.g. log 32) showed Core 0 at ~92% busy (IDLE0
 * 8%) and it was treated as the saturated resource.  After the PSRAM memory
 * moves (BT host + WiFi/LWIP to PSRAM) log 64 shows IDLE0 back at 15% with the
 * BT stack lighter — Core 0 is no longer the binding constraint.  This table is
 * still the primary signal for spotting any regression back toward saturation.
 *
 * The buffer is sized from the live task count (+headroom) and allocated in
 * PSRAM per call: a fixed cap risks uxTaskGetSystemState() returning 0 when
 * WiFi+Bluedroid+ADF push the task count past it, which would silently blank
 * the table — the exact failure this function exists to avoid. */
static void log_cpu_stats(void)
{
    UBaseType_t array_size = uxTaskGetNumberOfTasks() + 8;  /* headroom for races */
    TaskStatus_t *tasks = heap_caps_malloc(array_size * sizeof(TaskStatus_t),
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tasks) {
        tasks = malloc(array_size * sizeof(TaskStatus_t));
    }
    if (!tasks) {
        ESP_LOGW(TAG, "CPU stats unavailable (alloc failed)");
        return;
    }

    uint32_t total_runtime = 0;
    UBaseType_t n = uxTaskGetSystemState(tasks, array_size, &total_runtime);
    if (n == 0 || total_runtime == 0) {
        ESP_LOGW(TAG, "CPU stats unavailable (n=%u, total=%u)",
                 (unsigned)n, (unsigned)total_runtime);
        free(tasks);
        return;
    }

    ESP_LOGI(TAG, "CPU per task (IDLE0/IDLE1 = per-core idle headroom):");
    for (UBaseType_t i = 0; i < n; i++) {
        /* 64-bit multiply-first keeps full precision and cannot overflow:
         * ulRunTimeCounter is cumulative-since-boot, so ×100 exceeds 32 bits
         * within ~43s on a busy core. */
        uint32_t pct = (uint32_t)(((uint64_t)tasks[i].ulRunTimeCounter * 100) / total_runtime);
        ESP_LOGI(TAG, "  %-16s %2u%%  (prio %2u, stk_hwm %5u)",
                 tasks[i].pcTaskName, (unsigned)pct,
                 (unsigned)tasks[i].uxCurrentPriority,
                 (unsigned)tasks[i].usStackHighWaterMark);
    }
    free(tasks);
}

static void monitor_task(void *arg)
{
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

        log_cpu_stats();
    }

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
