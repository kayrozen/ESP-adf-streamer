#pragma once
#include <stdint.h>

/**
 * monitor.h — Phase C resource monitoring
 *
 * Starts a background FreeRTOS task that logs every MONITOR_INTERVAL_S:
 *   - Free internal heap (MALLOC_CAP_INTERNAL)
 *   - Free PSRAM (MALLOC_CAP_SPIRAM)
 *   - Per-task CPU runtime stats (vTaskGetRunTimeStats)
 *
 * Heap values are logged as KB. A sustained downward trend indicates a leak.
 * CPU stats show % time each task consumed since last interval.
 */

#define MONITOR_INTERVAL_S  30

void monitor_start(void);
void monitor_stop(void);
