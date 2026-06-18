#pragma once
#include "audio_element.h"

/**
 * passthrough_el — Phase D custom AEL element.
 *
 * A zero-copy passthrough that counts bytes flowing through the pipeline.
 * Demonstrates that custom elements can be inserted into ESP-ADF pipelines.
 * Simulates a future telemetry or hashing interception point.
 */

typedef struct {
    uint64_t bytes_passed;   /* total bytes seen */
    uint32_t frames_passed;  /* number of process() calls */
} passthrough_stats_t;

/**
 * Create the passthrough element.
 * Insert between decoder and a2dp_stream in the pipeline.
 */
audio_element_handle_t passthrough_el_init(void);

/**
 * Copy current stats snapshot (thread-safe read).
 */
void passthrough_el_get_stats(audio_element_handle_t el,
                               passthrough_stats_t *out);
