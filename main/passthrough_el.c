#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "audio_element.h"
#include "passthrough_el.h"

static const char *TAG = "passthrough";

typedef struct {
    passthrough_stats_t stats;
    SemaphoreHandle_t   mutex;
} passthrough_ctx_t;

static esp_err_t passthrough_open(audio_element_handle_t el)
{
    if (!el) {
        return ESP_ERR_INVALID_ARG;
    }
    passthrough_ctx_t *ctx = audio_element_getdata(el);
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(&ctx->stats, 0, sizeof(ctx->stats));
    ESP_LOGI(TAG, "opened");
    return ESP_OK;
}

static audio_element_err_t passthrough_process(audio_element_handle_t el,
                                                char *in_buf, int in_len)
{
    if (!el || !in_buf) {
        return AEL_IO_ABORT;
    }
    passthrough_ctx_t *ctx = audio_element_getdata(el);
    if (!ctx) {
        return AEL_IO_ABORT;
    }

    int bytes_read = audio_element_input(el, in_buf, in_len);
    if (bytes_read <= 0) {
        return bytes_read;
    }

    int bytes_written = audio_element_output(el, in_buf, bytes_read);

    /* Count only bytes that were successfully written downstream. */
    if (bytes_written > 0) {
        xSemaphoreTake(ctx->mutex, portMAX_DELAY);
        ctx->stats.bytes_passed  += bytes_written;
        ctx->stats.frames_passed += 1;
        xSemaphoreGive(ctx->mutex);
    }

    return bytes_written;
}

static esp_err_t passthrough_close(audio_element_handle_t el)
{
    if (!el) {
        return ESP_ERR_INVALID_ARG;
    }
    passthrough_ctx_t *ctx = audio_element_getdata(el);
    if (!ctx) {
        return ESP_OK;
    }
    ESP_LOGI(TAG, "closed — total bytes: %"PRIu64", frames: %"PRIu32,
             ctx->stats.bytes_passed, ctx->stats.frames_passed);
    return ESP_OK;
}

static esp_err_t passthrough_destroy(audio_element_handle_t el)
{
    if (!el) {
        return ESP_ERR_INVALID_ARG;
    }
    passthrough_ctx_t *ctx = audio_element_getdata(el);
    if (!ctx) {
        return ESP_OK;
    }
    vSemaphoreDelete(ctx->mutex);
    free(ctx);
    return ESP_OK;
}

audio_element_handle_t passthrough_el_init(void)
{
    passthrough_ctx_t *ctx = calloc(1, sizeof(passthrough_ctx_t));
    if (!ctx) {
        ESP_LOGE(TAG, "calloc failed");
        return NULL;
    }
    ctx->mutex = xSemaphoreCreateMutex();
    if (!ctx->mutex) {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed");
        free(ctx);
        return NULL;
    }

    audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
    cfg.open    = passthrough_open;
    cfg.process = passthrough_process;
    cfg.close   = passthrough_close;
    cfg.destroy = passthrough_destroy;
    cfg.tag     = "passthrough";
    cfg.task_stack = 4 * 1024;
    cfg.out_rb_size = 8 * 1024;   /* ~46 ms PCM at 44.1kHz stereo — directly before A2DP, absorbs BT jitter */
    cfg.buffer_len = 2048;

    audio_element_handle_t el = audio_element_init(&cfg);
    if (!el) {
        ESP_LOGE(TAG, "audio_element_init failed");
        vSemaphoreDelete(ctx->mutex);
        free(ctx);
        return NULL;
    }
    audio_element_setdata(el, ctx);
    return el;
}

void passthrough_el_get_stats(audio_element_handle_t el,
                               passthrough_stats_t *out)
{
    if (!el || !out) return;
    passthrough_ctx_t *ctx = audio_element_getdata(el);
    if (!ctx || !ctx->mutex) return;
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    *out = ctx->stats;
    xSemaphoreGive(ctx->mutex);
}
