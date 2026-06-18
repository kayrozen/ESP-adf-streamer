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
    passthrough_ctx_t *ctx = audio_element_getdata(el);
    memset(&ctx->stats, 0, sizeof(ctx->stats));
    ESP_LOGI(TAG, "opened");
    return ESP_OK;
}

static audio_element_err_t passthrough_process(audio_element_handle_t el,
                                                char *in_buf, int in_len)
{
    passthrough_ctx_t *ctx = audio_element_getdata(el);

    int bytes_read = audio_element_input(el, in_buf, in_len);
    if (bytes_read <= 0) {
        return bytes_read;
    }

    int bytes_written = audio_element_output(el, in_buf, bytes_read);

    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    ctx->stats.bytes_passed  += (bytes_read > 0 ? bytes_read : 0);
    ctx->stats.frames_passed += 1;
    xSemaphoreGive(ctx->mutex);

    return bytes_written;
}

static esp_err_t passthrough_close(audio_element_handle_t el)
{
    passthrough_ctx_t *ctx = audio_element_getdata(el);
    ESP_LOGI(TAG, "closed — total bytes: %"PRIu64", frames: %"PRIu32,
             ctx->stats.bytes_passed, ctx->stats.frames_passed);
    return ESP_OK;
}

static esp_err_t passthrough_destroy(audio_element_handle_t el)
{
    passthrough_ctx_t *ctx = audio_element_getdata(el);
    vSemaphoreDelete(ctx->mutex);
    free(ctx);
    return ESP_OK;
}

audio_element_handle_t passthrough_el_init(void)
{
    passthrough_ctx_t *ctx = calloc(1, sizeof(passthrough_ctx_t));
    ctx->mutex = xSemaphoreCreateMutex();

    audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
    cfg.open    = passthrough_open;
    cfg.process = passthrough_process;
    cfg.close   = passthrough_close;
    cfg.destroy = passthrough_destroy;
    cfg.tag     = "passthrough";
    cfg.task_stack = 2 * 1024;
    cfg.buffer_len = 2048;

    audio_element_handle_t el = audio_element_init(&cfg);
    audio_element_setdata(el, ctx);
    return el;
}

void passthrough_el_get_stats(audio_element_handle_t el,
                               passthrough_stats_t *out)
{
    passthrough_ctx_t *ctx = audio_element_getdata(el);
    xSemaphoreTake(ctx->mutex, portMAX_DELAY);
    *out = ctx->stats;
    xSemaphoreGive(ctx->mutex);
}
