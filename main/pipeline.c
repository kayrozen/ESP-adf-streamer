#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "audio_pipeline.h"
#include "audio_element.h"
#include "audio_event_iface.h"
#include "http_stream.h"
#include "mp3_decoder.h"
#include "aac_decoder.h"
#include "filter_resample.h"
#include "esp_peripherals.h"  /* must precede a2dp_stream.h (defines esp_periph_handle_t) */
#include "a2dp_stream.h"
#include "bt_manager.h"
#include "pipeline.h"

static const char *TAG = "pipeline";

/* PCM-side jitter buffer — sits on rsp_filter.out_rb (44100 Hz stereo 16-bit,
 * 176.4 KB/s) feeding a2dp_stream.  THE buffer that rides out HTTPS delivery
 * stalls: when the TLS socket goes quiet the decoder stalls, the rsp_filter
 * stalls, and a2dp_stream drains from this reserve.
 *
 *   512KB ≈ 2.9 s @ 44100 Hz/stereo/16-bit (176.4 KB/s) — absorbs the 2.08 s
 *   worst-case TLS delivery gap observed in log 47 with margin.
 *
 * Lives in PSRAM (>16 KB SPIRAM_MALLOC_ALWAYSINTERNAL threshold). */
#define PCM_JITTER_RB_SIZE  (512 * 1024)

/* Intermediate ring buffer between decoder and rsp_filter (PCM at source rate,
 * either 44100 or 48000 Hz).  Sized for ~0.33 s — enough for the rsp_filter
 * to drain decoder bursts; the large jitter buffer is downstream of the
 * resampler on rsp_filter.out_rb, so only a small hop is needed here. */
#define DECODER_TO_RSP_RB_SIZE  (64 * 1024)

/* Pipeline handles */
static audio_pipeline_handle_t   s_pipeline     = NULL;
static audio_element_handle_t    s_http_el      = NULL;
static audio_element_handle_t    s_decoder_el   = NULL;
static audio_element_handle_t    s_resample_el  = NULL;
static audio_element_handle_t    s_a2dp_el      = NULL;
static audio_event_iface_handle_t s_evt         = NULL;

/* Peer BDA cache */
static uint8_t s_peer_bda[6] = {0};

/* Decoder type tracking for dynamic selection */
static pipeline_codec_t s_current_codec = PIPELINE_CODEC_AUTO;

/* Mutex for protecting pipeline state during station changes */
static SemaphoreHandle_t s_pipeline_mutex = NULL;

/* Sample rate / channel count the rsp_filter element is currently BUILT for
 * (i.e. the src_rate/src_ch passed to create_resample_filter() for the element
 * that is registered right now).  The resampler is never reconfigured in place —
 * rsp_filter_set_src_info() destroys and rebuilds the internal SRC handle and was
 * the source of the deterministic vQueueDelete(NULL) crash (logs 54/55/56).
 * Instead the whole rsp_filter element is hot-swapped for a fresh one created at
 * the new rate via the proven init path (see swap_rsp_element_locked).
 *
 * Set in pipeline_init() to the boot codec's expected rate, then updated by
 * start_station_locked() once the true decoded rate is known (from the NVS cache
 * or a decode-only probe).  Used as the no-op guard in swap_rsp_element_locked():
 * a station whose rate already matches needs no rebuild. */
static int s_rsp_src_rate = 48000;
static int s_rsp_src_ch   = 2;

/* ---- Per-URL resample-rate cache (NVS) ----
 *
 * The decoder only learns a stream's true sample rate after it opens and reads
 * the first frame header.  Rather than guess the rate from the URL (the old race:
 * a wrong guess played at the wrong pitch until a watcher rebuilt the resampler
 * mid-stream), we run a decode-only probe on the first visit to each URL, learn
 * the real rate before any audio reaches the BT sink, and persist it here.  On
 * every later visit (including across reboots) the cache hit skips the probe and
 * the resampler is built correct the first time — zero wrong-rate audio. */
/* Namespace is versioned: a probe bug (pre-v2) latched the AAC decoder's
 * transient first-frame rate (44100 for a 48000 Hz stream), poisoning entries
 * with the wrong rate.  Bumping the namespace abandons every stale entry so all
 * URLs are re-probed with the fixed settle logic (see probe_rate_locked). */
#define RSP_CACHE_NS  "rspcache2"

/* Forward declarations for static functions used before their definition */
static esp_err_t pipeline_recreate_decoder(pipeline_codec_t new_codec);
static audio_element_handle_t create_resample_filter(int src_rate, int src_ch);
static pipeline_codec_t detect_codec_from_url(const char *url);
static esp_err_t start_station_locked(const char *url);

/* ---- helpers ---- */

static audio_element_handle_t create_http_stream(void)
{
    http_stream_cfg_t cfg = HTTP_STREAM_CFG_DEFAULT();
    cfg.type              = AUDIO_STREAM_READER;
    cfg.enable_playlist_parser = true;   /* HLS playlist support */
    /* Stack size is owned by CONFIG_HTTP_STREAM_TASK_STACK_SIZE (8192 in sdkconfig.defaults),
     * which HTTP_STREAM_CFG_DEFAULT() already applies to cfg.task_stack — no override here so
     * the two can't drift.  8 KB covers mbedTLS TLS 1.2 RSA call frames (ASN.1 parse + key
     * exchange); TLS I/O buffers live in PSRAM via MBEDTLS_EXTERNAL_MEM_ALLOC=y, so only call
     * frames land on this stack, and CANARY catches any genuine overflow. */
    /* Keep http stack in INTERNAL DRAM — but note this is NOT the cause of, nor the fix
     * for, the TLS-handshake crash.  Logs 62 (stack_in_ext=true) and 63 (=false) crashed
     * IDENTICALLY at the first https handshake, which proves the stack's location is
     * irrelevant to it.  The real cause is internal-DRAM exhaustion under WiFi+BT+TLS
     * coexistence ("wifi:m f null" flood -> NULL queue handle -> vQueueDelete/abort); the
     * fix is moving the Bluedroid host and WiFi/LWIP buffers to PSRAM
     * (BT_ALLOCATION_FROM_SPIRAM_FIRST + SPIRAM_TRY_ALLOCATE_WIFI_LWIP in
     * sdkconfig.defaults).  We leave this stack internal because the http task is small
     * and a PSRAM (40 MHz cache-backed) stack buys nothing once Bluedroid is the freed
     * memory; internal keeps it simple and avoids any PSRAM-stack edge cases. */
    cfg.stack_in_ext      = false;
    cfg.task_prio         = 23;
    /* HTTP_STREAM_TASK_CORE defaults to 0 via Kconfig — move to Core 1.
     *
     * Log 32 CPU table (pre-PSRAM): Core 0 ran at 92% busy (IDLE0 = 8%) —
     * BTC_TASK 20% + btController 13% + WiFi 13% + BTU_TASK 9% + hciT 3% = 58%
     * for BT/WiFi, plus http 18% competing on the same core.  Pinning http to
     * Core 1 moved that ~18% off Core 0, leaving more for the BT SBC-encode path
     * (the source of the L2CAP is_cong bursts).  http and dec share Core 1 in
     * natural turn: http fills the ring buffer, dec drains it, so they alternate
     * rather than compete.
     *
     * Post-PSRAM (log 64): Core 0 is no longer saturated (IDLE0 15%, BTC_TASK
     * down to 12%), so this pinning is no longer load-bearing — but it is still
     * correct (Core 1 has 42% idle, and keeping the I/O task off the radio core
     * costs nothing).  Kept as-is. */
    cfg.task_core         = 1;
    /* Compressed-side jitter buffer. 64KB ≈ 4s of 128kbps MP3. Lives in PSRAM
     * (>16KB SPIRAM_MALLOC_ALWAYSINTERNAL threshold), so it costs no internal
     * DRAM — and moving it out of internal actually frees the old 4KB. Lets the
     * decoder burst-refill from the socket backlog faster than real-time after
     * a BT/WiFi coexistence stall, instead of starving on a 4KB input. */
    cfg.out_rb_size       = 64 * 1024;
    return http_stream_init(&cfg);
}

static audio_element_handle_t create_mp3_decoder(void)
{
    mp3_decoder_cfg_t cfg = DEFAULT_MP3_DECODER_CONFIG();
    /* Pin the decoder to Core 1. Core 0 runs the WiFi driver (core=0 in the
     * logs), the BT controller, Bluedroid, and BT/WiFi coexistence — at 160 MHz
     * the MP3 decode competed with all of that and could only sustain ~108 KB/s
     * PCM (log 26), below the 192 KB/s real-time rate → choppy. Core 1 only runs
     * the app/monitor tasks, so the decoder gets a near-dedicated core; combined
     * with the 240 MHz bump this clears the real-time decode budget with margin.
     * Confirmed in log 64: decoder ~24% on Core 1 (IDLE1 42%) with steady-state
     * playback underflow-free, so the decode budget is met with headroom. */
    cfg.task_core         = 1;
    cfg.task_prio         = 23;
    cfg.out_rb_size       = DECODER_TO_RSP_RB_SIZE;  /* hop to rsp_filter; jitter buffer is on rsp_filter.out_rb */
    /* Keep this stack in INTERNAL DRAM even though SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y.
     * The decoder is compute-bound; a PSRAM stack (40 MHz cache-backed) would slow the
     * tight DSP loops and bring back the real-time underrun we fixed with 240 MHz. Only
     * the I/O-bound http_stream stack is allowed in PSRAM. */
    cfg.stack_in_ext      = false;
    return mp3_decoder_init(&cfg);
}

static audio_element_handle_t create_aac_decoder(void)
{
    aac_decoder_cfg_t cfg = DEFAULT_AAC_DECODER_CONFIG();
    /* Pin to Core 1 — same rationale as create_mp3_decoder(): keep CPU-heavy
     * decode off Core 0, which is saturated by WiFi + BT + coexistence. */
    cfg.task_core         = 1;
    cfg.task_prio         = 23;
    cfg.out_rb_size       = DECODER_TO_RSP_RB_SIZE;  /* hop to rsp_filter; jitter buffer is on rsp_filter.out_rb */
    /* Compute-bound — keep stack in internal DRAM (see create_mp3_decoder note). */
    cfg.stack_in_ext      = false;
    return aac_decoder_init(&cfg);
}

static audio_element_handle_t create_resample_filter(int src_rate, int src_ch)
{
    /* Normalise all decoded PCM to 44100 Hz stereo 16-bit before the SBC encoder.
     *
     * Root cause (log 53): the JBL speaker negotiated SBC at 44100 Hz, but the
     * AAC streams output 48000 Hz PCM.  The SBC encoder consumed only 44100*2*2 =
     * 176.4 KB/s while the decoder produced 48000*2*2 = 192.0 KB/s — an 8.8%
     * surplus that filled the 512 KB ring buffer in ~33 s and caused periodic
     * decoder stalls (perceived as 1-second drops).  L2CAP is_cong_cback_context
     * errors in logs 49-53 are the same mismatch on the BT side.
     *
     * rsp_filter does NOT learn the upstream rate on its own (ADF elements only
     * exchange raw PCM through ring buffers).  We cannot know an arbitrary stream's
     * rate until the decoder opens it, so the element is created at the rate learned
     * from the NVS cache or the decode-only probe (see start_station_locked), and is
     * hot-swapped for a fresh one if a later station needs a different rate —
     * see swap_rsp_element_locked().  We deliberately do NOT call
     * rsp_filter_set_src_info() at runtime: it tears down and rebuilds the internal
     * SRC handle and was the source of the deterministic vQueueDelete(NULL) crash
     * (logs 54/55/56).  Every working ADF resample example configures the rate at
     * init only; this keeps us on that proven path for any URL/codec/rate.
     *
     * PCM_JITTER_RB_SIZE (512 KB) is placed here, on the resampler's output ring
     * buffer (44100 Hz, 176.4 KB/s), giving 2.9 s of reserve for HTTPS delivery
     * stalls. */
    rsp_filter_cfg_t cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    cfg.src_rate    = src_rate;  /* real decoded rate; non-identity avoids NULL SRC handle */
    cfg.src_ch      = src_ch;
    cfg.dest_rate   = 44100;  /* fixed to match JBL SBC negotiation */
    cfg.dest_ch     = 2;
    cfg.task_core   = 1;
    cfg.task_prio   = 22;     /* just below decoder (23) */
    cfg.task_stack  = 4 * 1024;  /* resample task only calls esp_resample_process(); heap-allocated bufs; 4 KB ample */
    cfg.out_rb_size = PCM_JITTER_RB_SIZE;
    /* DSP task — keep stack in internal DRAM (see create_mp3_decoder note). */
    cfg.stack_in_ext = false;
    return rsp_filter_init(&cfg);
}

/* ---- NVS rate cache ----
 *
 * NVS keys are limited to 15 chars, so a URL cannot be a key directly.  Hash the
 * URL (FNV-1a 32-bit) and format it as "u" + 8 hex digits (9 chars).  The stored
 * value packs rate (lower 24 bits, max 48000) and channels (top 8 bits). */
static void rate_cache_key(const char *url, char out[12])
{
    uint32_t h = 2166136261u;
    for (const unsigned char *p = (const unsigned char *)url; *p; p++) {
        h ^= *p;
        h *= 16777619u;
    }
    snprintf(out, 12, "u%08lx", (unsigned long)h);
}

static esp_err_t rate_cache_get(const char *url, int *rate, int *ch)
{
    nvs_handle_t nvs;
    if (nvs_open(RSP_CACHE_NS, NVS_READONLY, &nvs) != ESP_OK) {
        return ESP_FAIL;  /* namespace absent until first put — treat as miss */
    }
    char key[12];
    rate_cache_key(url, key);
    uint32_t v = 0;
    esp_err_t ret = nvs_get_u32(nvs, key, &v);
    nvs_close(nvs);
    if (ret != ESP_OK) return ret;
    *rate = (int)(v & 0x00FFFFFF);
    *ch   = (int)((v >> 24) & 0xFF);
    if (*rate <= 0 || *ch <= 0) return ESP_FAIL;
    return ESP_OK;
}

static void rate_cache_put(const char *url, int rate, int ch)
{
    nvs_handle_t nvs;
    if (nvs_open(RSP_CACHE_NS, NVS_READWRITE, &nvs) != ESP_OK) return;
    char key[12];
    rate_cache_key(url, key);
    uint32_t v = ((uint32_t)rate & 0x00FFFFFF) | (((uint32_t)ch & 0xFF) << 24);
    if (nvs_set_u32(nvs, key, v) == ESP_OK) {
        nvs_commit(nvs);
    }
    nvs_close(nvs);
}

/* Stop the pipeline only if it is actually running.
 *
 * audio_pipeline_wait_for_stop() blocks listening for stop reports from running
 * element tasks; on a freshly-linked pipeline that has never been run (e.g. the
 * boot path, before the very first audio_pipeline_run) no tasks exist to report,
 * so an unconditional stop+wait could hang.  http is element 0 in every linked
 * config and always runs, so its state is a reliable "is the pipeline active?"
 * proxy.  Only stop when it is RUNNING/PAUSED. */
static void pipeline_stop_if_running(void)
{
    audio_element_state_t st = audio_element_get_state(s_http_el);
    if (st == AEL_STATE_RUNNING || st == AEL_STATE_PAUSED) {
        audio_pipeline_stop(s_pipeline);
        audio_pipeline_wait_for_stop(s_pipeline);
    }
}

/* Hot-swap the rsp_filter element for a fresh one built at (rate, ch).
 *
 * Precondition: the pipeline is STOPPED and UNLINKED (the caller does the stop/
 * unlink as part of the full restart sequence).  This only re-registers the rsp
 * element; it does NOT link or run — start_station_locked() does that for all
 * four elements afterward.  No-op (and keeps the existing element) when the rate
 * already matches.  On failure the old element is restored and left registered so
 * the caller's link()/run() still produces playback (at the wrong rate, logged).
 * The caller MUST hold s_pipeline_mutex. */
static esp_err_t swap_rsp_element_locked(int rate, int ch)
{
    if (rate == s_rsp_src_rate && ch == s_rsp_src_ch) {
        return ESP_OK;
    }

    audio_element_handle_t new_rsp = create_resample_filter(rate, ch);
    if (!new_rsp) {
        ESP_LOGE(TAG, "Failed to create resample filter for %d Hz / %d ch", rate, ch);
        return ESP_FAIL;
    }

    audio_element_handle_t old_rsp = s_resample_el;
    audio_pipeline_unregister(s_pipeline, old_rsp);
    s_resample_el = new_rsp;
    esp_err_t ret = audio_pipeline_register(s_pipeline, s_resample_el, "rsp");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register new resampler: %d — keeping old element", ret);
        s_resample_el = old_rsp;
        audio_pipeline_register(s_pipeline, s_resample_el, "rsp");
        audio_element_deinit(new_rsp);
        return ret;
    }

    audio_element_deinit(old_rsp);
    s_rsp_src_rate = rate;
    s_rsp_src_ch   = ch;
    ESP_LOGI(TAG, "Resampler built for %d Hz / %d ch -> 44100 Hz / 2 ch", rate, ch);
    return ESP_OK;
}

/* Decode-only probe: run just http -> dec to learn the stream's true sample rate
 * before any audio reaches the BT sink.  Used only for MP3 (and future codecs
 * that reliably report their rate via audio_element_getinfo early in playback).
 *
 * WHY NOT AAC: the AAC decoder in this ADF build initializes its info struct with
 * a 44100 Hz default in aac_open() before any ADTS frame is decoded.  The getinfo
 * value stays at 44100 for the entire probe window (even with 600 ms stability
 * requirement and 8 s timeout), never updating to the real bitstream rate (48000 Hz
 * for Radio France streams).  Confirmed across logs 69 and 70: rspcache2 consistently
 * holds 44100 for AAC URLs after a probe, resampler runs 44100→44100 (pass-through)
 * while the decoder produces 192 KB/s (48000 Hz), L2CAP gets 192 KB/s worth of SBC
 * frames while the radio slot timing absorbs only 176.4 KB/s → sustained is_cong
 * bursts → choppy audio throughout.  Additionally, AEL_MSG_CMD_REPORT_MUSIC_INFO
 * never arrives in the event loop for AAC, so event-based probing would also fail.
 *
 * AAC rate is handled in start_station_locked() with a hardcoded 48000 Hz default
 * (matching all tested Radio France AAC streams) cached to NVS on first visit.
 *
 * MP3: the MP3 decoder reports 44100 stably within ~100 ms; probe is reliable.
 *
 * Leaves the pipeline STOPPED and UNLINKED (http + dec tasks terminated) so the
 * caller can swap the resampler and relink all four elements. */
#define PROBE_TIMEOUT_US   (8LL * 1000 * 1000)
#define PROBE_POLL_MS      50
#define PROBE_STABLE_MS    300   /* two stable reads is enough for MP3 */

static esp_err_t probe_rate_locked(const char *url, int *out_rate, int *out_ch)
{
    pipeline_stop_if_running();
    audio_pipeline_unlink(s_pipeline);

    audio_element_set_uri(s_http_el, url);

    const char *probe_tag[] = {"http", "dec"};
    esp_err_t ret = audio_pipeline_link(s_pipeline, probe_tag, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Probe link failed: %d", ret);
        return ret;
    }
    audio_pipeline_reset_ringbuffer(s_pipeline);
    audio_pipeline_reset_elements(s_pipeline);
    audio_pipeline_change_state(s_pipeline, AEL_STATE_INIT);
    ret = audio_pipeline_run(s_pipeline);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Probe run failed: %d", ret);
        audio_pipeline_unlink(s_pipeline);
        return ret;
    }

    int last_rate = -1, last_ch = -1, stable_ms = 0;
    esp_err_t result = ESP_ERR_TIMEOUT;
    int64_t t0 = esp_timer_get_time();
    while ((esp_timer_get_time() - t0) < PROBE_TIMEOUT_US) {
        /* Bail early if http hit a hard error (dead URL / 4xx) — the decoder will
         * never produce a rate, so don't burn the full timeout. */
        if (audio_element_get_state(s_http_el) == AEL_STATE_ERROR) {
            ESP_LOGW(TAG, "Probe: http element entered error state");
            result = ESP_FAIL;
            break;
        }
        audio_element_info_t ai = {0};
        audio_element_getinfo(s_decoder_el, &ai);
        /* Log every nonzero reading so we can trace decoder init behaviour. */
        if (ai.sample_rates > 0 && ai.channels > 0) {
            ESP_LOGD(TAG, "Probe poll: %d Hz / %d ch  (stable_ms=%d)",
                     ai.sample_rates, ai.channels, stable_ms);
            if (ai.sample_rates == last_rate && ai.channels == last_ch) {
                stable_ms += PROBE_POLL_MS;
                if (stable_ms >= PROBE_STABLE_MS) {
                    *out_rate = ai.sample_rates;
                    *out_ch   = ai.channels;
                    result = ESP_OK;
                    break;
                }
            } else {
                last_rate = ai.sample_rates;
                last_ch   = ai.channels;
                stable_ms = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(PROBE_POLL_MS));
    }

    pipeline_stop_if_running();
    /* Terminate probe tasks so the real run creates fresh ones.  Without
     * terminate, http and dec remain in STOPPED state.  audio_pipeline_run()
     * on stopped tasks sends RESUME instead of creating new tasks — but the
     * RESUME handler calls open(), which on HLS must complete a multi-step
     * TLS + playlist fetch sequence within ~4000ms while BT is now competing
     * with WiFi.  That budget is regularly exceeded → dec RESUME timeout.
     * Terminate destroys the tasks; the subsequent audio_pipeline_run()
     * creates fresh ones whose open() is not on any timeout clock. */
    audio_element_terminate(s_http_el);
    audio_element_terminate(s_decoder_el);
    audio_pipeline_unlink(s_pipeline);
    return result;
}

/* Bring the pipeline up on `url` with the resampler built for the stream's true
 * decoded rate.  Single entry point for both the boot start and every station
 * change.  The caller MUST hold s_pipeline_mutex.
 *
 * Sequence:
 *   1. Recreate the decoder if the URL's codec differs from the current one.
 *   2. Determine the source rate: NVS cache hit; for AAC use 48000 Hz default;
 *      for non-AAC (MP3) run the decode-only probe.
 *   3. Stop + unlink, swap the resampler to the known rate, link all four, run.
 *
 * Because the rate is known before the audio path (rsp -> bt) is ever linked,
 * there is no wrong-rate / wrong-pitch window: the first PCM that reaches the SBC
 * encoder is already at the correct rate. */
static esp_err_t start_station_locked(const char *url)
{
    pipeline_codec_t new_codec = detect_codec_from_url(url);
    esp_err_t ret = pipeline_recreate_decoder(new_codec);
    if (ret != ESP_OK) {
        return ret;
    }

    int rate = 0, ch = 0;
    if (rate_cache_get(url, &rate, &ch) == ESP_OK) {
        ESP_LOGI(TAG, "Rate cache hit: %d Hz / %d ch", rate, ch);
    } else if (new_codec == PIPELINE_CODEC_AAC) {
        /* AAC: do NOT probe.
         *
         * The ESP-ADF AAC decoder initializes its getinfo struct with a 44100 Hz
         * default in aac_open() before decoding any ADTS frames.  This value
         * persists in getinfo for the entire probe window (confirmed across two
         * firmware versions and two stability thresholds — 100 ms and 600 ms —
         * in logs 69 and 70), so any getinfo-based probe consistently caches 44100
         * for what is actually a 48000 Hz stream.  The resampler then runs as a
         * 44100→44100 pass-through while the decoder produces 192 KB/s (48000 Hz
         * PCM), overfeeding the BT sink (which absorbs only 176.4 KB/s at 44100 Hz
         * SBC) → sustained L2CAP is_cong bursts → steady choppy audio.
         *
         * AEL_MSG_CMD_REPORT_MUSIC_INFO is also never delivered for AAC in this
         * ADF build (confirmed: no "Stream info —" line in any log), ruling out
         * an event-based probe as well.
         *
         * Default to 48000 Hz — the correct rate for all tested Radio France AAC
         * streams (francemusique-hifi.aac, franceinter_hifi.m3u8).  The resampler
         * is then built as 48000→44100 (proper SRC), which matches the original
         * pipeline_init() default (s_rsp_src_rate = 48000 for AAC) that worked
         * before the probe was introduced.  Cache so reboots are instant. */
        rate = 48000;
        ch   = 2;
        ESP_LOGI(TAG, "AAC (no cache): defaulting to %d Hz / %d ch — caching", rate, ch);
        rate_cache_put(url, rate, ch);
    } else {
        /* Non-AAC (MP3): probe the decoder.  The MP3 decoder reports its rate
         * reliably via getinfo within ~100 ms of stream open (no long-lived init
         * default), making getinfo-based probing accurate and fast for MP3. */
        ret = probe_rate_locked(url, &rate, &ch);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Probe learned %d Hz / %d ch — caching", rate, ch);
            rate_cache_put(url, rate, ch);
        } else {
            rate = 44100;
            ch   = 2;
            ESP_LOGW(TAG, "Probe failed (%d) — falling back to %d Hz / %d ch", ret, rate, ch);
        }
    }

    /* Full restart at the known rate.  Stop+unlink first: after a cache hit the
     * pipeline may still be running the previous station; after a probe it is
     * already stopped+unlinked; at boot it has never run (stop_if_running skips). */
    pipeline_stop_if_running();
    audio_pipeline_unlink(s_pipeline);

    swap_rsp_element_locked(rate, ch);  /* best-effort; logs and keeps old rsp on failure */

    const char *link_tag[] = {"http", "dec", "rsp", "bt"};
    ret = audio_pipeline_link(s_pipeline, link_tag, 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Pipeline link failed: %d", ret);
        return ret;
    }
    audio_element_set_uri(s_http_el, url);
    audio_pipeline_reset_ringbuffer(s_pipeline);
    audio_pipeline_reset_elements(s_pipeline);
    audio_pipeline_change_state(s_pipeline, AEL_STATE_INIT);
    return audio_pipeline_run(s_pipeline);
}

/* Create decoder based on codec type */
static audio_element_handle_t create_decoder(pipeline_codec_t codec)
{
    switch (codec) {
        case PIPELINE_CODEC_AAC:
            return create_aac_decoder();
        case PIPELINE_CODEC_MP3:
        case PIPELINE_CODEC_AUTO:
        default:
            return create_mp3_decoder();
    }
}

/* Detect codec from URL */
static pipeline_codec_t detect_codec_from_url(const char *url)
{
    if (!url) return PIPELINE_CODEC_MP3;
    
    if (strstr(url, ".aac") || strstr(url, "AAC") || strstr(url, "aac")) {
        return PIPELINE_CODEC_AAC;
    }
    if (strstr(url, ".m3u8") || strstr(url, "hls") || strstr(url, "HLS")) {
        /* HLS typically uses AAC */
        return PIPELINE_CODEC_AAC;
    }
    return PIPELINE_CODEC_MP3;
}

static audio_element_handle_t create_a2dp_stream(void)
{
    /* ADF v2.8: type is a2dp_stream_config_t; no A2DP_STREAM_CFG_DEFAULT macro.
     * task_stack / task_prio are not fields of this struct. */
    a2dp_stream_config_t cfg = {
        .type          = AUDIO_STREAM_WRITER,  /* source: we push PCM to BT */
        .user_callback = {
            /* a2dp_stream owns the esp_a2d callback; it invokes this on
             * connection-state events so bt_manager can track link state. */
            .user_a2d_cb = bt_manager_a2dp_state_cb,
        },
        .audio_hal     = NULL,
    };
    return a2dp_stream_init(&cfg);
}

/* ---- public API ---- */

esp_err_t pipeline_init(const uint8_t peer_bda[6], const char *boot_url)
{
    memcpy(s_peer_bda, peer_bda, 6);

    if (!s_pipeline_mutex) {
        s_pipeline_mutex = xSemaphoreCreateMutex();
        if (!s_pipeline_mutex) {
            ESP_LOGE(TAG, "Failed to create pipeline mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    s_pipeline = audio_pipeline_init(&pipeline_cfg);
    if (!s_pipeline) {
        ESP_LOGE(TAG, "pipeline_init failed");
        return ESP_FAIL;
    }

    /* Create the initial decoder matching the first station's codec so the first
     * playback runs on a freshly-created decoder, not one reached via a live
     * MP3->AAC hot-swap. This isolates the decoder from the swap mechanism when
     * diagnosing AAC audio corruption. */
    pipeline_codec_t boot_codec = boot_url ? detect_codec_from_url(boot_url)
                                           : PIPELINE_CODEC_MP3;
    /* Best-guess source rate from the boot codec for the element created here.
     * pipeline_start() immediately re-derives the true rate (NVS cache or probe)
     * and rebuilds the element via swap_rsp_element_locked() if this guess is
     * wrong, so it only needs to be a sane non-identity default (a 1:1 rate would
     * leave the SRC handle NULL — see create_resample_filter). */
    s_rsp_src_rate = (boot_codec == PIPELINE_CODEC_AAC) ? 48000 : 44100;
    s_rsp_src_ch   = 2;
    s_http_el      = create_http_stream();
    s_decoder_el   = create_decoder(boot_codec);
    s_resample_el  = create_resample_filter(s_rsp_src_rate, s_rsp_src_ch);
    s_a2dp_el      = create_a2dp_stream();

    if (!s_http_el || !s_decoder_el || !s_resample_el || !s_a2dp_el) {
        ESP_LOGE(TAG, "Failed to create one or more pipeline elements");
        if (s_http_el)     { audio_element_deinit(s_http_el);     s_http_el = NULL; }
        if (s_decoder_el)  { audio_element_deinit(s_decoder_el);  s_decoder_el = NULL; }
        if (s_resample_el) { audio_element_deinit(s_resample_el); s_resample_el = NULL; }
        if (s_a2dp_el)     { audio_element_deinit(s_a2dp_el);     s_a2dp_el = NULL; }
        audio_pipeline_deinit(s_pipeline);
        s_pipeline = NULL;
        return ESP_FAIL;
    }

    esp_err_t ret = audio_pipeline_register(s_pipeline, s_http_el, "http");
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Register http failed: %d", ret); goto err_cleanup; }
    ret = audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Register dec failed: %d", ret); goto err_cleanup; }
    ret = audio_pipeline_register(s_pipeline, s_resample_el, "rsp");
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Register rsp failed: %d", ret); goto err_cleanup; }
    ret = audio_pipeline_register(s_pipeline, s_a2dp_el, "bt");
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Register bt failed: %d", ret); goto err_cleanup; }

    const char *link_tag[] = {"http", "dec", "rsp", "bt"};
    ret = audio_pipeline_link(s_pipeline, link_tag, 4);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Pipeline link failed: %d", ret); goto err_cleanup; }

    /* Event interface: listen to pipeline + element events */
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    s_evt = audio_event_iface_init(&evt_cfg);
    if (!s_evt) {
        ESP_LOGE(TAG, "Failed to create event interface");
        goto err_cleanup;
    }
    audio_pipeline_set_listener(s_pipeline, s_evt);

    /* NOTE: the A2DP connection is intentionally NOT initiated here.  a2dp_stream
     * has now run esp_a2d_source_init (so the source profile is ready), but the
     * actual page must happen while WiFi is idle — app_main calls
     * bt_manager_connect_a2dp_blocking() before pipeline_start() so the BR/EDR
     * page does not contend with the HTTP stream under BT/WiFi coexistence. */

    s_current_codec = boot_codec;
    ESP_LOGI(TAG, "Pipeline initialized (boot codec %d)", boot_codec);
    return ESP_OK;

err_cleanup:
    if (s_evt) {
        audio_event_iface_destroy(s_evt);
        s_evt = NULL;
    }
    if (s_pipeline) {
        audio_pipeline_unregister(s_pipeline, s_http_el);
        audio_pipeline_unregister(s_pipeline, s_decoder_el);
        audio_pipeline_unregister(s_pipeline, s_resample_el);
        audio_pipeline_unregister(s_pipeline, s_a2dp_el);
        audio_pipeline_deinit(s_pipeline);
        s_pipeline = NULL;
    }
    if (s_http_el)     { audio_element_deinit(s_http_el);     s_http_el = NULL; }
    if (s_decoder_el)  { audio_element_deinit(s_decoder_el);  s_decoder_el = NULL; }
    if (s_resample_el) { audio_element_deinit(s_resample_el); s_resample_el = NULL; }
    if (s_a2dp_el)     { audio_element_deinit(s_a2dp_el);     s_a2dp_el = NULL; }
    return ESP_FAIL;
}

esp_err_t pipeline_start(const char *url)
{
    if (!s_pipeline || !url) return ESP_ERR_INVALID_STATE;

    /* Boot start runs through the same probe/cache path as a station change so the
     * resampler is built for station 0's true rate before any audio is produced.
     * The mutex acquire window covers the probe (up to PROBE_TIMEOUT_US); at boot
     * the event loop has not started yet so there is no real contention. */
    if (xSemaphoreTake(s_pipeline_mutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take pipeline mutex for pipeline_start");
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG, "Starting pipeline -> %s", url);
    esp_err_t ret = start_station_locked(url);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "pipeline_start failed: %d", ret);
    }
    xSemaphoreGive(s_pipeline_mutex);
    return ret;
}

/* Recreate decoder element for new codec type */
static esp_err_t pipeline_recreate_decoder(pipeline_codec_t new_codec)
{
    if (new_codec == s_current_codec) {
        return ESP_OK;  /* No change needed */
    }

    audio_element_handle_t new_decoder = create_decoder(new_codec);
    if (!new_decoder) {
        ESP_LOGE(TAG, "Failed to create new decoder for codec %d", new_codec);
        return ESP_FAIL;
    }

    /* Keep reference to old decoder for potential rollback.
     * Do NOT deinit old_decoder until new one is fully linked. */
    audio_element_handle_t old_decoder = s_decoder_el;

    /* Stop pipeline to safely swap decoder */
    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);

    /* Unlink pipeline before swapping elements */
    audio_pipeline_unlink(s_pipeline);

    /* Unregister old decoder (but don't deinit yet - keep for rollback) */
    audio_pipeline_unregister(s_pipeline, s_decoder_el);
    s_decoder_el = new_decoder;
    esp_err_t ret = audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register new decoder: %d", ret);
        /* Rollback: restore old decoder (still valid, not deinit'd) */
        s_decoder_el = old_decoder;
        audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
        const char *rb_link_tag[] = {"http", "dec", "rsp", "bt"};
        audio_pipeline_link(s_pipeline, rb_link_tag, 4);
        audio_pipeline_run(s_pipeline);
        /* Now safe to deinit failed new decoder */
        audio_element_deinit(new_decoder);
        return ret;
    }

    const char *link_tag[] = {"http", "dec", "rsp", "bt"};
    ret = audio_pipeline_link(s_pipeline, link_tag, 4);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to relink pipeline after decoder swap: %d", ret);
        /* Rollback: restore old decoder (still valid, not deinit'd) */
        s_decoder_el = old_decoder;
        audio_pipeline_unregister(s_pipeline, new_decoder);
        audio_pipeline_register(s_pipeline, s_decoder_el, "dec");
        audio_pipeline_link(s_pipeline, link_tag, 4);
        audio_pipeline_run(s_pipeline);
        /* Deinit failed new decoder only — old decoder was restored, not replaced */
        audio_element_deinit(new_decoder);
        return ret;
    }

    /* Success: new decoder is linked. Now safe to deinit old decoder. */
    audio_element_deinit(old_decoder);
    s_current_codec = new_codec;
    ESP_LOGI(TAG, "Decoder recreated for codec %d", new_codec);
    return ESP_OK;
}

esp_err_t pipeline_change_station(const char *new_url)
{
    if (!s_pipeline || !new_url) return ESP_ERR_INVALID_STATE;

    /* Take mutex to prevent concurrent access with event loop.  The window covers
     * the decode-only probe on a cache miss (up to PROBE_TIMEOUT_US), so allow a
     * generous acquire timeout. */
    if (xSemaphoreTake(s_pipeline_mutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take pipeline mutex for station change");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Changing station -> %s", new_url);

    /* start_station_locked() keeps all elements initialized (never terminates the
     * pipeline), so the A2DP connection survives the change.  It learns the new
     * stream's true rate from the NVS cache or a probe and builds the resampler for
     * it before linking the rsp -> bt audio path — so there is no wrong-rate window
     * and no mid-stream resampler rebuild. */
    esp_err_t ret = start_station_locked(new_url);

    xSemaphoreGive(s_pipeline_mutex);
    return ret;
}

esp_err_t pipeline_stop(void)
{
    if (!s_pipeline) return ESP_OK;
    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);
    audio_pipeline_terminate(s_pipeline);
    ESP_LOGI(TAG, "Pipeline stopped");
    return ESP_OK;
}

void pipeline_deinit(void)
{
    if (!s_pipeline) return;
    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);
    audio_pipeline_terminate(s_pipeline);
    audio_pipeline_unregister(s_pipeline, s_http_el);
    audio_pipeline_unregister(s_pipeline, s_decoder_el);
    audio_pipeline_unregister(s_pipeline, s_resample_el);
    audio_pipeline_unregister(s_pipeline, s_a2dp_el);
    audio_pipeline_deinit(s_pipeline);
    audio_element_deinit(s_http_el);
    audio_element_deinit(s_decoder_el);
    audio_element_deinit(s_resample_el);
    audio_element_deinit(s_a2dp_el);
    audio_event_iface_destroy(s_evt);

    s_pipeline    = NULL;
    s_http_el     = NULL;
    s_decoder_el  = NULL;
    s_resample_el = NULL;
    s_a2dp_el     = NULL;
    s_evt         = NULL;

    if (s_pipeline_mutex) {
        vSemaphoreDelete(s_pipeline_mutex);
        s_pipeline_mutex = NULL;
    }
    ESP_LOGI(TAG, "Pipeline destroyed");
}

audio_event_iface_handle_t pipeline_get_event_iface(void)
{
    return s_evt;
}

audio_element_handle_t pipeline_get_decoder_el(void)
{
    return s_decoder_el;
}
