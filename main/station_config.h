#pragma once

/*
 * station_config.h — Prototype test configuration
 *
 * Edit WIFI_SSID / WIFI_PASS before flashing.
 * Stream URLs verified via https://www.radio-browser.info
 *
 * BT_PEER_ADDR: set to your Bluetooth speaker MAC address (LSB first).
 * Leave all zeros to enable scan mode (connects to first A2DP sink found).
 */

/* ---------- WiFi (hotspot of your phone) ---------- */
#define WIFI_SSID   "YourHotspotSSID"
#define WIFI_PASS   "YourHotspotPassword"

/* ---------- Bluetooth speaker ---------- */
/* MAC address of the target Bluetooth speaker, byte[0] = LSB.
 * Example: speaker at AA:BB:CC:DD:EE:FF → {0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA}
 * Set all to 0 to scan and connect to first A2DP sink found. */
#define BT_PEER_ADDR    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define BT_DEVICE_NAME  "ESP-ADF-Streamer"

/* ---------- Test stations ----------
 *
 * Phase B test order: MP3 first (simplest), then AAC, then HLS.
 * Replace with actual resolved URLs from radio-browser.info for your region.
 *
 * Recommended sources:
 *   MP3:  http://stream.radiostyle.fr:8000/stream  (or any Icecast /stream)
 *   AAC:  Use radio-browser.info → filter codec=AAC → copy url_resolved
 *   HLS:  BBC streams end in .m3u8; Radio France also provides HLS
 */

/* Phase B step 1: plain MP3 Icecast (baseline) */
#define STATION_MP3_URL \
    "http://icecast.radiofrance.fr/franceinfo-midfi.mp3"

/* Phase B step 2: AAC Icecast */
#define STATION_AAC_URL \
    "https://icecast.radiofrance.fr/francemusique-hifi.aac"

/* Phase B step 3 (critical): HLS master playlist */
#define STATION_HLS_URL \
    "https://stream.radiofrance.fr/franceinter/franceinter_hifi.m3u8"

/* Phase B step 4: HLS with master+variant playlists (multi-bitrate) */
#define STATION_HLS_MULTI_URL \
    "https://a.files.bbci.co.uk/media/live/manifesto/audio/simulcast/hls/nonuk/sbr_low/ak/bbc_radio_one.m3u8"

/* Station table — used for rotation test in Phase D */
typedef struct {
    const char *name;
    const char *url;
} station_t;

#define NUM_TEST_STATIONS 4
