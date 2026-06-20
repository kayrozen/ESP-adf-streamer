#pragma once

/*
 * station_config.h — Prototype test configuration
 *
 * Edit WIFI_SSID / WIFI_PASS before flashing.
 * Stream URLs verified via https://www.radio-browser.info
 *
 * BT_PEER_ADDR: set to your Bluetooth speaker MAC address (MSB first —
 * same order as printed on the device label, e.g. AA:BB:CC:DD:EE:FF).
 * Leave all zeros to enable scan mode (connects to first A2DP sink found).
 */

/* ---------- WiFi (hotspot of your phone) ---------- */
#define WIFI_SSID   "YourHotspotSSID"
#define WIFI_PASS   "YourHotspotPassword"

/* ---------- Bluetooth speaker ---------- */
/* MAC address of the target Bluetooth speaker, byte[0] = MSB.
 * Enter exactly as shown on the device label or in your phone's BT settings.
 * Example: speaker at AA:BB:CC:DD:EE:FF → {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
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

/* Phase B step 3: France Inter AAC Icecast.
 * History: BBC World Service HLS on Akamai geo-blocks Canada (log 59).  Replaced
 * with CBC Radio One HLS on akamaihd.net — that CDN was decommissioned (log 64:
 * stream does not open, rotation stops at CBC).  Replaced with France Inter AAC on
 * the same icecast.radiofrance.fr CDN proven reliable in all recent logs. */
#define STATION_HLS_URL \
    "https://icecast.radiofrance.fr/franceinter-hifi.aac"

/* Phase B step 4: second HTTPS AAC Icecast (France Culture).
 * BBC Radio 1 HLS nonuk/sbr_low path returned HTTP 410 Gone in logs 48/49 —
 * the nonuk CDN tier has been decommissioned by the BBC.  Replaced with France
 * Culture AAC Icecast which is known reachable from the same network. */
#define STATION_HLS_MULTI_URL \
    "https://icecast.radiofrance.fr/franceculture-hifi.aac"

/* Station table — used for rotation test in Phase D */
typedef struct {
    const char *name;
    const char *url;
} station_t;

#define NUM_TEST_STATIONS 4
