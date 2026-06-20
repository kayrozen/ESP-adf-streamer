# ESP-ADF Streamer

Streams internet radio (MP3, AAC, HLS) over WiFi and plays it through a Bluetooth A2DP speaker. Targets the **TTGO T7 Mini32 V1.5** (ESP32-WROVER-B, 16 MB flash, 8 MB PSRAM).

---

## Hardware required

| Item | Notes |
|------|-------|
| TTGO T7 Mini32 V1.5 (ESP32-WROVER-B) | Other WROVER boards with 8 MB PSRAM should work |
| Bluetooth A2DP speaker | Put it in pairing mode before first boot |
| USB-C cable | Data-capable (not charge-only) |
| Phone hotspot or 2.4 GHz WiFi AP | The ESP32 only supports 2.4 GHz |

---

## Flash

The CI pipeline publishes a browser-based installer to GitHub Pages on every push to `main`. Open the install page in **Chrome or Edge** (WebSerial required — Firefox does not support it):

```
https://kayrozen.github.io/esp-adf-streamer/
```

1. Click **Flash** and select the USB serial port for your board.
2. Wait for the three binaries (bootloader, partition table, firmware) to write — takes ~30 s.
3. The board reboots automatically when flashing is complete.

To flash manually with `idf.py`:

```bash
idf.py -p /dev/ttyUSB0 flash
```

---

## Provision WiFi and Bluetooth

After flashing, the firmware listens on UART0 (115200 baud) for a JSON provisioning command. Credentials are stored in NVS and survive reboots.

### Option A — browser (recommended)

On the same install page, after flashing:

1. Enter your WiFi SSID, password, and Bluetooth speaker MAC address (as printed on the device label, e.g. `AA:BB:CC:DD:EE:FF`).
2. Click **Provision via UART** — the page opens a Web Serial connection and sends the command.
3. The board saves to NVS and reboots.

### Option B — serial terminal

Send the following line (replace values, end with `\n`):

```
PROVISION:{"ssid":"YourSSID","pass":"YourPassword","btmac":"AA:BB:CC:DD:EE:FF"}
```

Example with `screen`:

```bash
screen /dev/ttyUSB0 115200
# paste the PROVISION: line, press Enter
```

The board replies `OK` and reboots. If it replies `ERROR:…`, check that the JSON is valid.

**BT MAC format:** Enter exactly as shown on the speaker label (`AA:BB:CC:DD:EE:FF`). Leave the `btmac` field out or set it to all zeros to scan for the nearest A2DP sink instead.

---

## What to check after flashing

Open a serial monitor at **115200 baud** and watch the boot log.

### 1 — Boot heap (Phase A)

Within the first second you should see:

```
I (app_main): === ESP-ADF Streamer prototype ===
I (app_main): Heap at boot: internal=XXXXB  SPIRAM=YYYYB
```

**Pass:** `SPIRAM` is ≥ 4 MB (4 194 304 B). If you see `PSRAM not detected`, check that the board is a WROVER variant and that `CONFIG_SPIRAM` is enabled in sdkconfig.

### 2 — WiFi (Phase A)

```
I (wifi_mgr): Connected — IP: 192.168.x.x
```

**Fail indicators:**
- `WiFi connect failed` — check SSID/password, confirm 2.4 GHz band.
- Repeated `retry` lines — weak signal or DHCP timeout; move the board closer to the AP.

### 3 — Bluetooth discovery (Phase A)

```
I (bt_mgr): A2DP Sink found: AA:BB:CC:DD:EE:FF
```

or (if MAC was provisioned):

```
I (bt_mgr): Using configured peer BDA
```

**Fail:** `No A2DP sink found within 30 s` — make sure the speaker is in pairing mode and within 1–2 m. If the speaker is already paired to another device, unpair it first.

### 4 — A2DP connection

```
I (bt_mgr): A2DP connected
```

The firmware waits up to 10 s for this before starting the pipeline.

**Fail:** No `A2DP connected` line — the speaker may have accepted discovery but then timed out the connection. Power-cycle the speaker and reflash or re-provision.

### 5 — Stream info (Phase B)

Within a few seconds of `A2DP connected` you should hear audio, and the log should show:

```
I (app_main): Stream info — codec:X  sample_rate:44100  channels:2  bits:16
```

**Codec values:** `0` = MP3, `2` = AAC. The first station (`STATION_MP3_URL`) should report `codec:0`.

**Fail indicators:**
- `Stream error (status=X) — retrying in 3s` — network issue or the stream URL has changed. See [Updating stream URLs](#updating-stream-urls).
- Continuous retries with no audio — the A2DP write pipeline may be stalled; power-cycle and try again.

### 6 — Resource monitor (Phase C)

Every 30 s (default `MONITOR_INTERVAL_S`):

```
I (monitor): HEAP  internal:  120 KB free  (min ever:   98 KB) | PSRAM: 5432 KB free  (min ever: 5400 KB)
I (monitor): CPU runtime stats:
  ...
```

**Watch for:** internal heap `min ever` approaching 0 — indicates a memory leak or under-provisioned stack. PSRAM min should stay well above 1 MB.

---

## Phase D — station rotation (optional)

Enable in sdkconfig by setting `CONFIG_PROTOTYPE_PHASE_D_ROTATION=y` (defined in `main/Kconfig.projbuild`). The firmware will automatically cycle through all four test stations (MP3 → AAC → HLS mono → HLS multi-bitrate) at 30 s intervals without dropping the Bluetooth connection.

Check the log for:

```
I (app_main): Switching to station 1: AAC Icecast
I (app_main): Station switch took 1234 ms
I (app_main): Phase D passthrough: bytes=12345678  frames=56789
```

Each switch should take under 5 s. Audio should resume on the Bluetooth speaker within that window.

---

## Updating stream URLs

Stream URLs are baked into the firmware at build time (`main/station_config.h`). If a URL goes dead:

1. Find a replacement on [radio-browser.info](https://www.radio-browser.info) — use the **url_resolved** field.
2. Edit `main/station_config.h`.
3. Rebuild and reflash.

For quick testing without a rebuild, open the serial monitor and send a raw HTTP URL to verify connectivity manually.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| No PSRAM detected | Wrong board or sdkconfig | Verify `CONFIG_SPIRAM=y`, `CONFIG_SPIRAM_USE_CAPS_ALLOC=y` |
| WiFi connects but stream never starts | Stream URL dead or geo-blocked | Replace URL in `station_config.h` |
| Audio stutters | Insufficient internal heap | Check heap monitor; reduce task stacks if near 0 |
| BT keeps disconnecting | Speaker power-save timeout | Keep speaker plugged in; some speakers disconnect after 30 s of silence |
| `cJSON_Parse failed` during provisioning | Malformed JSON | Confirm no extra spaces/quotes around the `PROVISION:` prefix |
| `ERROR:` response to PROVISION | NVS write failed | Erase NVS with `idf.py erase-flash` and retry |

---

## Build locally

Prerequisites: ESP-IDF v5.3.1, ESP-ADF v2.8.

```bash
export ADF_PATH=/path/to/esp-adf
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```
