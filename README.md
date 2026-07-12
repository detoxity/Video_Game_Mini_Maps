# Video Game Mini Map

A video-game-style GPS mini map on ESP32-S3 with real u-blox GPS, offline
OSM tiles from SD card, a Dragy-style performance meter with IMU launch
fusion, and battery/power management.

Based on the Garage Tinkering project featured in
https://youtu.be/sAp7oCB939c (original ESP32-P4 version in git history).

## Hardware

Two Waveshare ESP32-S3 boards are supported, selected with the `BOARD`
environment variable at build time:

| BOARD (env)  | Board                             | Display                     | BSP |
|--------------|-----------------------------------|-----------------------------|-----|
| `lcd_1_85`   | ESP32-S3-Touch-LCD-1.85 (default) | 360x360 round, ST77916 QSPI | custom, `boards/bsp_ws_s3_touch_lcd_1_85` |
| `amoled_1_8` | ESP32-S3-Touch-AMOLED-1.8         | 368x448 AMOLED, CO5300 QSPI | vendored + patched, `boards/esp32_s3_touch_amoled_1_8` |

Peripherals (all optional except the SD card):

- **GPS**: u-blox SAM-M8Q or M10 on the back UART header —
  GPS TX → GPIO44, GPS RX → GPIO43, 3V3, GND. Any baud rate: the driver
  auto-detects and reconfigures the module at boot (RAM-only, unbrickable).
- **Compass**: QMC5883L on the I2C header (heading at standstill).
- **Battery** (AMOLED 1.8): the on-board AXP2101 handles charging;
  firmware saves position and powers off on USB unplug / power button.
- **SD card**: FAT32 on MBR (**not GPT** — mounts fail with error 13),
  holding the map tiles (`tiles1/z/x/y.bin`).

## Quick start

1. **Install ESP-IDF v5.5** (https://docs.espressif.com/projects/esp-idf/ —
   `git clone -b v5.5 ... && ./install.sh esp32s3`).

2. **Clone with submodules**:

   ```
   git clone --recursive <this repo>
   ```

3. **Prepare the SD card**: format FAT32/MBR and copy a tile set to
   `tiles1/` in the card root. A ready pipeline for generating tiles for
   any city from OpenStreetMap is in [tools/map_generator](tools/map_generator/README.md)
   (the shipped configuration covers Kyiv at zoom 16, petrol stations included).

4. **Configure** (optional): every feature toggle lives in
   [main/app_config.h](main/app_config.h) — GPS source
   (`UART` real GPS / `DEMO` drag-strip simulator / `CAN` legacy), home
   position for first boot, overlays, IMU, PMU, auto-rotate, perf meter.
   No GPS module yet? Set `GPS_SOURCE` to `GPS_SOURCE_DEMO` to see the
   map and performance meter working immediately.

5. **Flash** (board connected over USB-C; the same port is the serial console):

   Linux/macOS:
   ```
   ./flash_lcd_1_85.sh   [/dev/ttyACM0]     # LCD 1.85
   ./flash_amoled_1_8.sh [/dev/ttyACM0]     # AMOLED 1.8
   ```

   Windows (PowerShell):
   ```
   .\flash_lcd_1_85.ps1   [COM4]
   .\flash_amoled_1_8.ps1 [COM15]
   ```

   The scripts build with the right `BOARD` and flash. Each board uses its
   own build directory, so both can stay built side by side. The `.sh`
   scripts source `$IDF_PATH/export.sh` (default `~/esp/esp-idf`); the
   `.ps1` scripts assume the standard Windows install under `%USERPROFILE%`.

6. **First boot**: the map starts at the last saved position (or the
   configured home) and jumps to the vehicle on the first GPS fix. Cold
   TTFF with a clear sky view is typically 30-60 s.

## Controls

| Input                        | Action |
|------------------------------|--------|
| Drag on map                  | free 2D pan (auto-returns to vehicle after 15 s) |
| BOOT click                   | toggle battery + satellites overlay |
| BOOT double-click            | record mode on/off (red dot; perf overlay, run saving and GPS perf-rate only while recording) |
| BOOT hold (~1 s)             | run history (last 30 runs; swipe left on an entry to delete) |
| PWR short press (on battery) | save position and power off (AMOLED 1.8) |

## Debugging

All logging is compiled out for performance
(`CONFIG_LOG_DEFAULT_LEVEL_NONE` in `sdkconfig.defaults`). To get logs
back: delete the build dir's `sdkconfig`, set the level to Info in
`sdkconfig.defaults` (or via `menuconfig`), rebuild. Map/pan diagnostics
can be enabled separately with `PAN_DIAG` in
`components/gps_locator/gps_locator.cpp` (plain printf, works even with
logging compiled out). Serial console runs over USB-Serial-JTAG:
`idf.py monitor` or any terminal at 115200.

## License

[CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)
(Attribution-NonCommercial) — free to use and modify, but not
commercially. Fork of the
[Garage Tinkering project](https://github.com/garagetinkering/Video_Game_Mini_Maps),
which set these terms.

Third-party parts keep their own licenses (Waveshare AMOLED BSP:
Apache-2.0; 0015/map_tiles converter submodule: MIT) — see
[LICENSE](LICENSE). Map tiles are rendered from
[OpenStreetMap](https://www.openstreetmap.org/copyright) data,
© OpenStreetMap contributors, ODbL.
