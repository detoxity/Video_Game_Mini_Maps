#!/usr/bin/env bash
# Flash the ESP32-S3-Touch-LCD-1.85 board.
# Usage: ./flash_lcd_1_85.sh [port]   (default /dev/ttyACM0, USB-Serial-JTAG)
set -e
PORT="${1:-/dev/ttyACM0}"
IDF_PATH="${IDF_PATH:-$HOME/esp/esp-idf}"

if [ ! -f "$IDF_PATH/export.sh" ]; then
    echo "ESP-IDF not found at $IDF_PATH - install it or set IDF_PATH" >&2
    exit 1
fi
# shellcheck disable=SC1091
source "$IDF_PATH/export.sh"

export BOARD=lcd_1_85
cd "$(dirname "$0")"
idf.py -p "$PORT" build flash
