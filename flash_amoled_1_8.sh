#!/usr/bin/env bash
# Flash the ESP32-S3-Touch-AMOLED-1.8 board.
# Usage: ./flash_amoled_1_8.sh [port]   (default /dev/ttyACM0, USB-Serial-JTAG)
set -e
PORT="${1:-/dev/ttyACM0}"
IDF_PATH="${IDF_PATH:-$HOME/esp/esp-idf}"

if [ ! -f "$IDF_PATH/export.sh" ]; then
    echo "ESP-IDF not found at $IDF_PATH - install it or set IDF_PATH" >&2
    exit 1
fi
# shellcheck disable=SC1091
source "$IDF_PATH/export.sh"

export BOARD=amoled_1_8
cd "$(dirname "$0")"
# separate build dir so both boards can stay built side by side
idf.py -B build_amoled -D SDKCONFIG=build_amoled/sdkconfig -p "$PORT" build flash
