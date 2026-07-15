// =============================================================================
// app_config.h - every feature toggle and tunable in one place.
// This is the file to edit; the rest of main/ implements what's set here.
// =============================================================================
#pragma once

#include "driver/gpio.h"

// ---- GPS input source -------------------------------------------------------
//   GPS_SOURCE_CAN  - CANBus messages with ID 0x430 (external transceiver on GPIO43/44)
//   GPS_SOURCE_DEMO - drag-strip simulator, no hardware needed (desk testing)
//   GPS_SOURCE_UART - u-blox module on the UART header (GPIO43=TX, GPIO44=RX);
//                     auto-baud, auto-config (18/25Hz), UBX NAV-PVT + NMEA
#define GPS_SOURCE_CAN  0
#define GPS_SOURCE_DEMO 1
#define GPS_SOURCE_UART 2

#define GPS_SOURCE GPS_SOURCE_UART

// ---- optional hardware ------------------------------------------------------
// QMC5883L compass (e.g. on FlyfishRC GPS combos) via the I2C header:
// rotates the car icon by real heading while standing still. Falls back
// to GPS-track heading automatically when absent.
#define USE_COMPASS 0

// on-board QMI8658 accelerometer refines the perf-meter launch instant
// (GNSS+IMU fusion) - set 0 to rely on GNSS interpolation alone
#define USE_IMU 1

// auto-flip the screen 180deg when the device is mounted upside down.
// Uses the panel's hardware mirror (zero render cost) and disables
// itself if the display driver can't do it. Needs USE_IMU.
#define AUTO_ROTATE 1
// accelerometer axis along the screen's vertical (0=X, 1=Y, 2=Z) and a
// sign flip - adjust these if the screen flips the wrong way when the
// device is turned on your mounting
#define AUTO_ROTATE_AXIS   0   // measured: gravity on X on the AMOLED 1.8
#define AUTO_ROTATE_INVERT 1   // +10 m/s^2 = upright on this board

// Battery power management. AMOLED 1.8: AXP2101 PMU (USB unplug or PWR short
// press shows "Saving...", saves position, powers off). LCD 1.85: no PMU -
// discrete GPIO power latch (held on boot) + PWR long-press to save & off +
// battery voltage on the ADC. Backend picked at compile time by board.
#define USE_PMU 1

// ---- on-screen overlays -----------------------------------------------------
// gold edge ring on the round display (vector-drawn, cheap; still off by
// default for a clean full-bleed map). Only applies to the LCD 1.85.
#define SHOW_RING 0

// speed readout, bottom center
#define SHOW_SPEED 1

// battery voltage + satellite count, top-left;
// BOOT single click toggles it (state persists across power cycles)
#define SHOW_BATTERY 1

// performance meter: BOOT double click arms record mode (red dot) -
// runs then show live 0-60/0-100/100-200/402m and are saved to the
// history screen (hold BOOT; swipe an entry left to delete).
// Record mode also switches the GPS module: cruise = multi-GNSS 10Hz
// (best TTFF/accuracy), record = GPS-only at max rate (18-25Hz timing).
#define SHOW_PERF 1
// calibration offset in milliseconds applied to all perf-meter times.
// Set this to match a reference Dragy or other trusted timing source.
#define PERF_CALIBRATION_OFFSET_MS 0
#define HISTORY_MAX 30

// runs are timestamped from GPS UTC time (UBX NAV-PVT carries date/time
// on both M8 and M10); local time = UTC + this offset in hours.
// Kyiv: +2 winter / +3 summer (no DST logic - set for the season)
#define UTC_OFFSET_HOURS 3

// ---- buttons ----------------------------------------------------------------
#define BOOT_BUTTON_GPIO GPIO_NUM_0

// ---- position / persistence -------------------------------------------------
// the map boots at the last saved position; factory default is Kyiv
#define HOME_LAT 50.4501f   // Maidan Nezalezhnosti
#define HOME_LON 30.5245f

// 1 = also persist the position periodically (max 1 write/minute);
// 0 = flash is written only by the PMU shutdown sequence
#define SAVE_LAST_POSITION 0

// ---- map behaviour ----------------------------------------------------------
#define MIN_MOVE_DISTANCE     2.0     // meters of movement before the icon rotates
#define PAN_RETURN_TIMEOUT_MS 15000   // return to vehicle after browse inactivity
