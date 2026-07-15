// Board power backend, one API for two very different designs:
//  - AMOLED 1.8: AXP2101 PMU over I2C (0x34) - VBUS sense, soft power-off,
//    battery gauge.
//  - LCD 1.85: no PMU. Discrete GPIO power latch (GPIO7 held high keeps the
//    board on), power button on GPIO6, battery voltage on the GPIO1 ADC.
// The backend is selected at compile time from BOARD_LCD_1_85 / BOARD_AMOLED_1_8.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Latch the board power on as early as possible (LCD 1.85: drive the
// power-hold GPIO high so releasing the button doesn't cut power). No-op
// where the PMU latches in hardware (AMOLED). Call first thing in app_main.
void pmu_power_hold_early(void);

// bring up the power backend (probe the PMU / set up the ADC + button) and
// start the monitor task; returns false if the expected hardware is absent
bool pmu_start(int i2c_port);

// called from the monitor task when USB power is removed (debounced);
// typical use: persist state, then pmu_power_off()
void pmu_set_unplug_callback(void (*cb)(void));

// called from the monitor task on a short press of the PWR button
// (long press is a hardware force-off, power-on from off is hardware too)
void pmu_set_powerkey_callback(void (*cb)(void));

// true while USB power is present
bool pmu_vbus_present(void);

// battery state of charge 0..100, or -1 if unavailable
int pmu_battery_percent(void);

// battery voltage in millivolts (cached, refreshed ~1s), or -1 if unavailable
int pmu_battery_voltage_mv(void);

// cut the battery rails - device powers off (wakes on power button / USB)
void pmu_power_off(void);

#ifdef __cplusplus
}
#endif
