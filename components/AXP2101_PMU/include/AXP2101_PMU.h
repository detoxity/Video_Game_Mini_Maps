// Minimal AXP2101 PMU driver (ESP32-S3-Touch-AMOLED-1.8 and similar):
// USB (VBUS) presence monitoring, soft power-off, battery gauge.
// Sits on the board's shared I2C bus at address 0x34.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// probe the PMU on an already-initialized I2C bus and start the VBUS
// monitor task; returns false if no AXP2101 is present (e.g. LCD 1.85)
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
