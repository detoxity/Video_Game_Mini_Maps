// Position persistence (NVS) and the PMU power-off sequence.
#pragma once

// NVS init + read the last saved position into current/new lat/lon
// (falls back to HOME_LAT/HOME_LON)
void persistence_init(void);

// throttled periodic save - no-op unless SAVE_LAST_POSITION is 1
void save_last_position(void);

// unconditional one-shot save (used by the shutdown sequence)
void save_position_now(void);

// hook the AXP2101: USB unplug / PWR short press (on battery) ->
// "Saving..." overlay -> save -> power off. No-op without USE_PMU.
void persistence_pmu_init(void);
