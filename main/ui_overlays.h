// On-screen overlays and the BOOT button UI:
//   - speed readout (bottom center)
//   - battery voltage + satellite count (top-left, BOOT single click)
//   - performance meter overlay + record mode (BOOT double click, red dot)
//   - run history screen (hold BOOT; swipe left deletes an entry)
#pragma once

#include "lvgl.h"

// build all overlay widgets on the given parent (call inside the LVGL lock)
void overlays_create(lv_obj_t *parent);

// periodic servicing - call from the main LVGL timer (~10ms)
void overlays_tick(void);

// current visibility of the battery overlay (persisted at shutdown)
bool overlays_battery_visible(void);
