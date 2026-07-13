// Shared application state (defined in main.cpp) and the color palettes.
#pragma once

#include <stdint.h>
#include "lvgl.h"

// screens / widgets built by main.cpp
extern lv_obj_t *main_scr;
extern lv_obj_t *no_satellite_bg;
extern lv_obj_t *map_container;
extern lv_obj_t *car_icon_img;
extern lv_obj_t *north_pointer_img;

// vehicle state fed by the GPS driver / demo / CAN.
// C linkage: the C driver headers (GPS_UART_Driver.h etc.) declare these too.
#ifdef __cplusplus
extern "C" {
#endif
extern float new_latitude;
extern float new_longitude;
extern float gps_speed_kmh;
extern int gps_sat_count;
extern bool receiving_data;
extern volatile bool data_ready;
#ifdef __cplusplus
}
#endif

extern float current_latitude;
extern float current_longitude;
extern int current_angle;
extern int new_angle;
extern bool location_initialized;

// touch browsing
extern bool follow_vehicle;
extern uint32_t last_touch_tick;

// a saved run's track is being reviewed on the map (suspends the
// return-to-vehicle timeout; see ui_overlays track_view_*)
extern bool track_view_active;

// palettes
extern const lv_color_t PALETTE_BLACK;
extern const lv_color_t PALETTE_BLUE;
extern const lv_color_t PALETTE_BLUE_NEON;
extern const lv_color_t PALETTE_DARK_GREY;
extern const lv_color_t PALETTE_RED;
extern const lv_color_t PALETTE_GREEN;
extern const lv_color_t PALETTE_GREY;
extern const lv_color_t PALETTE_WHITE;
extern const lv_color_t PALETTE_NFS_WHITE;
extern const lv_color_t PALETTE_NFS_BLUE;
extern const lv_color_t PALETTE_NFS_CYAN;
extern const lv_color_t PALETTE_NFS_GREEN;
extern const lv_color_t PALETTE_NFS_CITRUS;
extern const lv_color_t PALETTE_NFS_LIME;
extern const lv_color_t PALETTE_NFS_ORANGE;
extern const lv_color_t PALETTE_NFS_RED;
extern const lv_color_t PALETTE_NFS_PURPLE;
extern const lv_color_t PALETTE_NFS_GREY;
extern const lv_color_t PALETTE_NFS_BLUE2;
extern const lv_color_t PALETTE_NFS_YELLOW;
