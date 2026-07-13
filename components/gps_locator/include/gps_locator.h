#pragma once

#include "lvgl.h"
#include "TrackLog.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAP_TILES_TILE_SIZE 256
// 5x5 grid = 25 tiles x 128KB = 3.2MB PSRAM (of 8MB); loaded area 1280px
#define MAP_TILES_GRID_COLS 5
#define MAP_TILES_GRID_ROWS 5
#define MAP_TILES_BYTES_PER_PIXEL 2
#define MAP_TILES_COLOR_FORMAT LV_COLOR_FORMAT_RGB565
#define MAP_ZOOM 16
#define BASE_PATH "/sdcard"
#define TILE_FOLDER "tiles1"
#define STEP_ANIMATION_DURATION 500

class GPSLocator {
public:
    static lv_obj_t** tile_components;

    // Initialize the simple map system (width/height = visible map viewport)
    static bool init(lv_obj_t* parent_screen, int width, int height);
    
    // Display map at specific coordinates
    static void show_initial_location(double latitude, double longitude);

    // Update location (for moving GPS)
    static void move_location(double latitude, double longitude);

    // The scrollable LVGL container (attach touch/browse listeners here)
    static lv_obj_t* get_container();

    // Screen offset of a GPS position relative to the viewport center
    // (for drawing the vehicle marker while browsing)
    static void get_screen_offset(double latitude, double longitude, int *dx, int *dy);

    static bool fetch_images_from_sd(int index, int tile_x, int tile_y);

    // Draw a recorded run as a polyline over the map (segments colored by
    // speed); stays glued to the tiles across panning and grid shifts
    static void track_show(const track_view_pt_t *pts, int n);
    static void track_clear();

    // Cleanup
    static void cleanup();

private:
    static lv_obj_t* map_container;
    static lv_obj_t* map_group;
    static int tile_count;
    static int top_left_tile_x;
    static int top_left_tile_y;
    static int new_top_left_tile_x;
    static int new_top_left_tile_y;
    static int marker_offset_x;
    static int marker_offset_y;
    static int new_marker_offset_x;
    static int new_marker_offset_y;
    static bool initialized;
    static bool is_loading;
    static int map_w;
    static int map_h;
    static int tile_mapping[MAP_TILES_GRID_ROWS * MAP_TILES_GRID_COLS];

    static void create_tile_components();
    static void pan_event_cb(lv_event_t *e);
    static void pan_by(int dx, int dy);
    static bool load_tile_images();
    static void set_fallback_tile(int col);
    static void get_tile_coordinates(double latitude, double longitude, double &tile_x, double &tile_y);
    static void get_marker_offsets(double &tile_x, double &tile_y, int &offset_x, int &offset_y);
    
    static void deinit();
    static void center_map_on_gps();
    static void animate_map_center();
    static void shift_slots_x(int dir);
    static void shift_slots_y(int dir);
    static void shift_grid(int sx, int sy);
    static void queue_cell(int index);
    static void tile_loader_task(void *param);
    static bool read_tile_to_buffer(uint8_t *dst, int tile_x, int tile_y);
    static bool is_location_within_center(int new_latitude, int new_longitude);
    static void show_loading_popup();
    static void hide_loading_popup();
};

#ifdef __cplusplus
}
#endif