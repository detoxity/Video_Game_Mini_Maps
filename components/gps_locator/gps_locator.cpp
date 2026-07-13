#include "gps_locator.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "images/area_locked_tile.h"

LV_IMG_DECLARE(area_locked_tile);

// largest believable single-read pan vector; bigger = touch glitch (see
// pan_event_cb)
#define MAX_PAN_STEP_PX 120

// pan/tile diagnostics: plain printf so it survives
// CONFIG_LOG_DEFAULT_LEVEL_NONE (this is how the stale-coordinate
// double-shift bug was caught - see group_pos_x below)
#define PAN_DIAG 0
#if PAN_DIAG
#define PAN_LOG(...) printf(__VA_ARGS__)
#else
#define PAN_LOG(...)
#endif

// set 1 to restore verbose map logging - printf on the LVGL thread is a
// measurable performance cost at 18Hz GPS updates
#define MAP_DEBUG_LOG 0
#if MAP_DEBUG_LOG
#define MAP_LOG(...) printf(__VA_ARGS__)
#else
#define MAP_LOG(...)
#endif

lv_obj_t* GPSLocator::map_container = nullptr;
lv_obj_t* GPSLocator::map_group = nullptr;
lv_obj_t** GPSLocator::tile_components = nullptr;
int GPSLocator::top_left_tile_x = 0;
int GPSLocator::top_left_tile_y = 0;
int GPSLocator::new_top_left_tile_x = 0;
int GPSLocator::new_top_left_tile_y = 0;
int GPSLocator::new_marker_offset_x = 0;
int GPSLocator::new_marker_offset_y = 0;
int GPSLocator::marker_offset_x = 0;
int GPSLocator::marker_offset_y = 0;
int GPSLocator::tile_count = 0;
bool GPSLocator::initialized = false;
bool GPSLocator::is_loading = false;
int GPSLocator::map_w = 0;
int GPSLocator::map_h = 0;

struct TileSlot {
    uint8_t* buf;          // Pixel buffer
    lv_image_dsc_t img;    // LVGL image descriptor
    int tx, ty;            // tile coordinates currently held in buf
    bool valid;            // buf holds a fully loaded tile for (tx, ty)
};

TileSlot* tiles = nullptr;

// Authoritative map_group position. lv_obj_get_x/y() only reflect
// lv_obj_set_pos() after the next layout refresh, and two touch reads can
// land between renders - reading the stale coordinate made pan_by repeat
// its grid-shift decision, teleporting the map a whole tile ("pans too
// far"). Every writer of the group position must go through these.
static int group_pos_x = 0;
static int group_pos_y = 0;

// Background tile loading: cells show the "area locked" placeholder the
// moment the grid shifts and get filled by a single loader task. Jobs are
// stamped with a grid generation so pans faster than the SD card simply
// drop the outdated loads.
struct TileJob {
    int index;              // grid cell
    int tx, ty;             // tile coordinates to load
    uint32_t gen;           // grid generation the job was created for
};
struct TileApply {
    TileJob job;
    bool ok;
};
static QueueHandle_t tile_load_queue = nullptr;
static SemaphoreHandle_t scratch_free = nullptr;
static uint8_t *scratch_buf = nullptr;
static volatile uint32_t grid_gen = 0;
static const size_t TILE_IMG_SIZE = MAP_TILES_TILE_SIZE * MAP_TILES_TILE_SIZE * MAP_TILES_BYTES_PER_PIXEL;

bool GPSLocator::init(lv_obj_t* parent_screen, int width, int height) {
    if (is_loading) return true;

    map_w = width;
    map_h = height;
    tile_count = MAP_TILES_GRID_COLS * MAP_TILES_GRID_ROWS;

    tiles = (TileSlot*)calloc(tile_count, sizeof(TileSlot));
    if (!tiles) {
        MAP_LOG("Failed to allocate tile slots\n");
        return false;
    }

    if (!tile_load_queue) {
        tile_load_queue = xQueueCreate(64, sizeof(TileJob));
        scratch_free = xSemaphoreCreateBinary();
        xSemaphoreGive(scratch_free);
        scratch_buf = (uint8_t*)heap_caps_malloc(TILE_IMG_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!tile_load_queue || !scratch_free || !scratch_buf) {
            MAP_LOG("Failed to allocate tile loader resources\n");
            return false;
        }
        // UI core: tile streaming belongs with LVGL, not with the
        // GPS/IMU measurement tasks on core 1
        xTaskCreatePinnedToCore(tile_loader_task, "tile_loader", 4096, NULL, 5, NULL, 0);
    }

    // Create map container. Panning is handled manually from the touch
    // vector: LVGL's native scroll locks each gesture to a single axis
    // (see lv_indev_scroll_handler), which makes diagonal panning impossible.
    map_container = lv_obj_create(parent_screen);
    lv_obj_set_size(map_container, width, height);
    lv_obj_center(map_container);
    lv_obj_set_style_pad_all(map_container, 0, 0);
    lv_obj_set_style_border_width(map_container, 0, 0);
    lv_obj_set_style_radius(map_container, 0, 0);
    lv_obj_set_style_bg_color(map_container, lv_color_make(0,0,0), 0);
    lv_obj_remove_flag(map_container, LV_OBJ_FLAG_SCROLLABLE);

    create_tile_components();

    initialized = true;
    return true;
}

void GPSLocator::create_tile_components() {
    if (!tile_components) {
        tile_components = (lv_obj_t**)calloc(tile_count, sizeof(lv_obj_t*));
        if (!tile_components) {
            MAP_LOG("Failed to allocate tile_components array\n");
            return;
        }
    }

    if (!map_group) {
        map_group = lv_obj_create(map_container);
        lv_obj_set_size(map_group, MAP_TILES_TILE_SIZE * MAP_TILES_GRID_COLS, MAP_TILES_TILE_SIZE * MAP_TILES_GRID_ROWS);
        lv_obj_set_style_pad_all(map_group, 0, 0);
        lv_obj_set_style_border_width(map_group, 0, 0);
        lv_obj_set_pos(map_group, 0, 0);
        lv_obj_remove_flag(map_group, LV_OBJ_FLAG_SCROLLABLE);
        // presses land on the group (tiles aren't clickable) - pan from here
        // and let the events bubble up to app-level listeners on the container
        lv_obj_add_flag(map_group, LV_OBJ_FLAG_EVENT_BUBBLE);
        lv_obj_add_event_cb(map_group, pan_event_cb, LV_EVENT_PRESSING, NULL);
        
        // Create grid of image widgets for tiles
        for (int i = 0; i < tile_count; i++) {
            tile_components[i] = lv_image_create(map_group);

            int row = i / MAP_TILES_GRID_COLS;
            int col = i % MAP_TILES_GRID_COLS;
            
            lv_obj_set_pos(tile_components[i], col * MAP_TILES_TILE_SIZE, row * MAP_TILES_TILE_SIZE);
            lv_obj_set_size(tile_components[i], MAP_TILES_TILE_SIZE, MAP_TILES_TILE_SIZE);

            // Set default background while tiles load
            lv_obj_set_style_bg_opa(tile_components[i], LV_OPA_COVER, 0);
        }
    }
}

bool GPSLocator::load_tile_images() {
    // queue every cell for background loading, center-out so the visible
    // area fills first; cells already holding the right tile show instantly
    const int cc = MAP_TILES_GRID_COLS / 2;
    const int cr = MAP_TILES_GRID_ROWS / 2;
    for (int dist = 0; dist <= cc + cr; dist++) {
        for (int row = 0; row < MAP_TILES_GRID_ROWS; row++) {
            for (int col = 0; col < MAP_TILES_GRID_COLS; col++) {
                if (abs(row - cr) + abs(col - cc) != dist) continue;
                queue_cell(row * MAP_TILES_GRID_COLS + col);
            }
        }
    }
    MAP_LOG("GPSLocator: Tile loading queued\n");
    return true;
}

bool GPSLocator::read_tile_to_buffer(uint8_t *dst, int tile_x, int tile_y) {
    char path[256];
    snprintf(path, sizeof(path), "%s/%s/%d/%d/%d.bin",
             BASE_PATH, TILE_FOLDER, MAP_ZOOM, tile_x, tile_y);

    FILE *f = fopen(path, "rb");
    if (!f) {
        MAP_LOG("Tile not found: %s\n", path);
        return false;
    }

    // Skip 12-byte header
    fseek(f, 12, SEEK_SET);

    size_t bytes_read = fread(dst, 1, TILE_IMG_SIZE, f);
    fclose(f);

    if (bytes_read != TILE_IMG_SIZE) {
        MAP_LOG("Incomplete tile read: %zu bytes\n", bytes_read);
        memset(dst + bytes_read, 0, TILE_IMG_SIZE - bytes_read);
    }
    return true;
}

static void setup_tile_descriptor(TileSlot &slot) {
    slot.img.header.w = MAP_TILES_TILE_SIZE;
    slot.img.header.h = MAP_TILES_TILE_SIZE;
    slot.img.header.cf = MAP_TILES_COLOR_FORMAT;
    slot.img.header.stride = MAP_TILES_TILE_SIZE * MAP_TILES_BYTES_PER_PIXEL;
    slot.img.data = slot.buf;
    slot.img.data_size = TILE_IMG_SIZE;
}

bool GPSLocator::fetch_images_from_sd(int index, int tile_x, int tile_y) {
    TileSlot& slot = tiles[index];

    if (!slot.buf) {
        slot.buf = (uint8_t*)heap_caps_malloc(TILE_IMG_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!slot.buf) {
            MAP_LOG("Tile %d: allocation failed\n", index);
            return false;
        }
    }

    if (!read_tile_to_buffer(slot.buf, tile_x, tile_y)) {
        return false;
    }

    setup_tile_descriptor(slot);
    return true;
}

void GPSLocator::get_tile_coordinates(double latitude, double longitude, double &tile_x, double &tile_y) {
    tile_x = ((longitude + 180.0) / 360.0 * (1 << MAP_ZOOM));
    double lat_rad = latitude * M_PI / 180.0;
    tile_y = (1.0 - asinh(tan(lat_rad)) / M_PI) / 2.0 * (1 << MAP_ZOOM);
}

void GPSLocator::get_marker_offsets(double &tile_x, double &tile_y, int &offset_x, int &offset_y) {
    offset_x = (int)((tile_x - (int)tile_x) * MAP_TILES_TILE_SIZE);
    offset_y = (int)((tile_y - (int)tile_y) * MAP_TILES_TILE_SIZE);
}

lv_obj_t* GPSLocator::get_container() {
    return map_container;
}

void GPSLocator::get_screen_offset(double latitude, double longitude, int *dx, int *dy) {
    if (!initialized) {
        *dx = 0;
        *dy = 0;
        return;
    }

    double tile_x, tile_y;
    get_tile_coordinates(latitude, longitude, tile_x, tile_y);

    // content px of the position within the loaded grid
    double content_x = (tile_x - top_left_tile_x) * MAP_TILES_TILE_SIZE;
    double content_y = (tile_y - top_left_tile_y) * MAP_TILES_TILE_SIZE;

    // screen px = content px + group position
    double screen_x = content_x + group_pos_x;
    double screen_y = content_y + group_pos_y;

    *dx = (int)(screen_x - map_w / 2);
    *dy = (int)(screen_y - map_h / 2);
}

// Free 2D panning from the raw touch vector (LVGL native scroll is locked
// to one axis per gesture). Infinite browsing: when the view gets close to
// the edge of the loaded 3x3 grid, shift the grid one tile in that
// direction and move the group back so the view stays put.
void GPSLocator::pan_event_cb(lv_event_t *e) {
    if (!initialized) return;

    lv_indev_t *indev = lv_indev_active();
    if (!indev) return;

    lv_point_t v;
    lv_indev_get_vect(indev, &v);
    if (v.x == 0 && v.y == 0) return;

    // The touch controllers (CST816/FT3168) occasionally report one bogus
    // point mid-gesture, producing a huge vector out and an equally huge one
    // back. Applied through the edge clamps / grid shifts below they don't
    // cancel, and the map ends up teleported. A real finger moves well under
    // 100 px between indev reads - drop anything implausible.
    if (v.x > MAX_PAN_STEP_PX || v.x < -MAX_PAN_STEP_PX ||
        v.y > MAX_PAN_STEP_PX || v.y < -MAX_PAN_STEP_PX) {
        PAN_LOG("[pan] GLITCH dropped v=(%d,%d)\n", (int)v.x, (int)v.y);
        return;
    }

    pan_by(v.x, v.y);
}

void GPSLocator::pan_by(int dx, int dy) {
    const int grid_w = MAP_TILES_TILE_SIZE * MAP_TILES_GRID_COLS;
    const int grid_h = MAP_TILES_TILE_SIZE * MAP_TILES_GRID_ROWS;
    // shift one tile before the viewport reaches the grid edge, so loading
    // happens off-screen (tuned for the 5x5 grid)
    const int threshold = MAP_TILES_TILE_SIZE;

    // user takes over: stop any running follow animation
    lv_anim_delete(map_group, NULL);

    int gx = group_pos_x + dx;
    int gy = group_pos_y + dy;

    // keep the viewport inside the loaded grid
    if (gx > 0) gx = 0;
    if (gx < map_w - grid_w) gx = map_w - grid_w;
    if (gy > 0) gy = 0;
    if (gy < map_h - grid_h) gy = map_h - grid_h;

    // shifting is never blocked: incoming cells show the "area locked"
    // placeholder immediately and fill in from the background loader
    int sx = 0, sy = 0;
    if (-gx < threshold)                      sx = -1;
    else if (gx + grid_w - map_w < threshold) sx = 1;
    if (-gy < threshold)                      sy = -1;
    else if (gy + grid_h - map_h < threshold) sy = 1;

    if (sx || sy) {
        shift_grid(sx, sy);
        // same world point sits one tile further in the new grid
        gx += sx * MAP_TILES_TILE_SIZE;
        gy += sy * MAP_TILES_TILE_SIZE;
        lv_obj_invalidate(map_group);
    }

    PAN_LOG("[pan] v=(%d,%d) g=(%d,%d)->(%d,%d) shift=(%d,%d) tl=(%d,%d)\n",
            dx, dy, group_pos_x, group_pos_y,
            gx, gy, sx, sy, top_left_tile_x, top_left_tile_y);

    group_pos_x = gx;
    group_pos_y = gy;
    lv_obj_set_pos(map_group, gx, gy);
}

// ---------------------------------------------------------------- track view
// A finished run drawn as a polyline over the tiles: points are stored in
// world pixels (tile coords * 256 at MAP_ZOOM), split into segments by
// speed bucket, and re-projected into map_group content coordinates on
// every grid shift so the line stays glued to the map.
#define TRACK_DRAW_MAX_PTS 128   // hard cap on drawn nodes
#define TRACK_MIN_NODE_PX  8     // min screen distance between nodes
#define TRACK_MAX_SEGS     32

static double *trk_wx = nullptr, *trk_wy = nullptr;   // world px
static float  *trk_v  = nullptr;                      // km/h
static int     trk_n  = 0;
static lv_obj_t *trk_line[TRACK_MAX_SEGS];
static lv_point_precise_t *trk_seg_pts[TRACK_MAX_SEGS];
static int trk_seg_start[TRACK_MAX_SEGS];
static int trk_seg_len[TRACK_MAX_SEGS];
static int trk_segs = 0;

static int track_bucket(float v_kmh) {
    if (v_kmh < 60.0f)  return 0;
    if (v_kmh < 100.0f) return 1;
    if (v_kmh < 150.0f) return 2;
    return 3;
}

static lv_color_t track_bucket_color(int b) {
    switch (b) {
    case 0:  return lv_color_make(93, 239, 39);    // green: < 60
    case 1:  return lv_color_make(221, 221, 37);   // citrus: 60-100
    case 2:  return lv_color_make(244, 153, 37);   // orange: 100-150
    default: return lv_color_make(255, 42, 22);    // red: 150+
    }
}

// re-project every segment into current grid content coordinates
static void track_reposition(int top_left_x, int top_left_y) {
    for (int s = 0; s < trk_segs; s++) {
        lv_point_precise_t *p = trk_seg_pts[s];
        int start = trk_seg_start[s];
        for (int i = 0; i < trk_seg_len[s]; i++) {
            p[i].x = (lv_value_precise_t)(trk_wx[start + i] - (double)top_left_x * MAP_TILES_TILE_SIZE);
            p[i].y = (lv_value_precise_t)(trk_wy[start + i] - (double)top_left_y * MAP_TILES_TILE_SIZE);
        }
        lv_line_set_points(trk_line[s], p, trk_seg_len[s]);
    }
}

void GPSLocator::track_clear() {
    for (int s = 0; s < trk_segs; s++) {
        lv_obj_delete(trk_line[s]);
        free(trk_seg_pts[s]);
    }
    trk_segs = 0;
    free(trk_wx); free(trk_wy); free(trk_v);
    trk_wx = trk_wy = nullptr;
    trk_v = nullptr;
    trk_n = 0;
}

void GPSLocator::track_show(const track_view_pt_t *pts, int n) {
    if (!initialized || !pts || n < 2) return;
    track_clear();

    // GPS points arrive every ~1px of screen distance at z16 - drawing a
    // node per sample made LVGL do orders of magnitude more work than the
    // eye can see (FPS dropped to ~8). Decimate by screen distance: keep
    // a node only every TRACK_MIN_NODE_PX pixels, always keep the last.
    const double min_d2 = (double)(TRACK_MIN_NODE_PX * TRACK_MIN_NODE_PX);

    trk_wx = (double *)malloc(TRACK_DRAW_MAX_PTS * sizeof(double));
    trk_wy = (double *)malloc(TRACK_DRAW_MAX_PTS * sizeof(double));
    trk_v  = (float *)malloc(TRACK_DRAW_MAX_PTS * sizeof(float));
    if (!trk_wx || !trk_wy || !trk_v) {
        track_clear();
        return;
    }

    int m = 0;
    for (int i = 0; i < n && m < TRACK_DRAW_MAX_PTS; i++) {
        double tx, ty;
        get_tile_coordinates(pts[i].lat, pts[i].lon, tx, ty);
        double wx = tx * MAP_TILES_TILE_SIZE;
        double wy = ty * MAP_TILES_TILE_SIZE;
        if (m > 0 && i < n - 1) {
            double dx = wx - trk_wx[m - 1], dy = wy - trk_wy[m - 1];
            if (dx * dx + dy * dy < min_d2) continue;   // too close to draw
        }
        trk_wx[m] = wx;
        trk_wy[m] = wy;
        trk_v[m] = pts[i].v_kmh;
        m++;
    }
    if (m < 2) {
        track_clear();
        return;
    }
    trk_n = m;

    // split into speed-bucket segments (consecutive segments share their
    // boundary point so the polyline stays visually continuous)
    int s = 0;
    while (s < m - 1 && trk_segs < TRACK_MAX_SEGS) {
        int bucket = track_bucket(trk_v[s]);
        int e = s + 1;
        if (trk_segs == TRACK_MAX_SEGS - 1) {
            e = m - 1;   // segment budget exhausted: rest in one piece
        } else {
            while (e < m - 1 && track_bucket(trk_v[e]) == bucket) e++;
        }
        int len = e - s + 1;
        lv_point_precise_t *p = (lv_point_precise_t *)malloc(len * sizeof(lv_point_precise_t));
        if (!p) break;

        lv_obj_t *line = lv_line_create(map_group);
        lv_obj_set_pos(line, 0, 0);
        lv_obj_set_style_line_width(line, 4, 0);
        // rounded joints draw two arcs per node - measurably slow, and
        // invisible at this width with 8px node spacing
        lv_obj_set_style_line_rounded(line, false, 0);
        lv_obj_set_style_line_color(line, track_bucket_color(bucket), 0);
        lv_obj_set_style_line_opa(line, LV_OPA_90, 0);

        trk_line[trk_segs] = line;
        trk_seg_pts[trk_segs] = p;
        trk_seg_start[trk_segs] = s;
        trk_seg_len[trk_segs] = len;
        trk_segs++;
        s = e;
    }

    track_reposition(top_left_tile_x, top_left_tile_y);
}

void GPSLocator::center_map_on_gps() {
    MAP_LOG("GPSLocator: Centering on GPS coordinates\n");

    // vehicle sits in the center tile of the grid; position the group so
    // that point lands on the viewport center
    group_pos_x = map_w / 2 - MAP_TILES_TILE_SIZE * (MAP_TILES_GRID_COLS / 2) - marker_offset_x;
    group_pos_y = map_h / 2 - MAP_TILES_TILE_SIZE * (MAP_TILES_GRID_ROWS / 2) - marker_offset_y;
    lv_obj_set_pos(map_group, group_pos_x, group_pos_y);
}

static void anim_set_x_cb(void * obj, int32_t v) {
    group_pos_x = v;
    lv_obj_set_x((lv_obj_t *)obj, v);
}

static void anim_set_y_cb(void * obj, int32_t v) {
    group_pos_y = v;
    lv_obj_set_y((lv_obj_t *)obj, v);
}

void GPSLocator::animate_map_center() {
    PAN_LOG("[pan] ANIM recenter off (%d,%d)->(%d,%d) g=(%d,%d)\n",
            marker_offset_x, marker_offset_y, new_marker_offset_x, new_marker_offset_y,
            group_pos_x, group_pos_y);

    const int cx = map_w / 2 - MAP_TILES_TILE_SIZE * (MAP_TILES_GRID_COLS / 2);
    const int cy = map_h / 2 - MAP_TILES_TILE_SIZE * (MAP_TILES_GRID_ROWS / 2);

    // X animation
    lv_anim_t ax;
    lv_anim_init(&ax);
    lv_anim_set_var(&ax, map_group);
    lv_anim_set_time(&ax, STEP_ANIMATION_DURATION);
    lv_anim_set_exec_cb(&ax, anim_set_x_cb);
    lv_anim_set_values(&ax, cx - marker_offset_x, cx - new_marker_offset_x);
    lv_anim_start(&ax);

    // Y animation
    lv_anim_t ay;
    lv_anim_init(&ay);
    lv_anim_set_var(&ay, map_group);
    lv_anim_set_time(&ay, STEP_ANIMATION_DURATION);
    lv_anim_set_exec_cb(&ay, anim_set_y_cb);
    lv_anim_set_values(&ay, cy - marker_offset_y, cy - new_marker_offset_y);
    lv_anim_start(&ay);

    marker_offset_x = new_marker_offset_x;
    marker_offset_y = new_marker_offset_y;
}

void GPSLocator::show_initial_location(double latitude, double longitude) {
    if (!initialized) return;

    PAN_LOG("[pan] REBUILD grid at %.6f,%.6f\n", latitude, longitude);

    double tile_x, tile_y;
    get_tile_coordinates(latitude, longitude, tile_x, tile_y);
    get_marker_offsets(tile_x, tile_y, marker_offset_x, marker_offset_y);

    // store integer tile coordinates
    // shift to top-left tile of grid
    top_left_tile_x = (int)tile_x - (int)(MAP_TILES_GRID_COLS / 2);
    top_left_tile_y = (int)tile_y - (int)(MAP_TILES_GRID_ROWS / 2);
    grid_gen++;   // invalidate any in-flight background loads

    // Load tile images and move map to center
    load_tile_images();
    center_map_on_gps();
    track_reposition(top_left_tile_x, top_left_tile_y);
}

void GPSLocator::set_fallback_tile(int col) {
   tiles[col].img = area_locked_tile;          // Copy descriptor
   tiles[col].buf = nullptr;                   // No writable buffer
   lv_image_set_src(GPSLocator::tile_components[col], &area_locked_tile);
}

void GPSLocator::shift_slots_x(int dir) {
    // dir=+1: view moved east - contents shift left, rightmost column reloads
    // dir=-1: view moved west - contents shift right, leftmost column reloads
    for (int row = 0; row < MAP_TILES_GRID_ROWS; row++) {
        if (dir > 0) {
            for (int col = 0; col < MAP_TILES_GRID_COLS - 1; col++)
                std::swap(tiles[row * MAP_TILES_GRID_COLS + col], tiles[row * MAP_TILES_GRID_COLS + col + 1]);
        } else {
            for (int col = MAP_TILES_GRID_COLS - 1; col > 0; col--)
                std::swap(tiles[row * MAP_TILES_GRID_COLS + col], tiles[row * MAP_TILES_GRID_COLS + col - 1]);
        }
    }
}

void GPSLocator::shift_slots_y(int dir) {
    // dir=+1: view moved south - rows shift up, bottom row reloads
    // dir=-1: view moved north - rows shift down, top row reloads
    if (dir > 0) {
        for (int row = 0; row < MAP_TILES_GRID_ROWS - 1; row++)
            for (int col = 0; col < MAP_TILES_GRID_COLS; col++)
                std::swap(tiles[row * MAP_TILES_GRID_COLS + col], tiles[(row + 1) * MAP_TILES_GRID_COLS + col]);
    } else {
        for (int row = MAP_TILES_GRID_ROWS - 1; row > 0; row--)
            for (int col = 0; col < MAP_TILES_GRID_COLS; col++)
                std::swap(tiles[row * MAP_TILES_GRID_COLS + col], tiles[(row - 1) * MAP_TILES_GRID_COLS + col]);
    }
}

// Persistent worker: reads one tile at a time into the scratch buffer and
// hands it to the LVGL thread, which copies it into the cell if - and only
// if - that cell still wants those coordinates (the grid may have shifted
// again while the SD read was running).
void GPSLocator::tile_loader_task(void *param) {
    TileJob job;
    for (;;) {
        xQueueReceive(tile_load_queue, &job, portMAX_DELAY);
        if (job.gen != grid_gen) continue;   // stale before we even started

        xSemaphoreTake(scratch_free, portMAX_DELAY);
        bool ok = read_tile_to_buffer(scratch_buf, job.tx, job.ty);

        TileApply *apply = new TileApply{job, ok};
        lv_async_call([](void *p) {
            TileApply *a = static_cast<TileApply *>(p);
            int row = a->job.index / MAP_TILES_GRID_COLS;
            int col = a->job.index % MAP_TILES_GRID_COLS;
            bool still_wanted = a->job.tx == top_left_tile_x + col &&
                                a->job.ty == top_left_tile_y + row;
            if (still_wanted && a->ok) {
                TileSlot &slot = tiles[a->job.index];
                if (!slot.buf) {
                    slot.buf = (uint8_t*)heap_caps_malloc(TILE_IMG_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                }
                if (slot.buf) {
                    memcpy(slot.buf, scratch_buf, TILE_IMG_SIZE);
                    setup_tile_descriptor(slot);
                    slot.tx = a->job.tx;
                    slot.ty = a->job.ty;
                    slot.valid = true;
                    lv_image_set_src(tile_components[a->job.index], &slot.img);
                    lv_obj_invalidate(tile_components[a->job.index]);
                    PAN_LOG("[tile] load cell %d = %d,%d\n", a->job.index, a->job.tx, a->job.ty);
                }
            } else {
                PAN_LOG("[tile] drop cell %d = %d,%d (wanted %d,%d ok=%d)\n",
                        a->job.index, a->job.tx, a->job.ty,
                        top_left_tile_x + col, top_left_tile_y + row, (int)a->ok);
            }
            xSemaphoreGive(scratch_free);
            delete a;
        }, apply);
    }
}

// show a cell's tile: instantly if the slot already holds it, otherwise
// placeholder + queue for background loading
void GPSLocator::queue_cell(int index) {
    int row = index / MAP_TILES_GRID_COLS;
    int col = index % MAP_TILES_GRID_COLS;
    int want_tx = top_left_tile_x + col;
    int want_ty = top_left_tile_y + row;

    TileSlot &slot = tiles[index];
    if (slot.valid && slot.tx == want_tx && slot.ty == want_ty) {
        // already holds the right tile (e.g. panned back) - no blink, no SD
        lv_image_set_src(tile_components[index], &slot.img);
        PAN_LOG("[tile] cache cell %d = %d,%d\n", index, want_tx, want_ty);
        return;
    }

    slot.valid = false;
    lv_image_set_src(tile_components[index], &area_locked_tile);

    TileJob job = {index, want_tx, want_ty, grid_gen};
    if (xQueueSend(tile_load_queue, &job, 0) != pdTRUE) {
        MAP_LOG("GPSLocator: tile load queue full, cell %d stays locked\n", index);
    }
}

void GPSLocator::shift_grid(int sx, int sy) {
    if (!sx && !sy) return;

    if (sx) shift_slots_x(sx);
    if (sy) shift_slots_y(sy);
    top_left_tile_x += sx;
    top_left_tile_y += sy;
    // the group anchor is tied to the (moved) grid origin
    marker_offset_x -= sx * MAP_TILES_TILE_SIZE;
    marker_offset_y -= sy * MAP_TILES_TILE_SIZE;
    grid_gen++;

    // re-queue every cell that doesn't hold its desired tile (this also
    // re-issues any loads dropped by the generation bump); cells that
    // already hold the right tile are shown instantly and skipped
    load_tile_images();

    // the track polyline is in content coordinates - follow the grid
    track_reposition(top_left_tile_x, top_left_tile_y);
}

void GPSLocator::move_location(double latitude, double longitude) {
    if (!initialized) return;

    double new_tile_x, new_tile_y;
    get_tile_coordinates(latitude, longitude, new_tile_x, new_tile_y);

    int sx = ((int)new_tile_x - MAP_TILES_GRID_COLS / 2) - top_left_tile_x;
    int sy = ((int)new_tile_y - MAP_TILES_GRID_ROWS / 2) - top_left_tile_y;

    if (sx < -1 || sx > 1 || sy < -1 || sy > 1) {
        // position jumped more than one tile - rebuild the whole grid
        show_initial_location(latitude, longitude);
        return;
    }

    if (sx || sy) {
        shift_grid(sx, sy);
    }
    // offsets relative to the current grid center tile
    new_marker_offset_x = (int)((new_tile_x - (top_left_tile_x + MAP_TILES_GRID_COLS / 2)) * MAP_TILES_TILE_SIZE);
    new_marker_offset_y = (int)((new_tile_y - (top_left_tile_y + MAP_TILES_GRID_ROWS / 2)) * MAP_TILES_TILE_SIZE);

    animate_map_center();
}
