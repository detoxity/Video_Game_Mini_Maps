// =============================================================================
// Video game style GPS mini map
//
// Code map (start with app_config.h - all feature toggles live there):
//   main/app_config.h    feature toggles and tunables
//   main/app_state.h     shared state (defined here) and color palettes
//   main/ui_overlays.*   speed / battery+sats / perf overlays, BOOT button,
//                        record mode, run history screen
//   main/persistence.*   NVS position save/load, PMU power-off sequence
//   main/demo_gps.*      drag-strip simulator (GPS_SOURCE_DEMO)
//   components/gps_locator       map tiles, panning, background loading
//   components/GPS_UART_Driver   u-blox UBX/NMEA input, auto-baud/config
//   components/PerfMeter         0-60/0-100/100-200/402m timing, IMU fusion
//   components/QMI8658_IMU       on-board accelerometer (launch detection)
//   components/QMC5883L_Compass  external magnetometer (heading at standstill)
//   components/AXP2101_PMU       battery/power management (AMOLED 1.8)
//   components/CANBus_Driver     legacy CAN GPS input (GPS_SOURCE_CAN)
//   boards/                      per-board BSPs (selected by BOARD env)
// =============================================================================
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "driver/twai.h"

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <cmath>

#include "app_config.h"
#include "app_state.h"
#include "ui_overlays.h"
#include "persistence.h"
#include "demo_gps.h"
#include "TrackLog.h"
#include "PerfMeter.h"

#include "CANBus_Driver.h"
#include "GPS_UART_Driver.h"
#include "gps_locator.h"

#if USE_COMPASS
#include "QMC5883L_Compass.h"
#endif
#if USE_IMU
#include "QMI8658_IMU.h"
#endif
#if USE_PMU
#include "AXP2101_PMU.h"
#endif
#if RACEBOX_BLE
#include "RaceBoxBLE.h"
#endif

#include "images/car_icon.h"
#include "images/north_pointer.h"
#include "images/no_satellite.h"

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)
#define RAD_TO_DEG(rad) ((rad) * 180.0 / M_PI)

LV_IMG_DECLARE(car_icon);
LV_IMG_DECLARE(north_pointer);
LV_IMG_DECLARE(no_satellite);

// ---------------------------------------------------------------- state
// (declared in app_state.h, shared with the other main/ modules)
lv_obj_t *main_scr;
lv_obj_t *no_satellite_bg;
lv_obj_t *map_container;
lv_obj_t *car_icon_img;
lv_obj_t *north_pointer_img;

float current_latitude        = 0.0;
float current_longitude       = 0.0;
float new_latitude            = 0.0;
float new_longitude           = 0.0;
int current_angle             = 0;
int new_angle                 = 0;
float gps_speed_kmh           = 0.0f;  // written by the GPS driver / demo
int gps_sat_count             = 0;     // satellites in use
bool receiving_data           = false; // has the first data been received
volatile bool data_ready      = false; // new incoming data
bool location_initialized     = false; // has the initial GPS location been set

bool follow_vehicle           = true;  // false while the user browses the map
uint32_t last_touch_tick      = 0;
bool track_view_active        = false; // reviewing a saved run's track

void (*can_message_handler)(twai_message_t *message) = NULL;

// general color palettes
const lv_color_t PALETTE_BLACK        = LV_COLOR_MAKE(0, 0, 0);
const lv_color_t PALETTE_BLUE         = LV_COLOR_MAKE(31, 104, 135); // fuel arc main
const lv_color_t PALETTE_BLUE_NEON    = LV_COLOR_MAKE(83, 252, 254); // fuel arc indicator
const lv_color_t PALETTE_DARK_GREY    = LV_COLOR_MAKE(24, 24, 24); // highlight button background
const lv_color_t PALETTE_RED          = LV_COLOR_MAKE(130, 35, 53); // redline
const lv_color_t PALETTE_GREEN        = LV_COLOR_MAKE(123, 207, 21); // buttons and text
const lv_color_t PALETTE_GREY         = LV_COLOR_MAKE(120, 120, 120); // button background
const lv_color_t PALETTE_WHITE        = LV_COLOR_MAKE(255, 255, 255);

// NFSU2 pickable colors
const lv_color_t PALETTE_NFS_WHITE    = LV_COLOR_MAKE(255, 255, 255);
const lv_color_t PALETTE_NFS_BLUE     = LV_COLOR_MAKE(52, 154, 227);
const lv_color_t PALETTE_NFS_CYAN     = LV_COLOR_MAKE(34, 199, 239);
const lv_color_t PALETTE_NFS_GREEN    = LV_COLOR_MAKE(93, 239, 39);
const lv_color_t PALETTE_NFS_CITRUS   = LV_COLOR_MAKE(221, 221, 37);
const lv_color_t PALETTE_NFS_LIME     = LV_COLOR_MAKE(148, 248, 38);
const lv_color_t PALETTE_NFS_ORANGE   = LV_COLOR_MAKE(244, 153, 37);
const lv_color_t PALETTE_NFS_RED      = LV_COLOR_MAKE(255, 42, 22);
const lv_color_t PALETTE_NFS_PURPLE   = LV_COLOR_MAKE(136, 86, 255);
const lv_color_t PALETTE_NFS_GREY     = LV_COLOR_MAKE(175, 181, 191);
const lv_color_t PALETTE_NFS_BLUE2    = LV_COLOR_MAKE(27, 173, 252);
const lv_color_t PALETTE_NFS_YELLOW   = LV_COLOR_MAKE(229, 223, 33);

// ---------------------------------------------------------------- geo math
// get bearing angle between two coordinates (LVGL tenths of a degree)
double angle_from_coordinate(double lat1, double long1, double lat2, double long2) {
    double lat1_rad = DEG_TO_RAD(lat1);
    double lat2_rad = DEG_TO_RAD(lat2);
    double dlon_rad = DEG_TO_RAD(long2 - long1);

    double y = sin(dlon_rad) * cos(lat2_rad);
    double x = cos(lat1_rad)*sin(lat2_rad) - sin(lat1_rad)*cos(lat2_rad)*cos(dlon_rad);
    double bearing_rad = atan2(y, x);

    double bearing_deg = fmod(RAD_TO_DEG(bearing_rad) + 360.0, 360.0);

    return (int)round(bearing_deg * 10.0);
}

// calculate distance between two coordinates in meters
double distance_between(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0; // Earth radius in meters
    double dLat = DEG_TO_RAD(lat2 - lat1);
    double dLon = DEG_TO_RAD(lon2 - lon1);
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(DEG_TO_RAD(lat1)) * cos(DEG_TO_RAD(lat2)) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

// normalize angle to shortest rotation direction
static float normalize_angle(float from, float to) {
    float diff = to - from;
    while (diff > 1800) diff -= 3600;
    while (diff < -1800) diff += 3600;
    return from + diff;
}

static void anim_set_r_cb(void * obj, int32_t v) {
    lv_img_set_angle((lv_obj_t *)obj, v);
}

// ---------------------------------------------------------------- vehicle
// keep the car icon glued to its real map position while browsing;
// hide it only when it drifts off the visible area
void update_vehicle_marker(void) {
    int dx, dy;
    GPSLocator::get_screen_offset(current_latitude, current_longitude, &dx, &dy);
    bool visible = (dx > -BSP_LCD_H_RES / 2 && dx < BSP_LCD_H_RES / 2 &&
                    dy > -BSP_LCD_V_RES / 2 && dy < BSP_LCD_V_RES / 2);
    lv_obj_set_style_opa(car_icon_img, visible ? LV_OPA_COVER : LV_OPA_0, 0);
    lv_obj_align(car_icon_img, LV_ALIGN_CENTER, dx, 5 + dy);
}

// a new position arrived: move the map / rotate the icon
static void update_values(void) {
    if (!follow_vehicle) {
        // user is browsing the map: keep tracking position, don't move the view
        current_latitude = new_latitude;
        current_longitude = new_longitude;
        update_vehicle_marker();
        return;
    }

    if (location_initialized) {
        GPSLocator::move_location((double)new_latitude, (double)new_longitude);
    } else {
        GPSLocator::show_initial_location((double)new_latitude, (double)new_longitude);
        location_initialized = true;
    }
    save_last_position();

    lv_obj_set_style_opa(no_satellite_bg, LV_OPA_0, 0);
    lv_obj_set_style_opa(map_container, LV_OPA_COVER, 0);
    lv_obj_set_style_opa(car_icon_img, LV_OPA_COVER, 0);

    double dist = distance_between(current_latitude, current_longitude, new_latitude, new_longitude);

    int rotate_target = -1;
    if (dist > MIN_MOVE_DISTANCE) {
        // moving: heading from GPS track
        new_angle = angle_from_coordinate(current_latitude, current_longitude, new_latitude, new_longitude);
        rotate_target = new_angle;
    }
#if USE_COMPASS
    else if (compass_available()) {
        // standing still: heading from the magnetometer
        int compass_angle = (int)(compass_heading() * 10.0f);
        int diff = compass_angle - current_angle;
        while (diff > 1800) diff -= 3600;
        while (diff < -1800) diff += 3600;
        if (abs(diff) > 30) {   // ignore jitter under 3 degrees
            rotate_target = compass_angle;
        }
    }
#endif

    if (rotate_target >= 0) {
        float anim_target_angle = normalize_angle(current_angle, rotate_target);

        lv_anim_t aa;
        lv_anim_init(&aa);
        lv_anim_set_var(&aa, car_icon_img);
        lv_anim_set_time(&aa, STEP_ANIMATION_DURATION);
        lv_anim_set_exec_cb(&aa, anim_set_r_cb);
        lv_anim_set_values(&aa, current_angle, anim_target_angle);
        lv_anim_start(&aa);

        current_angle = anim_target_angle;
    }

    current_latitude = new_latitude;
    current_longitude = new_longitude;
}

// ---------------------------------------------------------------- screen
static void make_screen(void) {
    main_scr = lv_obj_create(NULL);
    lv_obj_set_size(main_scr, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_set_style_bg_color(main_scr, PALETTE_BLACK, 0);
    lv_obj_set_style_pad_all(main_scr, 0, 0);
    lv_obj_set_style_border_width(main_scr, 0, 0);
    lv_obj_remove_flag(main_scr, LV_OBJ_FLAG_SCROLLABLE);

    no_satellite_bg = lv_img_create(main_scr);
    lv_image_set_src(no_satellite_bg, &no_satellite);
    lv_obj_align(no_satellite_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_opa(no_satellite_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(no_satellite_bg, PALETTE_BLACK, 0);
    lv_obj_set_style_pad_all(no_satellite_bg, 0, 0);
    lv_obj_set_style_border_width(no_satellite_bg, 0, 0);

    map_container = lv_obj_create(main_scr);
    lv_obj_set_size(map_container, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_align(map_container, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_opa(map_container, LV_OPA_0, 0);
    lv_obj_set_style_bg_color(map_container, PALETTE_BLACK, 0);
    lv_obj_set_style_pad_all(map_container, 0, 0);
    lv_obj_set_style_border_width(map_container, 0, 0);
    lv_obj_set_style_outline_color(map_container, PALETTE_GREY, 0);
    lv_obj_remove_flag(map_container, LV_OBJ_FLAG_SCROLLABLE);

#if SHOW_RING && defined(BSP_BOARD_WS_S3_TOUCH_LCD_1_85)
    // gold edge ring - drawn as a vector circle border instead of a
    // full-screen alpha-blended image (the image cost ~1/3 of the frame
    // budget: LVGL blends all 360x360 pixels incl. the transparent hole)
    lv_obj_t *ring = lv_obj_create(main_scr);
    lv_obj_set_size(ring, BSP_LCD_H_RES - 2, BSP_LCD_V_RES - 2);
    lv_obj_center(ring);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(ring, LV_OPA_0, 0);
    lv_obj_set_style_border_width(ring, 8, 0);
    lv_obj_set_style_border_color(ring, lv_color_make(160, 104, 40), 0);   // north-pointer gold
    lv_obj_remove_flag(ring, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(ring, LV_OBJ_FLAG_CLICKABLE);
#endif
}

static void make_vehicle_ui(void) {
    car_icon_img = lv_img_create(main_scr);
    lv_image_set_src(car_icon_img, &car_icon);
    lv_obj_set_style_opa(car_icon_img, LV_OPA_0, 0);
    lv_obj_align(car_icon_img, LV_ALIGN_CENTER, 0, 5);
    lv_obj_set_style_transform_pivot_x(car_icon_img, 24, 0);
    lv_obj_set_style_transform_pivot_y(car_icon_img, 21, 0);

    north_pointer_img = lv_img_create(main_scr);
    lv_image_set_src(north_pointer_img, &north_pointer);
    // orbit radius from screen center (image is 43x30, pre-scaled)
    const int north_orbit = BSP_LCD_V_RES / 2 - 45;
    lv_obj_align(north_pointer_img, LV_ALIGN_CENTER, 0, -north_orbit);
    lv_obj_set_style_transform_pivot_x(north_pointer_img, 21, 0);
    lv_obj_set_style_transform_pivot_y(north_pointer_img, (north_orbit + 15), 0);
}

// ---------------------------------------------------------------- CAN input
static void update_location(uint8_t* data) {
    new_latitude = 0.0f;
    new_longitude = 0.0f;
    memcpy(&new_latitude, data, sizeof(float));
    memcpy(&new_longitude, data + 4, sizeof(float));
}

// The GPS module sends messages with ID 0x430 containing lat/lon as floats
static void custom_can_message_handler(twai_message_t *message) {
    switch (message->identifier) {
        case 0x430:
            update_location(message->data);
            data_ready = true;
            break;
        default:
            break;
    }
    receiving_data = true;
}

// ---------------------------------------------------------------- glue
static void mount_sd(void) {
    esp_err_t err = bsp_sdcard_mount();
    if (err != ESP_OK) {
        printf("Failed to mount SD card, error: %s\n", esp_err_to_name(err));
    }
}

// any drag on the map switches to browse mode; the map component itself
// handles the panning and infinite tile loading (see GPSLocator::pan_by)
static void map_touch_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    last_touch_tick = lv_tick_get();

    if (code == LV_EVENT_PRESSING && location_initialized) {
        lv_indev_t *indev = lv_indev_active();
        if (!indev) return;
        lv_point_t v;
        lv_indev_get_vect(indev, &v);
        if (v.x == 0 && v.y == 0) return;

        follow_vehicle = false;
        update_vehicle_marker();
    }
}

static void return_to_vehicle(void) {
    follow_vehicle = true;
    lv_obj_set_style_opa(car_icon_img, LV_OPA_COVER, 0);
    lv_obj_align(car_icon_img, LV_ALIGN_CENTER, 0, 5);
    if (location_initialized) {
        // full recenter: resets panning and reloads the grid around the vehicle
        GPSLocator::show_initial_location(current_latitude, current_longitude);
    }
}

#if AUTO_ROTATE && USE_IMU
// flip the screen 180deg (hardware mirror) when gravity says the device
// is mounted upside down; sustained + hysteresis so driving dynamics
// can't trigger it. Self-disables if the panel has no hardware mirror.
static void auto_rotate_tick(void) {
    static uint32_t t = 0;
    static int sustain = 0;
    static bool flipped = false;
    static bool supported = true;

    if (!supported || lv_tick_elaps(t) < 200) return;
    t = lv_tick_get();

    float a[3];
    if (!imu_get_accel(&a[0], &a[1], &a[2])) return;

    // diagnostics (compiled out in silent builds)
    static uint32_t dbg_t = 0;
    if (lv_tick_elaps(dbg_t) > 2000) {
        dbg_t = lv_tick_get();
        ESP_LOGI("rotate", "accel x=%.1f y=%.1f z=%.1f", a[0], a[1], a[2]);
    }

    float v = a[AUTO_ROTATE_AXIS] * (AUTO_ROTATE_INVERT ? -1.0f : 1.0f);
    bool want;
    if (v > 7.0f)       want = true;    // clearly upside down
    else if (v < -7.0f) want = false;   // clearly upright
    else {                              // tilted/flat: keep current
        sustain = 0;
        return;
    }
    if (want == flipped) {
        sustain = 0;
        return;
    }
    if (++sustain < 10) return;         // ~2s sustained before acting
    sustain = 0;

    esp_err_t err = bsp_display_set_flip(want);
    ESP_LOGI("rotate", "flip(%d) -> %s", want, esp_err_to_name(err));
    if (err == ESP_OK) {
        flipped = want;
        lv_obj_invalidate(lv_screen_active());
    } else {
        supported = false;   // no hardware rotation - feature off for good
    }
}
#endif

static void lvgl_timer(lv_timer_t * timer) {
    if (data_ready) {
        data_ready = false;
        update_values();
    }

    if (!follow_vehicle && !track_view_active &&
        lv_tick_elaps(last_touch_tick) > PAN_RETURN_TIMEOUT_MS) {
        return_to_vehicle();
    }

#if AUTO_ROTATE && USE_IMU
    auto_rotate_tick();
#endif

    overlays_tick();
}

// ---------------------------------------------------------------- app_main
extern "C" void app_main(void) {
    // grab the power latch first thing (LCD 1.85: hold GPIO7 high so the
    // board stays on once the power button is released). No-op on AMOLED.
#if USE_PMU
    pmu_power_hold_early();
#endif

    persistence_init();   // NVS + last known position into current/new lat/lon
    mount_sd();

#ifdef BSP_BOARD_WS_S3_TOUCH_LCD_1_85
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,   // S3: DMA draw buffer must live in internal RAM
            .sw_rotate = false,
        }
    };
#else
    // Waveshare registry BSP (AMOLED 1.8): no sw_rotate flag, buffers via Kconfig
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
        }
    };
#endif
    // core layout: 0 = UI world (LVGL render, tile streaming), 1 =
    // measurement world (GPS + IMU tasks) - rendering bursts can never
    // add jitter to the perf-meter sample stream
    cfg.lvgl_port_cfg.task_affinity = 0;
    bsp_display_start_with_config(&cfg);
    // panel RAM is cleared to black inside the BSP, so lighting up now is safe
    bsp_display_backlight_on();
    bsp_display_brightness_set(100);

    // peripherals on the shared I2C bus (each degrades gracefully if absent)
#if USE_COMPASS
    compass_start(BSP_I2C_NUM);
#endif
    persistence_pmu_init();
#if USE_IMU
    imu_start(BSP_I2C_NUM);
#endif

    // perf-meter timing calibration + launch rollout (compile-time defaults;
    // keeps PerfMeter free of any app dependency)
    perf_set_calibration_offset(PERF_CALIBRATION_OFFSET_MS);
    perf_set_rollout_mm(PERF_ROLLOUT_MM);

    // track recorder buffer (fed by the GPS/demo task during perf runs)
    tracklog_init();

#if RACEBOX_BLE
    // advertise as a RaceBox Mini; the GPS driver publishes each NAV-PVT
    racebox_ble_start(RACEBOX_BLE_SERIAL);
#if RACEBOX_BLE_RECORD_ONLY
    racebox_ble_set_streaming(false);   // opened by record mode
#endif
#endif

    // position input
#if GPS_SOURCE == GPS_SOURCE_DEMO
    demo_gps_start();
#elif GPS_SOURCE == GPS_SOURCE_UART
    gps_uart_start();
#else
    can_message_handler = custom_can_message_handler;
    canbus_init();
    start_can_tasks();
#endif

    // build the UI
    bsp_display_lock(0);

    make_screen();

    if (!GPSLocator::init(map_container, BSP_LCD_H_RES, BSP_LCD_V_RES)) {
        printf("Failed to initialize map\n");
        return;
    }
    lv_obj_t *scroll_area = GPSLocator::get_container();
    lv_obj_add_event_cb(scroll_area, map_touch_event_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(scroll_area, map_touch_event_cb, LV_EVENT_RELEASED, NULL);

    make_vehicle_ui();
    overlays_create(main_scr);

    // boot straight into the map at the last known position (navigator
    // style) - GPS jumps it to the live position once a good fix arrives
    GPSLocator::show_initial_location((double)current_latitude, (double)current_longitude);
    location_initialized = true;
    lv_obj_set_style_opa(no_satellite_bg, LV_OPA_0, 0);
    lv_obj_set_style_opa(map_container, LV_OPA_COVER, 0);
    lv_obj_set_style_opa(car_icon_img, LV_OPA_COVER, 0);

    lv_screen_load(main_scr);

    bsp_display_unlock();

    lv_timer_create(lvgl_timer, 10, NULL);
}
