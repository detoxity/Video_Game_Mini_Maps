#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "nvs.h"

#include "app_config.h"
#include "app_state.h"
#include "ui_overlays.h"
#include "bsp/esp-bsp.h"
#include "GPS_UART_Driver.h"
#include "TrackLog.h"
#include "gps_locator.h"
#include "esp_heap_caps.h"

#if SHOW_PERF
#include "PerfMeter.h"
#endif
#if USE_PMU
#include "AXP2101_PMU.h"
#endif

// ---------------------------------------------------------------- widgets
#if SHOW_SPEED
static lv_obj_t *speed_label = NULL;
#endif
#if SHOW_BATTERY
static lv_obj_t *batt_label = NULL;
static bool batt_visible = true;
#endif
#if SHOW_PERF
static lv_obj_t *perf_label = NULL;
static lv_obj_t *rec_dot = NULL;
static bool record_mode = false;
static lv_obj_t *hist_panel = NULL;
static lv_obj_t *hist_list = NULL;
#endif

bool overlays_battery_visible(void) {
#if SHOW_BATTERY
    return batt_visible;
#else
    return true;
#endif
}

#if SHOW_BATTERY
// battery label and LVGL's FPS meter show/hide together (BOOT click)
static void batt_apply_visibility(void) {
    if (batt_visible) {
        lv_obj_remove_flag(batt_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(batt_label, LV_OBJ_FLAG_HIDDEN);
    }
#if LV_USE_PERF_MONITOR
    // LVGL auto-creates the FPS meter on the sys layer at display creation.
    // Toggle that label's hidden flag directly: lv_sysmon_show_performance()
    // would re-create it and leak a timer + observer per call.
    static lv_obj_t *fps_label = NULL;
    if (!fps_label) {
        fps_label = lv_obj_get_child(lv_layer_sys(), 0);
    }
    if (fps_label) {
        if (batt_visible) {
            lv_obj_remove_flag(fps_label, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(fps_label, LV_OBJ_FLAG_HIDDEN);
        }
    }
#endif
}
#endif

// ---------------------------------------------------------------- history
#if SHOW_PERF
struct PerfRecord {
    float t60, t100, t100_200, t402, v402;
    // local date/time of the run from GPS UTC (0 = unknown, e.g. demo mode)
    uint16_t year;
    uint8_t  mon, day, hour, min;
    // GPS stream time lost and bridged mid-run (0 = clean measurement)
    float gap_s;
    // id of the CSV track on SD (/sdcard/tracks/run_NNNNN.csv), 0 = none
    uint32_t track_id;
    // course slope over the run, negative = downhill (Dragy-style)
    float slope_pct;
    // 1 = launch instant came from the IMU, 0 = from GNSS speed
    uint8_t launch_imu;
};
static PerfRecord history[HISTORY_MAX];
static int history_count = 0;

static void history_save(void) {
    nvs_handle_t h;
    if (nvs_open("minimap", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "runs3", history, history_count * sizeof(PerfRecord));
        nvs_commit(h);
        nvs_close(h);
    }
}

static void history_load(void) {
    // key bumped to runs3 when the record grew (launch_imu field); older
    // on-device history does not carry over - the size check rejects it
    nvs_handle_t h;
    if (nvs_open("minimap", NVS_READONLY, &h) != ESP_OK) return;
    size_t len = sizeof(history);
    if (nvs_get_blob(h, "runs3", history, &len) == ESP_OK &&
        len % sizeof(PerfRecord) == 0) {
        history_count = len / sizeof(PerfRecord);
    }
    nvs_close(h);
}

// stamp a record with local time derived from the GPS UTC clock
static void stamp_local_time(PerfRecord *r) {
    r->year = 0; r->mon = 0; r->day = 0; r->hour = 0; r->min = 0;
#if GPS_SOURCE == GPS_SOURCE_UART
    if (!gps_time_valid) return;
    int y = gps_utc_year, mo = gps_utc_month, d = gps_utc_day;
    int h = gps_utc_hour + UTC_OFFSET_HOURS, mi = gps_utc_min;
    static const uint8_t dim[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (mo < 1 || mo > 12) return;
    if (h >= 24) {
        h -= 24;
        int days = dim[mo - 1] + ((mo == 2 && y % 4 == 0) ? 1 : 0);
        if (++d > days) { d = 1; if (++mo > 12) { mo = 1; y++; } }
    } else if (h < 0) {
        h += 24;
        if (--d < 1) {
            if (--mo < 1) { mo = 12; y--; }
            d = dim[mo - 1] + ((mo == 2 && y % 4 == 0) ? 1 : 0);
        }
    }
    r->year = y; r->mon = mo; r->day = d; r->hour = h; r->min = mi;
#endif
}

static void history_refresh_list(void);
static void history_toggle(void);

static void history_add(const perf_results_t *r) {
    if (history_count >= HISTORY_MAX) {
        // drop the oldest
        memmove(&history[0], &history[1], (HISTORY_MAX - 1) * sizeof(PerfRecord));
        history_count = HISTORY_MAX - 1;
    }
    PerfRecord rec = {r->t_0_60, r->t_0_100, r->t_100_200, r->t_402m, r->v_402m_kmh,
                      0, 0, 0, 0, 0, r->gap_s, 0, r->slope_pct,
                      (uint8_t)(r->launch_imu ? 1 : 0)};
    stamp_local_time(&rec);
    rec.track_id = tracklog_save_csv();   // GPS trace of the run -> SD card
    history[history_count++] = rec;
    history_save();
}

static void history_delete(int index) {
    if (index < 0 || index >= history_count) return;
    tracklog_delete_csv(history[index].track_id);
    memmove(&history[index], &history[index + 1], (history_count - 1 - index) * sizeof(PerfRecord));
    history_count--;
    history_save();
    history_refresh_list();
}

static void track_view_open(int index);

// swipe an entry to the left to delete it; tap it to view the run's
// track drawn over the map
static void hist_item_event_cb(lv_event_t *e) {
    static int32_t swipe_x = 0;
    static bool consumed = false;

    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        swipe_x = 0;
        consumed = false;
        return;
    }
    if (code == LV_EVENT_CLICKED) {
        if (!consumed && swipe_x > -20 && swipe_x < 20) {
            track_view_open((int)(intptr_t)lv_event_get_user_data(e));
        }
        return;
    }
    if (code != LV_EVENT_PRESSING || consumed) return;

    lv_indev_t *indev = lv_indev_active();
    if (!indev) return;
    lv_point_t v;
    lv_indev_get_vect(indev, &v);
    swipe_x += v.x;

    if (swipe_x < -60) {
        consumed = true;
        history_delete((int)(intptr_t)lv_event_get_user_data(e));
    }
}

static void fmt_time(char *dst, size_t n, float t) {
    if (t >= 0) snprintf(dst, n, "%.2fs", t);
    else        snprintf(dst, n, "-");
}

static void history_refresh_list(void) {
    lv_obj_clean(hist_list);

    if (history_count == 0) {
        lv_obj_t *lbl = lv_label_create(hist_list);
        lv_obj_set_style_text_color(lbl, PALETTE_GREY, 0);
        lv_label_set_text(lbl, "no runs yet");
        return;
    }

    // newest first
    for (int i = history_count - 1; i >= 0; i--) {
        PerfRecord *r = &history[i];
        lv_obj_t *item = lv_obj_create(hist_list);
        lv_obj_set_width(item, lv_pct(100));
        lv_obj_set_height(item, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(item, PALETTE_DARK_GREY, 0);
        lv_obj_set_style_bg_opa(item, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(item, 0, 0);
        lv_obj_set_style_radius(item, 6, 0);
        lv_obj_set_style_pad_all(item, 8, 0);
        lv_obj_set_style_pad_row(item, 3, 0);
        lv_obj_set_flex_flow(item, LV_FLEX_FLOW_COLUMN);
        lv_obj_remove_flag(item, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(item, hist_item_event_cb, LV_EVENT_PRESSED, (void *)(intptr_t)i);
        lv_obj_add_event_cb(item, hist_item_event_cb, LV_EVENT_PRESSING, (void *)(intptr_t)i);
        lv_obj_add_event_cb(item, hist_item_event_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);

        char t60[16], t100[16], t12[16], t402[32];
        fmt_time(t60, sizeof(t60), r->t60);
        fmt_time(t100, sizeof(t100), r->t100);
        fmt_time(t12, sizeof(t12), r->t100_200);
        if (r->t402 >= 0) snprintf(t402, sizeof(t402), "%.2f @%.0f", r->t402, r->v402);
        else              snprintf(t402, sizeof(t402), "-");
        if (r->slope_pct != 0.0f) {
            size_t l = strlen(t402);
            snprintf(t402 + l, sizeof(t402) - l, "  %+.1f%%", r->slope_pct);
        }

        // header: run number + date/time (from the GPS clock); a bridged
        // GPS stream gap marks the run as less trustworthy - highlight it
        lv_obj_t *hdr = lv_label_create(item);
        lv_obj_set_style_text_font(hdr, &lv_font_montserrat_14, 0);
        bool gap = r->gap_s > 0.05f;
        lv_obj_set_style_text_color(hdr, gap ? PALETTE_NFS_ORANGE : PALETTE_GREY, 0);
        char when[40];
        if (r->year) {
            snprintf(when, sizeof(when), "%02d.%02d.%04d  %02d:%02d",
                     r->day, r->mon, r->year, r->hour, r->min);
        } else {
            when[0] = '\0';
        }
        const char *src = r->launch_imu ? "IMU" : "GNS";
        if (gap) {
            lv_label_set_text_fmt(hdr, "#%d  %s  %s  !gap %.1fs", i + 1, when, src, r->gap_s);
        } else {
            lv_label_set_text_fmt(hdr, "#%d  %s  %s", i + 1, when, src);
        }

        lv_obj_t *lbl = lv_label_create(item);
#ifdef BSP_BOARD_WS_S3_TOUCH_LCD_1_85
        // round screen: narrower list, one size down to keep lines unwrapped
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
#else
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
#endif
        lv_obj_set_style_text_color(lbl, PALETTE_WHITE, 0);
        lv_label_set_text_fmt(lbl, "60: %s  100: %s\n100-200: %s\n402m: %s",
                              t60, t100, t12, t402);
    }
}

static void history_build_screen(lv_obj_t *parent) {
    hist_panel = lv_obj_create(parent);
    lv_obj_set_size(hist_panel, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_center(hist_panel);
    lv_obj_set_style_bg_color(hist_panel, PALETTE_BLACK, 0);
    lv_obj_set_style_bg_opa(hist_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hist_panel, 0, 0);
    lv_obj_set_style_radius(hist_panel, 0, 0);
    lv_obj_set_style_pad_all(hist_panel, 0, 0);
    lv_obj_remove_flag(hist_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(hist_panel);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, PALETTE_NFS_CITRUS, 0);
    lv_label_set_text(title, "RESULTS");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

    lv_obj_t *hint = lv_label_create(hist_panel);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(hint, PALETTE_GREY, 0);
    lv_label_set_text(hint, "swipe left = delete  |  hold BOOT = close");
    lv_obj_align(hint, LV_ALIGN_TOP_MID, 0, 44);

    hist_list = lv_obj_create(hist_panel);
#ifdef BSP_BOARD_WS_S3_TOUCH_LCD_1_85
    // round screen: inset the list so it stays inside the circle
    lv_obj_set_size(hist_list, BSP_LCD_H_RES - 80, BSP_LCD_V_RES - 110);
#else
    lv_obj_set_size(hist_list, BSP_LCD_H_RES - 16, BSP_LCD_V_RES - 70);
#endif
    lv_obj_align(hist_list, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_obj_set_style_bg_opa(hist_list, LV_OPA_0, 0);
    lv_obj_set_style_border_width(hist_list, 0, 0);
    lv_obj_set_style_pad_all(hist_list, 2, 0);
    lv_obj_set_style_pad_row(hist_list, 6, 0);
    lv_obj_set_flex_flow(hist_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(hist_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(hist_list, LV_SCROLLBAR_MODE_OFF);

    lv_obj_add_flag(hist_panel, LV_OBJ_FLAG_HIDDEN);
}

// leave the on-map track review: remove the polyline and the result box
static void track_view_exit(void) {
    if (!track_view_active) return;
    track_view_active = false;
    GPSLocator::track_clear();
    lv_obj_add_flag(perf_label, LV_OBJ_FLAG_HIDDEN);
    // normal browse rules resume; the 15s timer will snap back to the car
    last_touch_tick = lv_tick_get();
}

// tap on a history entry: jump the map to the run and draw its track,
// with the run's numbers shown in the live-style overlay box
static void track_view_open(int index) {
    if (index < 0 || index >= history_count) return;
    PerfRecord *r = &history[index];
    if (r->track_id == 0) return;

    track_view_pt_t *pts = (track_view_pt_t *)heap_caps_malloc(
        2304 * sizeof(track_view_pt_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pts) return;
    int n = tracklog_load_csv(r->track_id, pts, 2304);
    if (n < 2) {
        free(pts);
        return;
    }

    history_toggle();               // close the list
    track_view_exit();              // clear a previously shown track
    track_view_active = true;
    follow_vehicle = false;

    // jump the map to the launch point and draw the run
    GPSLocator::show_initial_location((double)pts[0].lat, (double)pts[0].lon);
    GPSLocator::track_show(pts, n);
    free(pts);

    // live-mode style overlay with the stored numbers
    char buf[128];
    int len = 0;
    if (r->year) {
        len += snprintf(buf + len, sizeof(buf) - len, "%02d.%02d  %02d:%02d",
                        r->day, r->mon, r->hour, r->min);
    } else {
        len += snprintf(buf + len, sizeof(buf) - len, "#%d", index + 1);
    }
    len += snprintf(buf + len, sizeof(buf) - len, "  [%s]", r->launch_imu ? "IMU" : "GNS");
    if (r->t60 >= 0)      len += snprintf(buf + len, sizeof(buf) - len, "\n0-60  %.2fs", r->t60);
    if (r->t100 >= 0)     len += snprintf(buf + len, sizeof(buf) - len, "\n0-100  %.2fs", r->t100);
    if (r->t100_200 >= 0) len += snprintf(buf + len, sizeof(buf) - len, "\n100-200  %.2fs", r->t100_200);
    if (r->t402 >= 0)     len += snprintf(buf + len, sizeof(buf) - len, "\n402m  %.2fs @%.0f", r->t402, r->v402);
    if (r->slope_pct != 0.0f)
        len += snprintf(buf + len, sizeof(buf) - len, "\nslope %+.1f%%", r->slope_pct);
    if (r->gap_s > 0.05f) len += snprintf(buf + len, sizeof(buf) - len, "\n! gps gap %.1fs", r->gap_s);
    lv_label_set_text(perf_label, buf);
    lv_obj_remove_flag(perf_label, LV_OBJ_FLAG_HIDDEN);
}

static void history_toggle(void) {
    if (lv_obj_has_flag(hist_panel, LV_OBJ_FLAG_HIDDEN)) {
        track_view_exit();          // opening the list closes the track view
        history_refresh_list();
        lv_obj_remove_flag(hist_panel, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(hist_panel, LV_OBJ_FLAG_HIDDEN);
    }
}
#endif // SHOW_PERF

// ---------------------------------------------------------------- create
void overlays_create(lv_obj_t *parent) {
#if SHOW_SPEED
    speed_label = lv_label_create(parent);
    lv_obj_set_style_text_font(speed_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(speed_label, PALETTE_WHITE, 0);
    lv_label_set_text(speed_label, "GPS...");
    lv_obj_align(speed_label, LV_ALIGN_BOTTOM_MID, 0, -30);
#endif

#if SHOW_BATTERY
    batt_label = lv_label_create(parent);
    lv_obj_set_style_text_font(batt_label, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(batt_label, PALETTE_WHITE, 0);
    lv_obj_set_style_bg_color(batt_label, PALETTE_BLACK, 0);
    lv_obj_set_style_bg_opa(batt_label, LV_OPA_50, 0);
    lv_obj_set_style_pad_hor(batt_label, 6, 0);
    lv_obj_set_style_pad_ver(batt_label, 2, 0);
    lv_obj_set_style_radius(batt_label, 4, 0);
    lv_label_set_text(batt_label, "-.--V");
#ifdef BSP_BOARD_WS_S3_TOUCH_LCD_1_85
    // round screen: the physical corner is clipped, keep inside the circle
    lv_obj_align(batt_label, LV_ALIGN_TOP_LEFT, 52, 46);
#else
    lv_obj_align(batt_label, LV_ALIGN_TOP_LEFT, 8, 6);
#endif

    // restore the saved show/hide state (persisted at shutdown)
    {
        nvs_handle_t h;
        if (nvs_open("minimap", NVS_READONLY, &h) == ESP_OK) {
            uint8_t shown = 1;
            if (nvs_get_u8(h, "batt_show", &shown) == ESP_OK) {
                batt_visible = shown;
            }
            nvs_close(h);
        }
    }
    batt_apply_visibility();
#endif

#if SHOW_PERF
    perf_label = lv_label_create(parent);
    lv_obj_set_style_text_font(perf_label, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(perf_label, PALETTE_NFS_CITRUS, 0);
    lv_obj_set_style_text_align(perf_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_bg_color(perf_label, PALETTE_BLACK, 0);
    lv_obj_set_style_bg_opa(perf_label, LV_OPA_60, 0);
    lv_obj_set_style_pad_all(perf_label, 6, 0);
    lv_obj_set_style_radius(perf_label, 6, 0);
    lv_obj_align(perf_label, LV_ALIGN_CENTER, 0, -70);
    lv_obj_add_flag(perf_label, LV_OBJ_FLAG_HIDDEN);

    // record-mode indicator (BOOT double-click)
    rec_dot = lv_obj_create(parent);
    lv_obj_set_size(rec_dot, 14, 14);
    lv_obj_set_style_radius(rec_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(rec_dot, PALETTE_NFS_RED, 0);
    lv_obj_set_style_bg_opa(rec_dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(rec_dot, 0, 0);
#ifdef BSP_BOARD_WS_S3_TOUCH_LCD_1_85
    lv_obj_align(rec_dot, LV_ALIGN_TOP_RIGHT, -54, 48);   // inside the circle
#else
    lv_obj_align(rec_dot, LV_ALIGN_TOP_RIGHT, -15, 20);
#endif
    lv_obj_add_flag(rec_dot, LV_OBJ_FLAG_HIDDEN);

    history_load();
    history_build_screen(parent);
#endif

    // BOOT button as a plain input (it's free after boot)
    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);
}

// ---------------------------------------------------------------- tick
static void tick_speed(void) {
#if SHOW_SPEED
    static uint32_t t = 0;
    if (lv_tick_elaps(t) < 200) return;
    t = lv_tick_get();
    if (receiving_data) {
        lv_label_set_text_fmt(speed_label, "%d km/h", (int)(gps_speed_kmh + 0.5f));
    }
#endif
}

static void tick_battery(void) {
#if SHOW_BATTERY
    static uint32_t t = 0;
    if (lv_tick_elaps(t) < 1000) return;
    t = lv_tick_get();
#if USE_PMU
    int mv = pmu_battery_voltage_mv();
#else
    int mv = -1;
#endif
    char buf[40];
    int len;
    if (mv > 0) {
        len = snprintf(buf, sizeof(buf), "%d.%02dV %dsat", mv / 1000, (mv % 1000) / 10, gps_sat_count);
    } else {
        len = snprintf(buf, sizeof(buf), "-.--V %dsat", gps_sat_count);
    }
#if SHOW_PERF
    // measured GPS update rate (confirms the configured 10/20 Hz is live)
    int hz = (int)(perf_sample_rate() + 0.5f);
    if (hz > 0 && len < (int)sizeof(buf)) {
        snprintf(buf + len, sizeof(buf) - len, " %dHz", hz);
    }
#endif
    lv_label_set_text(batt_label, buf);
#endif
}

// live track: while a run is recording, redraw the growing polyline
// behind the vehicle a couple of times per second; the final shape stays
// on the map after the run until record mode is switched off
static void tick_live_track(void) {
#if SHOW_PERF
    if (!record_mode || track_view_active) return;
    static uint32_t t = 0;
    if (lv_tick_elaps(t) < 500) return;
    t = lv_tick_get();

    static track_view_pt_t *live = NULL;
    static int last_drawn = 0;
    int n = tracklog_count();
    if (n < 2 || n == last_drawn) return;
    if (!live) {
        live = (track_view_pt_t *)heap_caps_malloc(
            2304 * sizeof(track_view_pt_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!live) return;
    }
    n = tracklog_get_live(live, 2304);
    if (n >= 2) {
        GPSLocator::track_show(live, n);
    }
    last_drawn = n;
#endif
}

static void tick_perf(void) {
#if SHOW_PERF
    // show timing results while a run is active and for 30s after
    static uint32_t last_seq = 0;
    static uint32_t shown_tick = 0;
    uint32_t pseq = perf_seq();
    if (pseq != last_seq && record_mode) {
        last_seq = pseq;
        shown_tick = lv_tick_get();

        perf_results_t r;
        perf_get_results(&r);
        char buf[128];
        int n = 0;
        const char *src = r.launch_imu ? "IMU" : "GNS";
        if (r.run_active && r.t_0_60 < 0) {
            n = snprintf(buf, sizeof(buf), "RUN! [%s]", src);
        } else if (r.t_0_60 >= 0) {
            n = snprintf(buf, sizeof(buf), "[%s]\n", src);
        }
        if (r.t_0_60 >= 0)
            n += snprintf(buf + n, sizeof(buf) - n, "0-60  %.2fs", r.t_0_60);
        if (r.t_0_100 >= 0)
            n += snprintf(buf + n, sizeof(buf) - n, "\n0-100  %.2fs", r.t_0_100);
        if (r.t_100_200 >= 0)
            n += snprintf(buf + n, sizeof(buf) - n, "\n100-200  %.2fs", r.t_100_200);
        if (r.t_402m >= 0)
            n += snprintf(buf + n, sizeof(buf) - n, "\n402m  %.2fs @%.0f", r.t_402m, r.v_402m_kmh);
        if (r.slope_pct != 0.0f)
            n += snprintf(buf + n, sizeof(buf) - n, "\nslope %+.1f%%", r.slope_pct);
        if (r.gap_s > 0.05f)
            n += snprintf(buf + n, sizeof(buf) - n, "\n! gps gap %.1fs", r.gap_s);
        if (n > 0) {
            lv_label_set_text(perf_label, buf);
            lv_obj_remove_flag(perf_label, LV_OBJ_FLAG_HIDDEN);
        }

        // a run just finished with at least one milestone: save it
        static bool prev_run_active = false;
        if (prev_run_active && !r.run_active && r.t_0_60 >= 0) {
            history_add(&r);
        }
        prev_run_active = r.run_active;
    }
    if (shown_tick && lv_tick_elaps(shown_tick) > 30000) {
        shown_tick = 0;
        lv_obj_add_flag(perf_label, LV_OBJ_FLAG_HIDDEN);
    }
#endif
}

// BOOT button: single click = battery overlay, double click = record
// mode, hold = results history screen
static void tick_button(void) {
    static int btn_prev = 1;
    static uint32_t btn_down_tick = 0;
    static uint32_t pending_click_tick = 0;   // single click awaiting a second

    int btn = gpio_get_level(BOOT_BUTTON_GPIO);
    if (btn_prev == 1 && btn == 0) {
        btn_down_tick = lv_tick_get();
    } else if (btn_prev == 0 && btn == 1) {
        uint32_t held = lv_tick_elaps(btn_down_tick);
        if (held > 30 && held < 800) {
#if SHOW_PERF
            if (pending_click_tick && lv_tick_elaps(pending_click_tick) < 350) {
                // double click: toggle record mode; the GPS switches with
                // it (cruise multi-GNSS <-> high-rate GPS-only for timing)
                pending_click_tick = 0;
                track_view_exit();   // arming a run clears any track review
                record_mode = !record_mode;
#if GPS_SOURCE == GPS_SOURCE_UART
                gps_set_perf_mode(record_mode);
#endif
                if (record_mode) {
                    lv_obj_remove_flag(rec_dot, LV_OBJ_FLAG_HIDDEN);
                } else {
                    lv_obj_add_flag(rec_dot, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(perf_label, LV_OBJ_FLAG_HIDDEN);
                    GPSLocator::track_clear();   // drop the live-drawn track
                }
            } else {
                pending_click_tick = lv_tick_get();
            }
#else
            pending_click_tick = lv_tick_get();
#endif
        }
#if SHOW_PERF
        else if (held >= 800 && held < 5000) {
            // long press: results history screen
            pending_click_tick = 0;
            history_toggle();
        }
#endif
    }
    btn_prev = btn;

#if SHOW_BATTERY
    // the click window expired without a second click: confirmed single
    if (pending_click_tick && lv_tick_elaps(pending_click_tick) >= 350) {
        pending_click_tick = 0;
        batt_visible = !batt_visible;
        batt_apply_visibility();
    }
#endif
}

void overlays_tick(void) {
    tick_speed();
    tick_battery();
    tick_perf();
    tick_live_track();
    tick_button();
}
