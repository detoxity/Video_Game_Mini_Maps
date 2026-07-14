#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs.h"

#include "TrackLog.h"
#include "PerfMeter.h"

static const char *TAG = "track";

#define TRACK_DIR      "/sdcard/tracks"
// 90s run timeout at 25Hz plus headroom
#define TRACK_MAX_PTS  2304

typedef struct {
    float lat, lon;
    int32_t v_mms;
    uint32_t itow;
    uint32_t tick_ms;
} track_pt_t;

static track_pt_t *buf = NULL;
static volatile int count = 0;
static bool was_active = false;

void tracklog_init(void) {
    buf = heap_caps_malloc(TRACK_MAX_PTS * sizeof(track_pt_t),
                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGW(TAG, "buffer allocation failed - track logging disabled");
    }
}

void tracklog_point(float lat, float lon, int32_t v_mms, uint32_t itow_ms,
                    uint32_t tick_ms) {
    if (!buf) return;

    perf_results_t r;
    perf_get_results(&r);
    if (!r.run_active) {
        was_active = false;   // buffer keeps the finished run until saved
        return;
    }
    if (!was_active) {
        was_active = true;
        count = 0;            // new run: restart capture
    }
    if (count < TRACK_MAX_PTS) {
        buf[count] = (track_pt_t){lat, lon, v_mms, itow_ms, tick_ms};
        count++;
    }
}

int tracklog_count(void) {
    return count;
}

int tracklog_get_live(track_view_pt_t *out, int max_pts) {
    if (!buf || !out) return 0;
    // points below `count` are already written and stable; the GPS task
    // only appends, so copying while it runs is safe
    int n = count;
    if (n > max_pts) n = max_pts;
    for (int i = 0; i < n; i++) {
        out[i].lat = buf[i].lat;
        out[i].lon = buf[i].lon;
        out[i].v_kmh = (float)buf[i].v_mms * 0.0036f;
    }
    return n;
}

static uint32_t next_track_id(void) {
    uint32_t id = 0;
    nvs_handle_t h;
    if (nvs_open("minimap", NVS_READWRITE, &h) == ESP_OK) {
        nvs_get_u32(h, "trk_seq", &id);
        id++;
        nvs_set_u32(h, "trk_seq", id);
        nvs_commit(h);
        nvs_close(h);
    }
    return id;
}

uint32_t tracklog_save_csv(void) {
    if (!buf || count < 2) return 0;

    mkdir(TRACK_DIR, 0775);   // idempotent
    uint32_t id = next_track_id();
    if (id == 0) return 0;

    char path[48];
    snprintf(path, sizeof(path), TRACK_DIR "/r%05lu.csv", (unsigned long)id);
    FILE *f = fopen(path, "w");
    if (!f) {
        ESP_LOGW(TAG, "can't create %s", path);
        return 0;
    }
    fprintf(f, "itow_ms,tick_ms,lat,lon,speed_kmh\n");
    int n = count;
    for (int i = 0; i < n; i++) {
        fprintf(f, "%lu,%lu,%.7f,%.7f,%.2f\n",
                (unsigned long)buf[i].itow,
                (unsigned long)buf[i].tick_ms,
                (double)buf[i].lat, (double)buf[i].lon,
                (double)buf[i].v_mms * 0.0036);
    }
    fclose(f);
    ESP_LOGI(TAG, "saved %d points to %s", n, path);
    return id;
}

void tracklog_delete_csv(uint32_t id) {
    if (id == 0) return;
    char path[48];
    snprintf(path, sizeof(path), TRACK_DIR "/r%05lu.csv", (unsigned long)id);
    remove(path);
}

int tracklog_load_csv(uint32_t id, track_view_pt_t *out, int max_pts) {
    if (id == 0 || !out || max_pts <= 0) return 0;

    char path[48];
    snprintf(path, sizeof(path), TRACK_DIR "/r%05lu.csv", (unsigned long)id);
    FILE *f = fopen(path, "r");
    if (!f) {
        ESP_LOGW(TAG, "track %s not found", path);
        return 0;
    }
    char line[80];
    int n = 0;
    fgets(line, sizeof(line), f);   // header
    while (n < max_pts && fgets(line, sizeof(line), f)) {
        unsigned long itow, tick_ms;
        float lat, lon, v;
        int fields = sscanf(line, "%lu,%lu,%f,%f,%f", &itow, &tick_ms, &lat, &lon, &v);
        if (fields == 5) {
            out[n].lat = lat;
            out[n].lon = lon;
            out[n].v_kmh = v;
            n++;
        } else if (fields == 4) {
            out[n].lat = lat;
            out[n].lon = lon;
            out[n].v_kmh = v;
            n++;
        }
    }
    fclose(f);
    return n;
}

