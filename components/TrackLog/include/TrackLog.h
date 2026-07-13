// Track recorder for perf-meter runs: captures the GPS trace of every
// timed run, saves it to the SD card as CSV and loads it back for the
// on-map track view (opened by tapping a result in the history screen).
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float lat, lon;
    float v_kmh;
} track_view_pt_t;

// allocate the capture buffer (PSRAM); call once at startup
void tracklog_init(void);

// feed every positioned GPS sample (called from the GPS/demo task on the
// measurement core); points are recorded while a perf run is active
void tracklog_point(float lat, float lon, int32_t v_mms, uint32_t itow_ms);

// points captured for the most recently finished run
int tracklog_count(void);

// snapshot the capture buffer (live drawing while a run is in progress);
// returns the number of points copied
int tracklog_get_live(track_view_pt_t *out, int max_pts);

// write the captured run to /sdcard/tracks/rNNNNN.csv (8.3 name - FatFS LFN is off)
// (itow_ms,lat,lon,speed_kmh per line); returns the id used in the
// filename, 0 if there was nothing to save or the write failed
uint32_t tracklog_save_csv(void);

// delete a saved track file (history entry swiped away)
void tracklog_delete_csv(uint32_t id);

// read a saved track back; returns the number of points (0 = missing)
int tracklog_load_csv(uint32_t id, track_view_pt_t *out, int max_pts);

#ifdef __cplusplus
}
#endif

