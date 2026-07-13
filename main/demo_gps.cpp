#include "app_config.h"

#if GPS_SOURCE == GPS_SOURCE_DEMO

#include <stdio.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_state.h"
#include "demo_gps.h"
#include "PerfMeter.h"
#include "TrackLog.h"

// Reference numbers at 3.5 m/s^2: 0-60 4.68s, 0-100 7.86s,
// 0-200 15.79s, 100-200 7.94s (each includes the 1 km/h start offset).
#define DEMO_TICK_MS    55        // mimic the SAM-M8Q's 18Hz sample rate
#define DEMO_ACCEL_MMS2 3500.0f   // launch acceleration, mm/s^2
#define DEMO_VMAX_MMS   (41667.0f * 2)  // 300 km/h (exercises the 200 km/h milestones)
#define DEMO_LAT0       50.4419
#define DEMO_LON0       30.5208
#define DEMO_BEARING    (35.0 * M_PI / 180.0)   // up Khreshchatyk, roughly NE

static void demo_gps_task(void *arg) {
    float v = 0.0f;          // mm/s
    double dist_m = 0.0;     // along-track distance from the start line
    int phase = 0;           // 0 = standing, 1 = accelerating, 2 = braking
    int phase_ticks = 0;
    uint32_t itow = 1000000; // fake GPS time of week

    vTaskDelay(pdMS_TO_TICKS(2000)); // let UI come up first

    while (true) {
        itow += DEMO_TICK_MS;
        phase_ticks++;

        switch (phase) {
        case 0:   // standing at the line (perf meter arms here)
            v = 0.0f;
            if (phase_ticks > 6000 / DEMO_TICK_MS) {
                phase = 1;
                phase_ticks = 0;
                printf("DEMO: launch!\n");
            }
            break;
        case 1:   // full send
            v += DEMO_ACCEL_MMS2 * DEMO_TICK_MS / 1000.0f;
            if (v >= DEMO_VMAX_MMS) {
                v = DEMO_VMAX_MMS;
            }
            if (dist_m > 600.0) {   // past the trap and the 200 mark - brake
                phase = 2;
                phase_ticks = 0;
            }
            break;
        case 2:   // brake to a stop, then reset to the start line
            v -= 6000.0f * DEMO_TICK_MS / 1000.0f;
            if (v <= 0.0f) {
                v = 0.0f;
                dist_m = 0.0;
                phase = 0;
                phase_ticks = 0;
                printf("DEMO: back to the start line\n");
            }
            break;
        }

        dist_m += (double)v * DEMO_TICK_MS / 1e6;

        new_latitude  = (float)(DEMO_LAT0 + dist_m * cos(DEMO_BEARING) / 111320.0);
        new_longitude = (float)(DEMO_LON0 + dist_m * sin(DEMO_BEARING) /
                                (111320.0 * cos(DEMO_LAT0 * M_PI / 180.0)));
        gps_speed_kmh = v * 0.0036f;
        receiving_data = true;
        perf_feed(itow, (int32_t)v);
        tracklog_point(new_latitude, new_longitude, (int32_t)v, itow);
        data_ready = true;

        vTaskDelay(pdMS_TO_TICKS(DEMO_TICK_MS));
    }
}

void demo_gps_start(void) {
    // same core as the real GPS task (measurement world, core 1)
    xTaskCreatePinnedToCore(demo_gps_task, "demo_gps", 4096, NULL, 5, NULL, 1);
}

#endif // GPS_SOURCE_DEMO
