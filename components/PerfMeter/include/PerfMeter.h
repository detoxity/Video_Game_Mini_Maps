// GPS performance meter (Dragy-style): feeds on UBX NAV-PVT Doppler
// velocity samples, detects a standing-start launch and times
// 0-60 km/h, 0-100 km/h and the 402m (1/4 mile) with trap speed.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float t_0_60;      // seconds, <0 = not (yet) reached
    float t_0_100;
    float t_0_200;
    float t_100_200;   // rolling 100->200 within the run
    float t_402m;      // 1/4 mile time
    float v_402m_kmh;  // trap speed at 402m
    float gap_s;       // GPS stream time lost mid-run and bridged (0 = clean;
                       // distance over gaps is estimated by trapezoid)
    bool run_active;   // a launch is currently being timed
} perf_results_t;

// feed one velocity sample; itow_ms = GPS time of week (NAV-PVT iTOW),
// gspeed_mms = Doppler ground speed in mm/s. Call from any task.
void perf_feed(uint32_t itow_ms, int32_t gspeed_mms);

// feed one accelerometer sample (m/s^2) with a millisecond timestamp;
// used to pin the launch instant between GNSS samples (GNSS+IMU fusion)
void perf_imu_feed(float ax, float ay, float az, uint32_t tick_ms);

// filtered acceleration in m/s^2 (positive = accelerating)
float perf_current_accel(void);

// measured GPS sample rate in Hz (0 until enough samples)
float perf_sample_rate(void);

// bumped every time a milestone is recorded or a run starts/ends
uint32_t perf_seq(void);

// copy out the latest results
void perf_get_results(perf_results_t *out);

#ifdef __cplusplus
}
#endif
