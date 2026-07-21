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
    float slope_pct;   // course slope over the measured distance (Doppler
                       // velD integrated): negative = downhill (favorable)
    bool launch_imu;   // true = launch instant from the IMU, false = from GNSS
    bool run_active;   // a launch is currently being timed
} perf_results_t;

// feed one velocity sample; itow_ms = GPS time of week (NAV-PVT iTOW),
// tick_ms = system tick timestamp in milliseconds when the sample was
// captured, gspeed_mms = Doppler ground speed, veld_mms = Doppler down
// velocity (NED, positive = descending), both mm/s. Call from any task.
void perf_feed(uint32_t itow_ms, uint32_t tick_ms, int32_t gspeed_mms, int32_t veld_mms);

// feed one accelerometer sample (m/s^2) with a millisecond timestamp;
// used to pin the launch instant between GNSS samples (GNSS+IMU fusion)
void perf_imu_feed(float ax, float ay, float az, uint32_t tick_ms);

// apply a fixed timing offset (milliseconds) to all perf results.
void perf_set_calibration_offset(int32_t offset_ms);

// NHRA roll-out, applied to the quarter-mile ET only (0-60/0-100 stay timed
// from first movement, as Dragy does). 305 = the standard 1 foot: the ET
// clock starts once the car has rolled that far. 0 disables it.
void perf_set_rollout_mm(int32_t mm);

// read the current calibration offset in milliseconds.
int32_t perf_get_calibration_offset(void);

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
