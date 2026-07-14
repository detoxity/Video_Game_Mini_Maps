#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "PerfMeter.h"

static const char *TAG = "perf";

// thresholds in mm/s
#define V_STILL       220     // ~0.8 km/h: considered standing. Raised from
                              // 0.5 - GPS-only perf mode has standstill speed
                              // noise that kept it from arming at the line.
#define V_LAUNCH      278     // ~1.0 km/h: run starts when crossed
#define STILL_ARM_COUNT 6     // net "still" samples needed to arm
#define V_60_KMH      16667
#define V_100_KMH     27778
#define V_200_KMH     55556
#define DIST_QUARTER  402336.0f   // 1/4 mile in mm
#define RUN_TIMEOUT_MS 90000

typedef enum { ST_IDLE, ST_ARMED, ST_RUN } perf_state_t;

// 3-sample moving average (the low-pass filter)
static int32_t ma_buf[3];
static int ma_fill = 0, ma_idx = 0;

static perf_state_t state = ST_IDLE;
static int still_samples = 0;
static bool launch_unconfirmed = false;   // IMU started a run GNSS hasn't confirmed

static int32_t v_prev = -1;        // previous filtered speed, mm/s
static uint32_t t_prev = 0;        // previous iTOW, ms
static float accel_ms2 = 0.0f;
static float rate_hz = 0.0f;

static uint32_t t_launch = 0;      // iTOW at launch
static float dist_mm = 0.0f;
static float dh_mm = 0.0f;         // height change since launch (velD integrated,
                                   // negative = descending)
static int32_t vd_prev = 0;        // previous Doppler down velocity, mm/s

// ---- GNSS+IMU fusion: launch-instant refinement -------------------------
// The accelerometer sees the car move within ~10ms; GNSS only ~2-3 samples
// later. While armed we learn the gravity vector, and a sustained
// deviation from it marks the true start of movement, mapped into the
// GPS time-of-week domain via the tick offset of the last GNSS sample.
#define IMU_LAUNCH_THRESHOLD  1.5f   // m/s^2 above the gravity baseline
#define IMU_BURST_SAMPLES     2      // sustained samples (~10ms) to count as
                                     // launch - 1 was too twitchy (idle
                                     // vibration / revving faked launches)
#define IMU_LAUNCH_MAX_AGE_MS 800    // reject an IMU launch instant older than
                                     // this vs the GNSS sample (bad tick bridge)
#define LAUNCH_CONFIRM_MS     700    // GNSS must show real movement within this
                                     // of an IMU-triggered launch or it's bogus

// arming from IMU stillness - the accelerometer settles in ~200ms, versus
// waiting on the noisy GPS standstill speed which could take seconds or stall.
// The threshold is deliberately loose: it must tolerate idle/rev engine
// vibration coupling through the mount (else it never arms), and permissive
// arming is safe here - a launch still needs a real acceleration burst and a
// false one is discarded by the GNSS-confirmation guard.
#define IMU_STILL_DEV      0.6f       // m/s^2 deviation counted as not moving
#define IMU_STILL_SAMPLES  40        // ~200ms sustained at 200Hz -> still

static float g0[3] = {0.0f, 0.0f, 9.81f};   // gravity estimate while standing
static bool g0_valid = false;
static int imu_burst = 0;
static uint32_t imu_burst_start_tick = 0;
static volatile uint32_t imu_launch_pitow = 0;   // pseudo-iTOW of movement start
static volatile int32_t tick_to_itow = 0;        // itow - tick, from last GNSS sample
static volatile bool tick_to_itow_valid = false;

// IMU stillness detector (drives arming; separate from the launch baseline)
static float still_g[3] = {0.0f, 0.0f, 9.81f};
static bool still_g_valid = false;
static int imu_still_count = 0;
static volatile bool imu_still = false;
static volatile bool imu_present = false;   // true once the IMU has fed a sample

void perf_imu_feed(float ax, float ay, float az, uint32_t tick_ms) {
    imu_present = true;

    // stillness detector runs in every state (used for arming): deviation
    // from a slowly-tracked gravity vector, sustained below a small
    // threshold, means the vehicle is physically stationary
    if (!still_g_valid) {
        still_g[0] = ax; still_g[1] = ay; still_g[2] = az;
        still_g_valid = true;
    }
    float sdx = ax - still_g[0], sdy = ay - still_g[1], sdz = az - still_g[2];
    float sdev = sqrtf(sdx * sdx + sdy * sdy + sdz * sdz);
    still_g[0] += 0.05f * (ax - still_g[0]);
    still_g[1] += 0.05f * (ay - still_g[1]);
    still_g[2] += 0.05f * (az - still_g[2]);
    if (sdev < IMU_STILL_DEV) {
        if (imu_still_count < IMU_STILL_SAMPLES) imu_still_count++;
        if (imu_still_count >= IMU_STILL_SAMPLES) imu_still = true;
    } else {
        imu_still_count = 0;
        imu_still = false;
    }

    if (state != ST_ARMED) {
        imu_burst = 0;
        if (state == ST_IDLE) {
            imu_launch_pitow = 0;
            g0_valid = false;
        }
        return;
    }

    if (!g0_valid) {
        g0[0] = ax; g0[1] = ay; g0[2] = az;
        g0_valid = true;
        return;
    }

    float dx = ax - g0[0], dy = ay - g0[1], dz = az - g0[2];
    float dev = sqrtf(dx * dx + dy * dy + dz * dz);

    if (dev < 0.4f) {
        // quiet: track slow gravity drift (temperature, small tilts)
        const float a = 0.02f;
        g0[0] += a * (ax - g0[0]);
        g0[1] += a * (ay - g0[1]);
        g0[2] += a * (az - g0[2]);
        imu_burst = 0;
    } else if (dev > IMU_LAUNCH_THRESHOLD) {
        if (imu_burst == 0) {
            imu_burst_start_tick = tick_ms;
        }
        if (++imu_burst >= IMU_BURST_SAMPLES && imu_launch_pitow == 0 && tick_to_itow_valid) {
            imu_launch_pitow = imu_burst_start_tick + tick_to_itow;
        }
    } else {
        imu_burst = 0;
    }
}

static perf_results_t results = {-1, -1, -1, -1, -1, -1, 0, 0, false};
static volatile uint32_t seq = 0;

float perf_current_accel(void) { return accel_ms2; }
float perf_sample_rate(void)   { return rate_hz; }
uint32_t perf_seq(void)        { return seq; }

void perf_get_results(perf_results_t *out) {
    *out = results;
}

// time (ms since launch) at which speed crossed `target`, linearly
// interpolated between the previous and current sample
static float cross_time_ms(uint32_t t_now, uint32_t dt, int32_t v_now, int32_t target) {
    float f = 1.0f;
    if (v_now != v_prev) {
        f = (float)(target - v_prev) / (float)(v_now - v_prev);
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
    }
    return (float)(t_now - dt - t_launch) + f * (float)dt;
}

static void finish_run(const char *why) {
    // runs that never reached 402m still get a slope over what was driven
    if (results.t_402m < 0 && dist_mm > 50000.0f) {
        results.slope_pct = dh_mm / dist_mm * 100.0f;
    }
    ESP_LOGI(TAG, "run ended (%s): 0-60 %.2fs  0-100 %.2fs  100-200 %.2fs  402m %.2fs @ %.0f km/h  (slope %+.1f%%, gaps %.1fs)",
             why, results.t_0_60, results.t_0_100, results.t_100_200, results.t_402m, results.v_402m_kmh,
             results.slope_pct, results.gap_s);
    results.run_active = false;
    state = ST_IDLE;
    still_samples = 0;
    launch_unconfirmed = false;
    seq++;
}

// applied by main at startup from PERF_CALIBRATION_OFFSET_MS (app_config.h);
// kept as a runtime value so PerfMeter stays independent of the app
static int32_t calibration_offset_ms = 0;

void perf_set_calibration_offset(int32_t offset_ms) {
    calibration_offset_ms = offset_ms;
}

int32_t perf_get_calibration_offset(void) {
    return calibration_offset_ms;
}

void perf_feed(uint32_t itow_ms, uint32_t tick_ms, int32_t gspeed_mms, int32_t veld_mms) {
    // low-pass: 3-sample moving average
    ma_buf[ma_idx] = gspeed_mms;
    ma_idx = (ma_idx + 1) % 3;
    if (ma_fill < 3) {
        ma_fill++;
        return;
    }
    int32_t v = (ma_buf[0] + ma_buf[1] + ma_buf[2]) / 3;

    if (v_prev < 0 || itow_ms <= t_prev || itow_ms - t_prev > 1000) {
        // first sample, week rollover or gap: resync.
        // Mid-run a stream gap must NOT silently drop the metres travelled -
        // that made every 402m result long by however far the car went while
        // the stream was down (speed crossings are immune, distance is not).
        // Bridge plausible gaps with the raw-speed trapezoid and flag it.
        if (state == ST_RUN && itow_ms > t_prev) {
            uint32_t gap = itow_ms - t_prev;
            if (gap <= 5000) {
                dist_mm += (float)(gspeed_mms + v_prev) * 0.5f * (float)gap / 1000.0f;
                dh_mm -= (float)(veld_mms + vd_prev) * 0.5f * (float)gap / 1000.0f;
                results.gap_s += (float)gap / 1000.0f;
                seq++;
                ESP_LOGW(TAG, "stream gap %lums mid-run - distance bridged",
                         (unsigned long)gap);
            } else {
                finish_run("stream lost");
            }
        }
        v_prev = v;
        vd_prev = veld_mms;
        t_prev = itow_ms;
        return;
    }
    uint32_t dt = itow_ms - t_prev;

    // bridge between the IMU's tick clock and the GNSS time of week
    tick_to_itow = (int32_t)(itow_ms - tick_ms);
    tick_to_itow_valid = true;

    // acceleration from consecutive filtered samples: (mm/s)/ms == m/s^2
    accel_ms2 = (float)(v - v_prev) / (float)dt;
    // smoothed observed sample rate
    float inst = 1000.0f / (float)dt;
    rate_hz = (rate_hz == 0.0f) ? inst : rate_hz + 0.05f * (inst - rate_hz);

    switch (state) {
    case ST_IDLE:
        if (imu_present) {
            // IMU stillness: fast (~200ms) and immune to GPS speed noise
            if (imu_still) {
                state = ST_ARMED;
                ESP_LOGI(TAG, "armed (IMU still, GPS %.0f Hz)", rate_hz);
            }
        } else {
            // no IMU: fall back to GPS-speed stillness. A single noise spike
            // must not reset the whole count - decay instead, so arming needs
            // predominantly-still samples, not a perfect uninterrupted run.
            if (v < V_STILL) {
                if (still_samples < STILL_ARM_COUNT) still_samples++;
                if (still_samples >= STILL_ARM_COUNT) {
                    state = ST_ARMED;
                    ESP_LOGI(TAG, "armed (GPS still, %.0f Hz)", rate_hz);
                }
            } else if (still_samples > 0) {
                still_samples--;
            }
        }
        break;

    case ST_ARMED:
        // IMU-primary: the accelerometer sees movement ~1 sample before GNSS
        // Doppler does, so it drives the launch instant - but only if the
        // mark is plausible, and the run stays provisional until GNSS
        // confirms real movement (see ST_RUN) so a bump can't fake a launch.
        if (imu_launch_pitow != 0) {
            uint32_t ip = imu_launch_pitow;
            imu_launch_pitow = 0;
            if (ip <= itow_ms && itow_ms - ip < IMU_LAUNCH_MAX_AGE_MS) {
                state = ST_RUN;
                t_launch = ip;
                launch_unconfirmed = true;
                dist_mm = 0.0f;
                dh_mm = 0.0f;
                results.t_0_60 = results.t_0_100 = results.t_0_200 = -1.0f;
                results.t_100_200 = results.t_402m = results.v_402m_kmh = -1.0f;
                results.gap_s = 0.0f;
                results.slope_pct = 0.0f;
                results.run_active = true;
                seq++;
                ESP_LOGI(TAG, "launch! (IMU, awaiting GNSS confirm)");
                break;
            }
            ESP_LOGI(TAG, "IMU launch mark ignored (implausible age)");
        }
        if (v >= V_LAUNCH && accel_ms2 > 0.0f) {
            state = ST_RUN;
            launch_unconfirmed = false;
            // interpolate the true standstill->launch crossing
            t_launch = itow_ms - dt;
            float f = (v != v_prev) ? (float)(V_LAUNCH - v_prev) / (float)(v - v_prev) : 0.0f;
            if (f > 0.0f && f <= 1.0f) t_launch += (uint32_t)(f * dt);

            dist_mm = 0.0f;
            dh_mm = 0.0f;
            results.t_0_60 = results.t_0_100 = results.t_0_200 = -1.0f;
            results.t_100_200 = results.t_402m = results.v_402m_kmh = -1.0f;
            results.gap_s = 0.0f;
            results.slope_pct = 0.0f;
            results.run_active = true;
            seq++;
            ESP_LOGI(TAG, "launch! (GNSS)");
        }
        break;

    case ST_RUN: {
        // confirm an IMU-triggered launch: real movement must appear soon,
        // else it was vibration at the line - discard the run and re-arm
        if (launch_unconfirmed) {
            if (v >= V_LAUNCH) {
                launch_unconfirmed = false;
            } else if (itow_ms - t_launch > LAUNCH_CONFIRM_MS) {
                ESP_LOGI(TAG, "false launch (no GNSS movement) - re-arming");
                state = ST_ARMED;
                launch_unconfirmed = false;
                results.run_active = false;
                seq++;
                break;   // post-switch updates v_prev/vd_prev/t_prev
            }
        }

        // distance: trapezoidal integration, mm/s * ms / 1000 = mm
        float d_step = (float)(v + v_prev) * 0.5f * (float)dt / 1000.0f;
        float dist_before = dist_mm;
        dist_mm += d_step;
        // height change from Doppler down-velocity (much cleaner than GNSS
        // altitude); negative = descending
        dh_mm -= (float)(veld_mms + vd_prev) * 0.5f * (float)dt / 1000.0f;

        if (results.t_0_60 < 0 && v >= V_60_KMH) {
            results.t_0_60 = cross_time_ms(itow_ms, dt, v, V_60_KMH) / 1000.0f;
            results.t_0_60 += (float)calibration_offset_ms / 1000.0f;
            seq++;
            ESP_LOGI(TAG, "0-60 km/h: %.2f s", results.t_0_60);
        }
        if (results.t_0_100 < 0 && v >= V_100_KMH) {
            results.t_0_100 = cross_time_ms(itow_ms, dt, v, V_100_KMH) / 1000.0f;
            results.t_0_100 += (float)calibration_offset_ms / 1000.0f;
            seq++;
            ESP_LOGI(TAG, "0-100 km/h: %.2f s", results.t_0_100);
        }
        if (results.t_0_200 < 0 && v >= V_200_KMH) {
            results.t_0_200 = cross_time_ms(itow_ms, dt, v, V_200_KMH) / 1000.0f;
            results.t_0_200 += (float)calibration_offset_ms / 1000.0f;
            if (results.t_0_100 >= 0) {
                results.t_100_200 = results.t_0_200 - results.t_0_100;
            }
            seq++;
            ESP_LOGI(TAG, "0-200 km/h: %.2f s (100-200: %.2f s)", results.t_0_200, results.t_100_200);
        }
        if (results.t_402m < 0 && dist_mm >= DIST_QUARTER) {
            float f = (d_step > 0.0f) ? (DIST_QUARTER - dist_before) / d_step : 1.0f;
            results.t_402m = ((float)(itow_ms - dt - t_launch) + f * (float)dt) / 1000.0f;
            results.t_402m += (float)calibration_offset_ms / 1000.0f;
            // trap speed at the interpolated crossing instant, not at the
            // (up to one sample later) detection sample
            results.v_402m_kmh = ((float)v_prev + f * (float)(v - v_prev)) * 0.0036f;
            // Dragy-style course slope over the quarter mile
            results.slope_pct = dh_mm / DIST_QUARTER * 100.0f;
            seq++;
            ESP_LOGI(TAG, "402m: %.2f s @ %.0f km/h", results.t_402m, results.v_402m_kmh);
        }

        if (results.t_402m >= 0 && results.t_0_200 >= 0) {
            finish_run("complete");
        } else if (v < V_STILL) {
            finish_run("stopped");
        } else if (itow_ms - t_launch > RUN_TIMEOUT_MS) {
            finish_run("timeout");
        }
        break;
    }
    }

    v_prev = v;
    vd_prev = veld_mms;
    t_prev = itow_ms;
}
