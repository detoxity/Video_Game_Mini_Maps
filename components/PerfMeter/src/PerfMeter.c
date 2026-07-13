#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "PerfMeter.h"

static const char *TAG = "perf";

// thresholds in mm/s
#define V_STILL       140     // ~0.5 km/h: considered standing
#define V_LAUNCH      278     // ~1.0 km/h: run starts when crossed
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

static int32_t v_prev = -1;        // previous filtered speed, mm/s
static uint32_t t_prev = 0;        // previous iTOW, ms
static float accel_ms2 = 0.0f;
static float rate_hz = 0.0f;

static uint32_t t_launch = 0;      // iTOW at launch
static float dist_mm = 0.0f;

// ---- GNSS+IMU fusion: launch-instant refinement -------------------------
// The accelerometer sees the car move within ~10ms; GNSS only ~2-3 samples
// later. While armed we learn the gravity vector, and a sustained
// deviation from it marks the true start of movement, mapped into the
// GPS time-of-week domain via the tick offset of the last GNSS sample.
#define IMU_LAUNCH_THRESHOLD  1.5f   // m/s^2 above the gravity baseline
#define IMU_BURST_SAMPLES     3      // sustained samples to count as launch

static float g0[3] = {0.0f, 0.0f, 9.81f};   // gravity estimate while standing
static bool g0_valid = false;
static int imu_burst = 0;
static uint32_t imu_burst_start_tick = 0;
static volatile uint32_t imu_launch_pitow = 0;   // pseudo-iTOW of movement start
static volatile int32_t tick_to_itow = 0;        // itow - tick, from last GNSS sample
static volatile bool tick_to_itow_valid = false;

void perf_imu_feed(float ax, float ay, float az, uint32_t tick_ms) {
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

static perf_results_t results = {-1, -1, -1, -1, -1, -1, 0, false};
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
    ESP_LOGI(TAG, "run ended (%s): 0-60 %.2fs  0-100 %.2fs  100-200 %.2fs  402m %.2fs @ %.0f km/h  (stream gaps %.1fs)",
             why, results.t_0_60, results.t_0_100, results.t_100_200, results.t_402m, results.v_402m_kmh,
             results.gap_s);
    results.run_active = false;
    state = ST_IDLE;
    still_samples = 0;
    seq++;
}

void perf_feed(uint32_t itow_ms, int32_t gspeed_mms) {
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
                results.gap_s += (float)gap / 1000.0f;
                seq++;
                ESP_LOGW(TAG, "stream gap %lums mid-run - distance bridged",
                         (unsigned long)gap);
            } else {
                finish_run("stream lost");
            }
        }
        v_prev = v;
        t_prev = itow_ms;
        return;
    }
    uint32_t dt = itow_ms - t_prev;

    // bridge between the IMU's tick clock and the GNSS time of week
    tick_to_itow = (int32_t)(itow_ms - (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS));
    tick_to_itow_valid = true;

    // acceleration from consecutive filtered samples: (mm/s)/ms == m/s^2
    accel_ms2 = (float)(v - v_prev) / (float)dt;
    // smoothed observed sample rate
    float inst = 1000.0f / (float)dt;
    rate_hz = (rate_hz == 0.0f) ? inst : rate_hz + 0.05f * (inst - rate_hz);

    switch (state) {
    case ST_IDLE:
        if (v < V_STILL) {
            if (++still_samples >= 5) {
                state = ST_ARMED;
                ESP_LOGI(TAG, "armed (standing still, GPS %.0f Hz)", rate_hz);
            }
        } else {
            still_samples = 0;
        }
        break;

    case ST_ARMED:
        if (v >= V_LAUNCH && accel_ms2 > 0.0f) {
            state = ST_RUN;
            // interpolate the true standstill->launch crossing
            t_launch = itow_ms - dt;
            float f = (v != v_prev) ? (float)(V_LAUNCH - v_prev) / (float)(v - v_prev) : 0.0f;
            if (f > 0.0f && f <= 1.0f) t_launch += (uint32_t)(f * dt);

            // GNSS+IMU fusion: the accelerometer saw the car start moving
            // earlier than GNSS could - use that instant if it's plausible
            if (imu_launch_pitow != 0) {
                uint32_t ip = imu_launch_pitow;
                if (t_launch > ip && t_launch - ip < 800) {
                    ESP_LOGI(TAG, "launch refined by IMU: %lu ms earlier",
                             (unsigned long)(t_launch - ip));
                    t_launch = ip;
                } else {
                    ESP_LOGI(TAG, "IMU launch mark ignored (out of window)");
                }
                imu_launch_pitow = 0;
            }

            dist_mm = 0.0f;
            results.t_0_60 = results.t_0_100 = results.t_0_200 = -1.0f;
            results.t_100_200 = results.t_402m = results.v_402m_kmh = -1.0f;
            results.gap_s = 0.0f;
            results.run_active = true;
            seq++;
            ESP_LOGI(TAG, "launch!");
        }
        break;

    case ST_RUN: {
        // distance: trapezoidal integration, mm/s * ms / 1000 = mm
        float d_step = (float)(v + v_prev) * 0.5f * (float)dt / 1000.0f;
        float dist_before = dist_mm;
        dist_mm += d_step;

        if (results.t_0_60 < 0 && v >= V_60_KMH) {
            results.t_0_60 = cross_time_ms(itow_ms, dt, v, V_60_KMH) / 1000.0f;
            seq++;
            ESP_LOGI(TAG, "0-60 km/h: %.2f s", results.t_0_60);
        }
        if (results.t_0_100 < 0 && v >= V_100_KMH) {
            results.t_0_100 = cross_time_ms(itow_ms, dt, v, V_100_KMH) / 1000.0f;
            seq++;
            ESP_LOGI(TAG, "0-100 km/h: %.2f s", results.t_0_100);
        }
        if (results.t_0_200 < 0 && v >= V_200_KMH) {
            results.t_0_200 = cross_time_ms(itow_ms, dt, v, V_200_KMH) / 1000.0f;
            if (results.t_0_100 >= 0) {
                results.t_100_200 = results.t_0_200 - results.t_0_100;
            }
            seq++;
            ESP_LOGI(TAG, "0-200 km/h: %.2f s (100-200: %.2f s)", results.t_0_200, results.t_100_200);
        }
        if (results.t_402m < 0 && dist_mm >= DIST_QUARTER) {
            float f = (d_step > 0.0f) ? (DIST_QUARTER - dist_before) / d_step : 1.0f;
            results.t_402m = ((float)(itow_ms - dt - t_launch) + f * (float)dt) / 1000.0f;
            // trap speed at the interpolated crossing instant, not at the
            // (up to one sample later) detection sample
            results.v_402m_kmh = ((float)v_prev + f * (float)(v - v_prev)) * 0.0036f;
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
    t_prev = itow_ms;
}
