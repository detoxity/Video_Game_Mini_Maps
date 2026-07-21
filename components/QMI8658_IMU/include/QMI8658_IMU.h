// QMI8658 6-axis IMU (on-board on both Waveshare boards, shared I2C bus).
// Samples the accelerometer at ~250Hz and feeds the performance meter,
// which uses it to pin the launch instant between GNSS samples.
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// probe the IMU on an already-initialized I2C bus (tries 0x6B then 0x6A)
// and start the sampling task; returns false if absent
bool imu_start(int i2c_port);

// true once the IMU is detected and streaming
bool imu_available(void);

// latest accelerometer sample in m/s^2; returns false until streaming
bool imu_get_accel(float *ax, float *ay, float *az);

// latest gyroscope sample in degrees/second; false until streaming
bool imu_get_gyro(float *gx, float *gy, float *gz);

#ifdef __cplusplus
}
#endif
