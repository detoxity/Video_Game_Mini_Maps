// QMC5883L magnetometer (the compass on FlyfishRC M10 GPS modules and
// countless GY-271 style breakouts), read over the board's exposed I2C
// header: SCL = GPIO10, SDA = GPIO11 on the ESP32-S3-Touch-LCD-1.85.
// Address 0x0D - coexists with the on-board TCA9554/QMI8658/PCF85063.
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// magnetic declination applied to raw heading (Kyiv 2026: ~ +9 deg east)
#define COMPASS_DECLINATION_DEG  9.0f

// attach to an already-initialized I2C master bus (the BSP creates port 0)
// and start the background sampling task; returns false if the chip is absent
bool compass_start(int i2c_port);

// true once the compass is detected and producing samples
bool compass_available(void);

// smoothed heading in degrees 0..360 (0 = true north, clockwise)
float compass_heading(void);

#ifdef __cplusplus
}
#endif
