#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "QMI8658_IMU.h"
#include "PerfMeter.h"

static const char *TAG = "qmi8658";

#define REG_WHO_AM_I   0x00   // reads 0x05
#define REG_CTRL1      0x02   // bit6: register address auto-increment
#define REG_CTRL2      0x03   // accel full scale + ODR
#define REG_CTRL7      0x08   // bit0: accel enable
#define REG_AX_L       0x35   // AX_L..AZ_H, 6 bytes, int16 LE

#define ACCEL_CFG      0x25   // 8g range, 250Hz ODR
#define MPS2_PER_LSB   (8.0f * 9.80665f / 32768.0f)

static i2c_master_dev_handle_t dev = NULL;
static volatile bool running = false;
static volatile float last_a[3] = {0.0f, 0.0f, 0.0f};

static esp_err_t reg_read(uint8_t reg, uint8_t *out, size_t n) {
    return i2c_master_transmit_receive(dev, &reg, 1, out, n, 100);
}

static esp_err_t reg_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, 2, 100);
}

bool imu_available(void) {
    return running;
}

bool imu_get_accel(float *ax, float *ay, float *az) {
    if (!running) return false;
    *ax = last_a[0];
    *ay = last_a[1];
    *az = last_a[2];
    return true;
}

static void imu_task(void *arg) {
    running = true;
    // fixed 200Hz cadence: vTaskDelayUntil holds the period regardless of
    // how long the read/processing takes, so launch-detection sampling
    // stays regular. Processing is deliberately short - one 6-byte I2C read
    // then a feed - and the arrival timestamp is captured right at the read.
    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5);
    for (;;) {
        vTaskDelayUntil(&next, period);

        uint8_t d[6];
        if (reg_read(REG_AX_L, d, 6) != ESP_OK) {
            continue;
        }
        uint32_t tick_ms = (uint32_t)(esp_timer_get_time() / 1000);
        int16_t rx = (int16_t)(d[0] | (d[1] << 8));
        int16_t ry = (int16_t)(d[2] | (d[3] << 8));
        int16_t rz = (int16_t)(d[4] | (d[5] << 8));

        last_a[0] = rx * MPS2_PER_LSB;
        last_a[1] = ry * MPS2_PER_LSB;
        last_a[2] = rz * MPS2_PER_LSB;

        perf_imu_feed(last_a[0], last_a[1], last_a[2], tick_ms);
    }
}

bool imu_start(int i2c_port) {
    i2c_master_bus_handle_t bus = NULL;
    if (i2c_master_get_bus_handle(i2c_port, &bus) != ESP_OK || bus == NULL) {
        ESP_LOGW(TAG, "I2C bus %d not initialized", i2c_port);
        return false;
    }

    const uint8_t addrs[] = {0x6B, 0x6A};
    uint8_t found = 0;
    for (size_t i = 0; i < sizeof(addrs); i++) {
        if (i2c_master_probe(bus, addrs[i], 100) == ESP_OK) {
            found = addrs[i];
            break;
        }
    }
    if (!found) {
        ESP_LOGW(TAG, "QMI8658 not found (0x6B/0x6A)");
        return false;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = found,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) {
        return false;
    }

    uint8_t who = 0;
    if (reg_read(REG_WHO_AM_I, &who, 1) != ESP_OK || who != 0x05) {
        ESP_LOGW(TAG, "unexpected WHO_AM_I 0x%02X at 0x%02X", who, found);
        return false;
    }

    if (reg_write(REG_CTRL1, 0x40) != ESP_OK ||     // address auto-increment
        reg_write(REG_CTRL2, ACCEL_CFG) != ESP_OK ||
        reg_write(REG_CTRL7, 0x01) != ESP_OK) {     // accel on
        ESP_LOGW(TAG, "QMI8658 init failed");
        return false;
    }

    ESP_LOGI(TAG, "QMI8658 at 0x%02X, accel 8g @ 250Hz", found);
    // measurement core (see gps_uart_start): high priority (6, just under
    // the GPS reader) for deterministic launch-detection sampling, off the
    // core that runs rendering and tile streaming
    xTaskCreatePinnedToCore(imu_task, "imu", 3072, NULL, 6, NULL, 1);
    return true;
}
