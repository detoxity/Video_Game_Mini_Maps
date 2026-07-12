#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "QMC5883L_Compass.h"

static const char *TAG = "qmc5883l";

#define QMC5883L_ADDR        0x0D
#define REG_DATA             0x00   // x lsb, x msb, y lsb, y msb, z lsb, z msb
#define REG_STATUS           0x06   // bit0 = data ready
#define REG_CONTROL          0x09
#define REG_SET_RESET        0x0B

// OSR=512, range=8G, ODR=50Hz, continuous mode
#define CONTROL_VALUE        0x1D

static i2c_master_dev_handle_t dev = NULL;
static volatile bool have_data = false;
// smoothed heading as a unit vector (avoids the 359->0 wrap problem)
static volatile float head_sin = 0.0f;
static volatile float head_cos = 1.0f;

static esp_err_t reg_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, 2, 100);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *out, size_t n) {
    return i2c_master_transmit_receive(dev, &reg, 1, out, n, 100);
}

static void compass_task(void *arg) {
    const float alpha = 0.25f;   // smoothing factor
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));

        uint8_t status;
        if (reg_read(REG_STATUS, &status, 1) != ESP_OK || !(status & 0x01)) {
            continue;
        }
        uint8_t d[6];
        if (reg_read(REG_DATA, d, 6) != ESP_OK) {
            continue;
        }
        int16_t x = (int16_t)(d[0] | (d[1] << 8));
        int16_t y = (int16_t)(d[2] | (d[3] << 8));

        // heading in the horizontal plane (module mounted flat)
        float h = atan2f((float)y, (float)x) + COMPASS_DECLINATION_DEG * (float)M_PI / 180.0f;
        float s = sinf(h), c = cosf(h);
        head_sin = head_sin + alpha * (s - head_sin);
        head_cos = head_cos + alpha * (c - head_cos);
        have_data = true;
    }
}

bool compass_start(int i2c_port) {
    i2c_master_bus_handle_t bus = NULL;
    if (i2c_master_get_bus_handle(i2c_port, &bus) != ESP_OK || bus == NULL) {
        ESP_LOGW(TAG, "I2C bus %d not initialized", i2c_port);
        return false;
    }
    if (i2c_master_probe(bus, QMC5883L_ADDR, 100) != ESP_OK) {
        ESP_LOGW(TAG, "QMC5883L not found at 0x%02X", QMC5883L_ADDR);
        return false;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMC5883L_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) {
        return false;
    }

    if (reg_write(REG_SET_RESET, 0x01) != ESP_OK ||
        reg_write(REG_CONTROL, CONTROL_VALUE) != ESP_OK) {
        ESP_LOGW(TAG, "QMC5883L init failed");
        return false;
    }

    ESP_LOGI(TAG, "QMC5883L compass started");
    xTaskCreate(compass_task, "compass", 3072, NULL, 4, NULL);
    return true;
}

bool compass_available(void) {
    return have_data;
}

float compass_heading(void) {
    float deg = atan2f(head_sin, head_cos) * 180.0f / (float)M_PI;
    if (deg < 0) deg += 360.0f;
    return deg;
}
