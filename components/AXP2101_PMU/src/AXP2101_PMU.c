#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "AXP2101_PMU.h"

static const char *TAG = "axp2101";

#define AXP2101_ADDR        0x34
#define REG_PMU_STATUS1     0x00   // bit5: VBUS good
#define REG_COMMON_CONFIG   0x10   // bit0: soft power off
#define REG_ADC_ENABLE      0x30   // bit0: VBAT voltage measurement
#define REG_VBAT_H          0x34   // VBAT high 6 bits (low 8 in 0x35), 1mV/LSB
#define REG_IRQ_EN2         0x41   // bit3: power key short press IRQ enable
#define REG_IRQ_STATUS1     0x48   // IRQ statuses, write 1 to clear
#define REG_IRQ_STATUS2     0x49   // bit3: power key short press
#define REG_IRQ_STATUS3     0x4A
#define PKEY_SHORT_MASK     0x08
#define REG_BAT_PERCENT     0xA4   // state of charge, %

static i2c_master_dev_handle_t dev = NULL;
static void (*unplug_cb)(void) = NULL;
static void (*powerkey_cb)(void) = NULL;
static volatile bool vbus_now = false;
static volatile int batt_mv = -1;

static esp_err_t reg_read(uint8_t reg, uint8_t *out) {
    return i2c_master_transmit_receive(dev, &reg, 1, out, 1, 100);
}

static esp_err_t reg_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, 2, 100);
}

bool pmu_vbus_present(void) {
    return vbus_now;
}

int pmu_battery_percent(void) {
    uint8_t v;
    if (!dev || reg_read(REG_BAT_PERCENT, &v) != ESP_OK || v > 100) return -1;
    return v;
}

int pmu_battery_voltage_mv(void) {
    return batt_mv;
}

static void refresh_battery_voltage(void) {
    uint8_t h, l;
    if (reg_read(REG_VBAT_H, &h) == ESP_OK && reg_read(REG_VBAT_H + 1, &l) == ESP_OK) {
        int mv = ((h & 0x3F) << 8) | l;
        batt_mv = (mv > 1000) ? mv : -1;   // <1V = no battery attached
    }
}

void pmu_power_off(void) {
    if (!dev) return;
    ESP_LOGW(TAG, "powering off");
    uint8_t v = 0;
    reg_read(REG_COMMON_CONFIG, &v);
    reg_write(REG_COMMON_CONFIG, v | 0x01);
    vTaskDelay(pdMS_TO_TICKS(1000));   // rails drop here; not reached on USB
}

static bool read_vbus(void) {
    uint8_t status = 0;
    if (reg_read(REG_PMU_STATUS1, &status) != ESP_OK) return vbus_now;
    return (status & 0x20) != 0;
}

static void pmu_task(void *arg) {
    int absent_count = 0;
    vbus_now = read_vbus();
    ESP_LOGI(TAG, "monitoring VBUS (currently %s)", vbus_now ? "present" : "absent");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        refresh_battery_voltage();

        // power key short press (latched IRQ status, write-1-to-clear)
        uint8_t irq2 = 0;
        if (reg_read(REG_IRQ_STATUS2, &irq2) == ESP_OK && (irq2 & PKEY_SHORT_MASK)) {
            reg_write(REG_IRQ_STATUS2, PKEY_SHORT_MASK);
            ESP_LOGI(TAG, "power key short press");
            if (powerkey_cb) powerkey_cb();
        }

        bool present = read_vbus();

        if (present) {
            absent_count = 0;
            vbus_now = true;
        } else if (vbus_now) {
            // debounce: two consecutive absent reads = real unplug
            if (++absent_count >= 2) {
                vbus_now = false;
                ESP_LOGW(TAG, "USB power removed");
                if (unplug_cb) unplug_cb();
            }
        }
    }
}

void pmu_set_unplug_callback(void (*cb)(void)) {
    unplug_cb = cb;
}

void pmu_set_powerkey_callback(void (*cb)(void)) {
    powerkey_cb = cb;
}

bool pmu_start(int i2c_port) {
    i2c_master_bus_handle_t bus = NULL;
    if (i2c_master_get_bus_handle(i2c_port, &bus) != ESP_OK || bus == NULL) {
        ESP_LOGW(TAG, "I2C bus %d not initialized", i2c_port);
        return false;
    }
    if (i2c_master_probe(bus, AXP2101_ADDR, 100) != ESP_OK) {
        ESP_LOGW(TAG, "AXP2101 not found at 0x%02X", AXP2101_ADDR);
        return false;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP2101_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) {
        return false;
    }

    // enable VBAT voltage measurement
    uint8_t adc = 0;
    reg_read(REG_ADC_ENABLE, &adc);
    reg_write(REG_ADC_ENABLE, adc | 0x01);

    // clear any stale latched IRQs, enable the power key short press one
    reg_write(REG_IRQ_STATUS1, 0xFF);
    reg_write(REG_IRQ_STATUS2, 0xFF);
    reg_write(REG_IRQ_STATUS3, 0xFF);
    uint8_t en2 = 0;
    reg_read(REG_IRQ_EN2, &en2);
    reg_write(REG_IRQ_EN2, en2 | PKEY_SHORT_MASK);

    ESP_LOGI(TAG, "AXP2101 found, battery %d%%", pmu_battery_percent());
    xTaskCreate(pmu_task, "pmu", 3072, NULL, 4, NULL);
    return true;
}
