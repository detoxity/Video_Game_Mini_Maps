#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "AXP2101_PMU.h"

#if defined(BOARD_LCD_1_85)
// =============================================================================
// LCD 1.85: no PMU. Power is a discrete soft-latch - GPIO7 held high keeps the
// board powered (the button only supplies power while pressed, so firmware
// must grab the latch at boot). Battery voltage is read on the GPIO1 ADC
// through a 200k/100k divider; the power button is on GPIO6 (high = pressed).
// =============================================================================
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "pwr185";

#define PWR_CONTROL_GPIO   GPIO_NUM_7      // BAT_Control: HIGH latches power on
#define PWR_KEY_GPIO       GPIO_NUM_6      // Key_BAT: LOW = pressed (3V3 pull-up)
// Battery voltage on GPIO8 = ADC1_CH7 (NOT GPIO1 - that's the touch I2C SDA
// on this touch variant; the wiki's "GPIO1" is for the non-touch board).
// Divider is 200k/100k, so battery = pin voltage x 3.
#define BAT_ADC_ENABLED    1
#define BAT_ADC_CHANNEL    ADC_CHANNEL_7   // GPIO8
#define BAT_DIVIDER        3               // 200k + 100k

static void (*unplug_cb)(void) = NULL;
static void (*powerkey_cb)(void) = NULL;
static volatile int batt_mv = -1;
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali = NULL;

void pmu_power_hold_early(void) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PWR_CONTROL_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(PWR_CONTROL_GPIO, 1);   // hold power on
}

bool pmu_vbus_present(void) {
    return false;   // no VBUS sense on this board - treat as battery
}

int pmu_battery_percent(void) {
    return -1;      // voltage only, no fuel gauge
}

int pmu_battery_voltage_mv(void) {
    return batt_mv;
}

void pmu_power_off(void) {
    ESP_LOGW(TAG, "powering off (release latch)");
    gpio_set_level(PWR_CONTROL_GPIO, 0);   // drop the latch -> rails fall
    vTaskDelay(pdMS_TO_TICKS(1000));       // (only cuts on battery, not USB)
}

void pmu_set_unplug_callback(void (*cb)(void)) { unplug_cb = cb; }
void pmu_set_powerkey_callback(void (*cb)(void)) { powerkey_cb = cb; }

static void refresh_battery_voltage(void) {
    if (!adc_handle) return;
    int sum = 0, n = 0;
    for (int i = 0; i < 8; i++) {
        int raw = 0;
        if (adc_oneshot_read(adc_handle, BAT_ADC_CHANNEL, &raw) == ESP_OK) {
            sum += raw; n++;
        }
    }
    if (n == 0) return;
    int raw = sum / n;

    int pin_mv;
    if (adc_cali) {
        // calibrated raw->mV (handles the real ADC width/attenuation), then
        // undo the 200k/100k divider to get the battery voltage
        if (adc_cali_raw_to_voltage(adc_cali, raw, &pin_mv) != ESP_OK) return;
    } else {
        pin_mv = (int)((float)raw * 3.3f / 4096.0f * 1000.0f);   // rough fallback
    }
    int mv = pin_mv * BAT_DIVIDER;
    batt_mv = (mv > 1000) ? mv : -1;   // <1V = no battery attached

    // diagnostic (visible only if logs are enabled): raw near 4095 = the ADC
    // is saturating, which means the wrong pin or attenuation, not a scale bug
    static int dbg = 0;
    if (++dbg >= 30) {   // ~3s at 100ms
        dbg = 0;
        ESP_LOGI(TAG, "batt: raw=%d pin=%dmV batt=%dmV", raw, pin_mv, batt_mv);
    }
}

static void pmu_task(void *arg) {
    // Key_BAT reads HIGH when released, LOW when pressed. The press that
    // powers the board on holds it LOW through boot - wait for release
    // (HIGH) before acting, so booting isn't read as a power-off. After
    // that, a short click of the button saves state and powers off.
    bool armed = (gpio_get_level(PWR_KEY_GPIO) == 1);
    int low_count = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        refresh_battery_voltage();

        int key = gpio_get_level(PWR_KEY_GPIO);   // 0 = pressed
        if (!armed) {
            if (key == 1) armed = true;   // button released after boot
            continue;
        }
        // short click -> power off (debounced over ~200ms)
        if (key == 0) {
            if (++low_count >= 2) {
                low_count = 0;
                ESP_LOGI(TAG, "power key click");
                if (powerkey_cb) powerkey_cb();
            }
        } else {
            low_count = 0;
        }
    }
}

bool pmu_start(int i2c_port) {
    (void)i2c_port;
    (void)unplug_cb;   // no USB-unplug detection without a VBUS sense pin

#if BAT_ADC_ENABLED
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    if (adc_oneshot_new_unit(&init_cfg, &adc_handle) != ESP_OK) {
        adc_handle = NULL;
        ESP_LOGW(TAG, "battery ADC init failed");
    } else {
        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        adc_oneshot_config_channel(adc_handle, BAT_ADC_CHANNEL, &chan_cfg);

        // calibration: converts raw to true millivolts regardless of the
        // ADC's actual bit width / attenuation curve
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_1,
            .chan = BAT_ADC_CHANNEL,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali) != ESP_OK) {
            adc_cali = NULL;
            ESP_LOGW(TAG, "ADC calibration unavailable, using rough scale");
        }
    }
#endif

    gpio_config_t key_io = {
        .pin_bit_mask = 1ULL << PWR_KEY_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&key_io);

    ESP_LOGI(TAG, "discrete power: latch GPIO%d, key GPIO%d, batt ADC GPIO1",
             PWR_CONTROL_GPIO, PWR_KEY_GPIO);
    xTaskCreate(pmu_task, "pmu", 3072, NULL, 4, NULL);
    return true;
}

#else  // -------------------------------------------------------- AXP2101 (AMOLED)
#include "driver/i2c_master.h"

// hardware PMU latches power itself - nothing to hold from firmware
void pmu_power_hold_early(void) { }

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

#endif  // BOARD_LCD_1_85 / AXP2101
