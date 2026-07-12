/*
 * Minimal BSP implementation for Waveshare ESP32-S3-Touch-LCD-1.85
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st77916.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_io_expander_tca9554.h"
#include "esp_lvgl_port.h"

#include "bsp/esp-bsp.h"

static const char *TAG = "bsp_ws_s3_185";

#define BSP_LCD_PIXEL_CLOCK_HZ     (50 * 1000 * 1000)   // ST77916 handles 50MHz QSPI
#define BSP_LCD_BACKLIGHT_LEDC_TIMER   LEDC_TIMER_1
#define BSP_LCD_BACKLIGHT_LEDC_CHANNEL LEDC_CHANNEL_1
#define BSP_LCD_BACKLIGHT_LEDC_DUTY_RES LEDC_TIMER_10_BIT

static i2c_master_bus_handle_t i2c_bus = NULL;
static esp_io_expander_handle_t io_expander = NULL;
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
static lv_display_t *lvgl_disp = NULL;
static esp_lcd_touch_handle_t tp = NULL;
static sdmmc_card_t *sd_card = NULL;

static esp_err_t bsp_i2c_init(void)
{
    if (i2c_bus) {
        return ESP_OK;
    }
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = BSP_I2C_NUM,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &i2c_bus), TAG, "I2C bus init failed");
    return ESP_OK;
}

static esp_err_t bsp_io_expander_init(void)
{
    if (io_expander) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "");
    ESP_RETURN_ON_ERROR(esp_io_expander_new_i2c_tca9554(i2c_bus,
                        ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &io_expander),
                        TAG, "TCA9554 init failed");

    /* Deassert everything: touch/LCD out of reset, SD CS high */
    esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 | IO_EXPANDER_PIN_NUM_2, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 1);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 1);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2, 1);
    return ESP_OK;
}

esp_err_t bsp_display_brightness_init(void)
{
    const ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BSP_LCD_BACKLIGHT_LEDC_DUTY_RES,
        .timer_num = BSP_LCD_BACKLIGHT_LEDC_TIMER,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "LEDC timer config failed");

    const ledc_channel_config_t channel_cfg = {
        .gpio_num = BSP_LCD_GPIO_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BSP_LCD_BACKLIGHT_LEDC_CHANNEL,
        .timer_sel = BSP_LCD_BACKLIGHT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_cfg), TAG, "LEDC channel config failed");
    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }
    const uint32_t max_duty = (1 << BSP_LCD_BACKLIGHT_LEDC_DUTY_RES) - 1;
    const uint32_t duty = (max_duty * brightness_percent) / 100;
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, BSP_LCD_BACKLIGHT_LEDC_CHANNEL, duty), TAG, "");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, BSP_LCD_BACKLIGHT_LEDC_CHANNEL), TAG, "");
    return ESP_OK;
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    (void)config;
    ESP_RETURN_ON_ERROR(bsp_io_expander_init(), TAG, "");
    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "");

    /* Hardware reset via IO expander (RST is active low) */
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_1, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    const spi_bus_config_t bus_cfg = ST77916_PANEL_BUS_QSPI_CONFIG(
        BSP_LCD_GPIO_SCLK,
        BSP_LCD_GPIO_D0, BSP_LCD_GPIO_D1, BSP_LCD_GPIO_D2, BSP_LCD_GPIO_D3,
        BSP_LCD_H_RES * 80 * sizeof(uint16_t));
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO), TAG, "SPI bus init failed");

    const esp_lcd_panel_io_spi_config_t io_cfg = ST77916_PANEL_IO_QSPI_CONFIG(BSP_LCD_GPIO_CS, NULL, NULL);
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_HOST, &io_cfg, &lcd_io), TAG, "LCD IO failed");

    st77916_vendor_config_t vendor_cfg = {
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = GPIO_NUM_NC,     // reset is wired to the IO expander
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
        .vendor_config = &vendor_cfg,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st77916(lcd_io, &panel_cfg, &lcd_panel), TAG, "ST77916 init failed");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(lcd_panel), TAG, "");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(lcd_panel), TAG, "");

    /* clear the panel RAM (powers up with random contents) before display-on */
    {
        const int strip_h = 16;
        uint16_t *strip = (uint16_t *)heap_caps_calloc(1, (size_t)BSP_LCD_H_RES * strip_h * 2, MALLOC_CAP_DMA);
        if (strip) {
            for (int y = 0; y < BSP_LCD_V_RES; y += strip_h) {
                int h = (y + strip_h > BSP_LCD_V_RES) ? BSP_LCD_V_RES - y : strip_h;
                esp_lcd_panel_draw_bitmap(lcd_panel, 0, y, BSP_LCD_H_RES, y + h, strip);
            }
            free(strip);
        }
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(lcd_panel, true), TAG, "");

    if (ret_panel) {
        *ret_panel = lcd_panel;
    }
    if (ret_io) {
        *ret_io = lcd_io;
    }
    return ESP_OK;
}

static esp_err_t bsp_touch_new(void)
{
    /* CST816 reset pulse via IO expander (active low) */
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0, 1);
    vTaskDelay(pdMS_TO_TICKS(80));

    i2c_master_bus_handle_t tp_bus;
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = BSP_TP_I2C_NUM,
        .sda_io_num = BSP_TP_GPIO_SDA,
        .scl_io_num = BSP_TP_GPIO_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &tp_bus), TAG, "touch I2C bus failed");

    esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    io_cfg.scl_speed_hz = 400000;
    esp_lcd_panel_io_handle_t tp_io;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(tp_bus, &io_cfg, &tp_io), TAG, "touch IO failed");

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,   // reset handled via IO expander above
        .int_gpio_num = BSP_TP_GPIO_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
    };
    return esp_lcd_touch_new_i2c_cst816s(tp_io, &tp_cfg, &tp);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);

    ESP_ERROR_CHECK(lvgl_port_init(&cfg->lvgl_port_cfg));

    ESP_ERROR_CHECK(bsp_display_new(NULL, NULL, NULL));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
            .sw_rotate = cfg->flags.sw_rotate,
            .swap_bytes = true,   // SPI LCD is big-endian RGB565
        },
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    if (bsp_touch_new() == ESP_OK) {
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lvgl_disp,
            .handle = tp,
        };
        lvgl_port_add_touch(&touch_cfg);
    } else {
        ESP_LOGW(TAG, "touch init failed - continuing without touch");
    }

    return lvgl_disp;
}

esp_err_t bsp_display_set_flip(bool flip)
{
    if (!lcd_panel) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = esp_lcd_panel_mirror(lcd_panel, flip, flip);
    if (err != ESP_OK) {
        return err;   // panel driver can't do it in hardware
    }
    if (tp) {
        esp_lcd_touch_set_mirror_x(tp, flip);
        esp_lcd_touch_set_mirror_y(tp, flip);
    }
    return ESP_OK;
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}

esp_err_t bsp_sdcard_mount(void)
{
    if (sd_card) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(bsp_io_expander_init(), TAG, "");

    /* SD over SPI. Chip-select sits on the IO expander, which the SDSPI
     * driver cannot toggle - the card is the only device on this bus, so
     * hold CS low for the whole session. Data clock kept at 10MHz: the
     * 20MHz default returned corrupted sector reads on this wiring. */
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2, 0);

    const spi_bus_config_t bus_cfg = {
        .sclk_io_num = BSP_SD_GPIO_SCLK,
        .mosi_io_num = BSP_SD_GPIO_MOSI,
        .miso_io_num = BSP_SD_GPIO_MISO,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 8192,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_SD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO), TAG, "SD SPI bus init failed");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = BSP_SD_SPI_HOST;
    host.max_freq_khz = 10000;

    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.host_id = BSP_SD_SPI_HOST;
    slot_cfg.gpio_cs = GPIO_NUM_NC;   // CS handled by the IO expander (held low)

    const esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    esp_err_t err = esp_vfs_fat_sdspi_mount(BSP_SD_MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &sd_card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s (note: card must be MBR-partitioned, FAT32 - GPT is not supported)", esp_err_to_name(err));
        esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_2, 1);
        spi_bus_free(BSP_SD_SPI_HOST);
        sd_card = NULL;
    }
    return err;
}

esp_err_t bsp_sdcard_unmount(void)
{
    if (!sd_card) {
        return ESP_OK;
    }
    esp_err_t err = esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, sd_card);
    sd_card = NULL;
    return err;
}
