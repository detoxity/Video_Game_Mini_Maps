/*
 * Minimal BSP for Waveshare ESP32-S3-Touch-LCD-1.85
 *
 * Mirrors the subset of the esp-bsp API used by this project so the
 * application code stays board-agnostic:
 *   - bsp_display_start_with_config() / lock / unlock / backlight
 *   - bsp_sdcard_mount()
 *
 * Board wiring (from the Waveshare schematic):
 *   LCD  ST77916 QSPI: SCK=40, CS=21, D0=46, D1=45, D2=42, D3=41, BL=5 (PWM)
 *   TCA9554 @0x20 on I2C0 (SCL=10, SDA=11): P0=TP_RST, P1=LCD_RST, P2=SD_CS
 *   SD (SPI mode): SCLK=14, MOSI=17, MISO=16, CS via expander (held low)
 */
#pragma once

#include "sdkconfig.h"
#include "bsp/display.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

/* identifies this board for app-level #ifdefs */
#define BSP_BOARD_WS_S3_TOUCH_LCD_1_85 (1)

/* I2C bus shared by TCA9554, QMI8658, PCF85063 */
#define BSP_I2C_NUM             (0)
#define BSP_I2C_SCL             (GPIO_NUM_10)
#define BSP_I2C_SDA             (GPIO_NUM_11)

/* LCD QSPI pins */
#define BSP_LCD_SPI_HOST        (SPI2_HOST)
#define BSP_LCD_GPIO_SCLK       (GPIO_NUM_40)
#define BSP_LCD_GPIO_CS         (GPIO_NUM_21)
#define BSP_LCD_GPIO_D0         (GPIO_NUM_46)
#define BSP_LCD_GPIO_D1         (GPIO_NUM_45)
#define BSP_LCD_GPIO_D2         (GPIO_NUM_42)
#define BSP_LCD_GPIO_D3         (GPIO_NUM_41)
#define BSP_LCD_GPIO_BL         (GPIO_NUM_5)

/* TCA9554 expander pins */
#define BSP_EXIO_TP_RST         (0)    /* P0 */
#define BSP_EXIO_LCD_RST        (1)    /* P1 */
#define BSP_EXIO_SD_CS          (2)    /* P2 */

/* Touch (CST816) on its own I2C bus */
#define BSP_TP_I2C_NUM          (1)
#define BSP_TP_GPIO_SDA         (GPIO_NUM_1)
#define BSP_TP_GPIO_SCL         (GPIO_NUM_3)
#define BSP_TP_GPIO_INT         (GPIO_NUM_4)

/* SD card (SPI mode) */
#define BSP_SD_SPI_HOST         (SPI3_HOST)
#define BSP_SD_GPIO_SCLK        (GPIO_NUM_14)
#define BSP_SD_GPIO_MOSI        (GPIO_NUM_17)
#define BSP_SD_GPIO_MISO        (GPIO_NUM_16)
#define BSP_SD_MOUNT_POINT      "/sdcard"

/* LVGL draw buffer: two 100-line buffers so LVGL renders while DMA flushes */
#define BSP_LCD_DRAW_BUFF_SIZE     (BSP_LCD_H_RES * 100) // Frame buffer size in pixels
#define BSP_LCD_DRAW_BUFF_DOUBLE   (1)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSP display configuration structure
 */
typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;  /*!< LVGL port configuration */
    uint32_t        buffer_size;    /*!< Size of the buffer for the screen in pixels */
    bool            double_buffer;  /*!< True, if should be allocated two buffers */
    struct {
        unsigned int buff_dma: 1;    /*!< Allocated LVGL buffer will be DMA capable */
        unsigned int buff_spiram: 1; /*!< Allocated LVGL buffer will be in PSRAM */
        unsigned int sw_rotate: 1;   /*!< Use software rotation (slower) */
    } flags;
} bsp_display_cfg_t;

/**
 * @brief Initialize display, backlight and start LVGL
 */
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);

/**
 * @brief Take LVGL mutex. timeout_ms=0 waits forever.
 */
bool bsp_display_lock(uint32_t timeout_ms);

/**
 * @brief Give LVGL mutex.
 */
void bsp_display_unlock(void);

/**
 * @brief Flip the display 180 degrees in hardware (MADCTL mirror, zero
 *        render cost). Touch coordinates are mirrored accordingly.
 *        Returns an error if the panel driver doesn't support it.
 */
esp_err_t bsp_display_set_flip(bool flip);

/**
 * @brief Mount microSD card to virtual file system (BSP_SD_MOUNT_POINT)
 */
esp_err_t bsp_sdcard_mount(void);

/**
 * @brief Unmount microSD card
 */
esp_err_t bsp_sdcard_unmount(void);

#ifdef __cplusplus
}
#endif
