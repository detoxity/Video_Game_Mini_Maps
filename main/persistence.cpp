#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"

#include "app_config.h"
#include "app_state.h"
#include "persistence.h"
#include "ui_overlays.h"

#if USE_PMU
#include "AXP2101_PMU.h"
#endif

void persistence_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    float lat = HOME_LAT, lon = HOME_LON;
    nvs_handle_t h;
    if (nvs_open("minimap", NVS_READONLY, &h) == ESP_OK) {
        uint32_t v;
        if (nvs_get_u32(h, "lat", &v) == ESP_OK) memcpy(&lat, &v, sizeof(v));
        if (nvs_get_u32(h, "lon", &v) == ESP_OK) memcpy(&lon, &v, sizeof(v));
        nvs_close(h);
    }
    current_latitude = new_latitude = lat;
    current_longitude = new_longitude = lon;
    printf("Starting at last known position: %f, %f\n", lat, lon);
}

void save_last_position(void) {
#if SAVE_LAST_POSITION
    static uint32_t last_save = 0;
    if (last_save != 0 && lv_tick_elaps(last_save) < 60000) return;   // spare the flash
    last_save = lv_tick_get();
    save_position_now();
#endif
}

void save_position_now(void) {
    nvs_handle_t h;
    if (nvs_open("minimap", NVS_READWRITE, &h) == ESP_OK) {
        uint32_t v;
        memcpy(&v, &new_latitude, sizeof(v));
        nvs_set_u32(h, "lat", v);
        memcpy(&v, &new_longitude, sizeof(v));
        nvs_set_u32(h, "lon", v);
        nvs_set_u8(h, "batt_show", overlays_battery_visible() ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
        printf("Position saved: %f, %f\n", new_latitude, new_longitude);
    }
}

#if USE_PMU
// full-screen "Saving..." overlay shown while we persist and power down
static void save_and_power_off(void) {
    bsp_display_lock(0);
    lv_obj_t *overlay = lv_obj_create(lv_screen_active());
    lv_obj_set_size(overlay, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_center(overlay);
    lv_obj_set_style_bg_color(overlay, PALETTE_BLACK, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(overlay, 0, 0);
    lv_obj_set_style_radius(overlay, 0, 0);

    lv_obj_t *lbl = lv_label_create(overlay);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lbl, PALETTE_WHITE, 0);
    lv_label_set_text(lbl, "Saving...");
    lv_obj_center(lbl);

    lv_refr_now(NULL);
    bsp_display_unlock();

    save_position_now();
    vTaskDelay(pdMS_TO_TICKS(600));   // let the message be seen
    pmu_power_off();
}

// USB unplugged: persist the position, then cut power
static void on_usb_unplugged(void) {
    save_and_power_off();
}

// PWR short press: power off manually - but only on battery
// (with USB present the PMU would immediately re-power anyway)
static void on_power_button(void) {
    if (!pmu_vbus_present()) {
        save_and_power_off();
    }
}
#endif

void persistence_pmu_init(void) {
#if USE_PMU
    if (pmu_start(BSP_I2C_NUM)) {
        pmu_set_unplug_callback(on_usb_unplugged);
        pmu_set_powerkey_callback(on_power_button);
    }
#endif
}
