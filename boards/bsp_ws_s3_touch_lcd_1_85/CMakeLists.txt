idf_component_register(
    SRCS "src/bsp_ws_s3_touch_lcd_1_85.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_lcd fatfs sdmmc
)
