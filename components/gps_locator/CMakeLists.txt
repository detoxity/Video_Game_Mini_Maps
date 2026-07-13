idf_component_register(
    SRCS "gps_locator.cpp"
    INCLUDE_DIRS "include"
    REQUIRES lvgl esp_system TrackLog
    PRIV_REQUIRES vfs fatfs
)

