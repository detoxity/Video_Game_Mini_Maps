idf_component_register(
    SRCS "src/TrackLog.c"
    INCLUDE_DIRS "include"
    REQUIRES PerfMeter
    PRIV_REQUIRES nvs_flash vfs fatfs
)
