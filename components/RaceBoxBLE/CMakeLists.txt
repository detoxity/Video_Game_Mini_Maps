idf_component_register(
    SRCS "src/RaceBoxBLE.c"
    INCLUDE_DIRS "include"
    REQUIRES bt QMI8658_IMU AXP2101_PMU
)
