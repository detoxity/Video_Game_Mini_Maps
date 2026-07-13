idf_component_register(
    SRCS "src/GPS_UART_Driver.c"
    INCLUDE_DIRS "include"
    REQUIRES driver PerfMeter TrackLog
)

