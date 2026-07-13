// NMEA GPS input over UART (u-blox M10 class modules).
//
// Wiring (both supported boards expose the UART header on the back):
//   GPS TX  -> board RXD pad (GPIO44)
//   GPS RX  -> board TXD pad (GPIO43)   (optional, only for configuring)
//   GPS VCC -> 3V3, GPS GND -> GND
//
// Baud rate is auto-detected (9600 / 38400 / 115200).
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_UART_NUM       UART_NUM_1
#define GPS_UART_RX_GPIO   44
#define GPS_UART_TX_GPIO   43

// auto-configure the module at boot (RAM layer only - a GPS power cycle
// reverts everything). Requires the module's RX wired to GPS_UART_TX_GPIO.
// Both u-blox dialects are sent (legacy M8 + VALSET M10); the module ACKs
// what it supports. Boot config: 115200 baud, UBX NAV-PVT only, automotive
// dynamic model, AssistNow Autonomous, then CRUISE mode.
#define GPS_AUTO_CONFIG    1

// provided by main
extern float new_latitude;
extern float new_longitude;
extern float gps_speed_kmh;
extern int gps_sat_count;
extern volatile bool data_ready;
extern bool receiving_data;

// UTC date/time from NAV-PVT (owned by the driver; usually valid before
// the position fix). Same payload layout on M8 and M10.
extern volatile unsigned short gps_utc_year;    // e.g. 2026; 0 = no time yet
extern volatile unsigned char  gps_utc_month, gps_utc_day;
extern volatile unsigned char  gps_utc_hour, gps_utc_min, gps_utc_sec;
extern volatile bool           gps_time_valid;

// install the UART driver and start the NMEA reader task
void gps_uart_start(void);

// CRUISE (default): multi-GNSS (GPS+GAL+GLO+SBAS+QZSS) at 10Hz - best
//   time-to-first-fix, accuracy and urban reliability for the map.
// PERF (enable=true): GPS+QZSS only at the highest rate the module ACKs
//   (25Hz on M10, 18Hz on M8) - for the performance meter.
// Applied asynchronously by the config task; safe to call from any task.
void gps_set_perf_mode(bool enable);

#ifdef __cplusplus
}
#endif
