// RaceBox Mini BLE emulation.
//
// Advertises as a RaceBox Mini over the Nordic UART service and streams the
// documented 80-byte GNSS+IMU packet, so RaceChrono / Solostorm / the RaceBox
// app can log from this device. The payload is essentially a repacked UBX
// NAV-PVT (velocity NED dropped) with battery and IMU appended, so it is
// built straight from the frame the GPS driver already receives.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bring up the BLE stack and start advertising; call once at boot.
// `serial` is the 10-digit unit ID shown after the "RaceBox Mini " prefix.
// It must be below 4000000000 - the RaceBox app refuses higher IDs. Pass
// NULL to derive one from the MAC instead.
void racebox_ble_start(const char *serial);

// true while a client is connected and subscribed to notifications
bool racebox_ble_connected(void);

// gate the stream without dropping the connection (record-only mode)
void racebox_ble_set_streaming(bool on);

// Feed one UBX NAV-PVT payload (the 92-byte body, without the UBX header
// or checksum). Builds the RaceBox frame - pulling IMU and battery itself -
// and notifies the client. Cheap no-op when nothing is connected.
void racebox_ble_publish(const uint8_t *nav_pvt, uint16_t len);

#ifdef __cplusplus
}
#endif
