#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "GPS_UART_Driver.h"
#include "PerfMeter.h"

static const char *TAG = "gps_uart";

// at 18-25Hz UBX the old 2KB buffer held ~1s of stream - any longer stall
// overflowed it and the resulting gap silently shortened the integrated
// 402m distance. 8KB rides out multi-second hiccups.
#define RX_BUF_SIZE   8192
#define LINE_MAX      128

static const int baud_candidates[] = {38400, 9600, 115200, 57600, 19200};

// ACK(1)/NAK(0)/no answer(-1) for config commands we care about
static volatile int8_t ack_cfg_rate = -1;
static volatile int8_t ack_cfg_gnss = -1;

// UTC date/time from NAV-PVT (exported; see header)
volatile unsigned short gps_utc_year = 0;
volatile unsigned char  gps_utc_month = 0, gps_utc_day = 0;
volatile unsigned char  gps_utc_hour = 0, gps_utc_min = 0, gps_utc_sec = 0;
volatile bool           gps_time_valid = false;

typedef enum {
    PROBE_SILENT,   // nothing received at all
    PROBE_GARBAGE,  // bytes came in but neither NMEA nor UBX (wrong baud)
    PROBE_NMEA,     // NMEA sentences seen
    PROBE_UBX,      // u-blox binary protocol seen (module configured UBX-only)
} probe_result_t;

// frame and send a UBX message (adds sync chars and checksum)
static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
    uint8_t hdr[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++) { ck_a += hdr[i]; ck_b += ck_a; }
    for (int i = 0; i < len; i++) { ck_a += payload[i]; ck_b += ck_a; }
    uint8_t ck[2] = {ck_a, ck_b};
    uart_write_bytes(GPS_UART_NUM, hdr, 6);
    uart_write_bytes(GPS_UART_NUM, payload, len);
    uart_write_bytes(GPS_UART_NUM, ck, 2);
}

// UBX-CFG-VALSET: set CFG-UART1OUTPROT-NMEA = 1 in RAM+BBR layers
// (recovers modules that a flight controller switched to UBX-only output)
static void ubx_enable_nmea_output(void) {
    const uint8_t payload[] = {
        0x00, 0x03, 0x00, 0x00, // version, layers (RAM|BBR), reserved
        0x02, 0x00, 0x74, 0x10, // key CFG-UART1OUTPROT-NMEA (0x10740002 LE)
        0x01,                   // value: enabled
    };
    ubx_send(0x06, 0x8A, payload, sizeof(payload));
}

#if GPS_AUTO_CONFIG
// -----------------------------------------------------------------------------
// Module auto-configuration. Everything is sent in BOTH u-blox dialects -
// legacy (M8 generation: CFG-MSG/RATE/PRT/GNSS/NAV5/NAVX5) and VALSET
// (M10 generation) - the module ACKs the dialect it supports and NAKs the
// other. All writes go to the RAM layer: a module power cycle restores
// factory state, so the device can never brick its GPS.
// -----------------------------------------------------------------------------

// send one VALSET (RAM layer) with the given key/value items
static void valset_send(const uint8_t *items, uint16_t items_len) {
    uint8_t payload[4 + 64];
    if (items_len > sizeof(payload) - 4) return;
    payload[0] = 0x00;          // version
    payload[1] = 0x01;          // layer: RAM
    payload[2] = 0x00;
    payload[3] = 0x00;
    memcpy(&payload[4], items, items_len);
    ubx_send(0x06, 0x8A, payload, 4 + items_len);
}

// stage 1, at the module's current baud: NAV-PVT output on, UBX-only
// protocol, then move the link to 115200 (both dialects) and follow.
static void ubx_config_link(void) {
    // legacy: enable NAV-PVT on the current port
    const uint8_t msg_pvt[] = {0x01, 0x07, 0x01};
    ubx_send(0x06, 0x01, msg_pvt, sizeof(msg_pvt));
    vTaskDelay(pdMS_TO_TICKS(50));

    // VALSET: UBX out on / NMEA out off, NAV-PVT on UART1
    const uint8_t vs_prot[] = {
        0x01, 0x00, 0x74, 0x10, 0x01,       // CFG-UART1OUTPROT-UBX  = 1
        0x02, 0x00, 0x74, 0x10, 0x00,       // CFG-UART1OUTPROT-NMEA = 0
        0x07, 0x00, 0x91, 0x20, 0x01,       // CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1
    };
    valset_send(vs_prot, sizeof(vs_prot));
    vTaskDelay(pdMS_TO_TICKS(50));

    // legacy CFG-PRT: UART1, 8N1, 115200, in UBX+NMEA+RTCM, out UBX only
    const uint8_t prt[] = {
        0x01, 0x00, 0x00, 0x00,             // portID 1, reserved, txReady
        0xD0, 0x08, 0x00, 0x00,             // mode: 8N1
        0x00, 0xC2, 0x01, 0x00,             // baud 115200
        0x07, 0x00, 0x01, 0x00,             // inProtoMask, outProtoMask (UBX)
        0x00, 0x00, 0x00, 0x00,             // flags, reserved
    };
    ubx_send(0x06, 0x00, prt, sizeof(prt));
    vTaskDelay(pdMS_TO_TICKS(100));

    // VALSET: CFG-UART1-BAUDRATE = 115200 (garbled on a module that
    // already switched via CFG-PRT - harmless, checksum discards it)
    const uint8_t vs_baud[] = {
        0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00,
    };
    valset_send(vs_baud, sizeof(vs_baud));
    vTaskDelay(pdMS_TO_TICKS(200));

    uart_set_baudrate(GPS_UART_NUM, 115200);
    uart_flush_input(GPS_UART_NUM);
    ESP_LOGI(TAG, "link configured, following to 115200 baud");
}

// navigation tuning that applies in every mode (run once, ACKs visible):
// automotive dynamic model + AssistNow Autonomous (self-predicted
// ephemeris - much faster warm starts, key when backup power is weak)
static void ubx_config_navigation(void) {
    // legacy CFG-NAV5: apply dynModel only (mask bit0), model 4 = automotive
    uint8_t nav5[36] = {0};
    nav5[0] = 0x01; nav5[1] = 0x00;     // parameter mask: dyn model
    nav5[2] = 0x04;                     // automotive
    ubx_send(0x06, 0x24, nav5, sizeof(nav5));
    vTaskDelay(pdMS_TO_TICKS(150));

    // legacy CFG-NAVX5: enable AssistNow Autonomous (mask1 bit14, aopCfg=1)
    uint8_t navx5[40] = {0};
    navx5[0] = 0x02; navx5[1] = 0x00;   // message version 2
    navx5[2] = 0x00; navx5[3] = 0x40;   // mask1: aop bit
    navx5[27] = 0x01;                   // aopCfg: enabled
    ubx_send(0x06, 0x23, navx5, sizeof(navx5));
    vTaskDelay(pdMS_TO_TICKS(150));

    // VALSET equivalents (split: one unknown key must not kill the other)
    const uint8_t vs_dyn[] = {0x21, 0x00, 0x11, 0x20, 0x04};   // CFG-NAVSPG-DYNMODEL = automotive
    valset_send(vs_dyn, sizeof(vs_dyn));
    vTaskDelay(pdMS_TO_TICKS(100));
    const uint8_t vs_ana[] = {0x01, 0x00, 0x23, 0x10, 0x01};   // CFG-ANA-USE_ANA = 1
    valset_send(vs_ana, sizeof(vs_ana));
    vTaskDelay(pdMS_TO_TICKS(100));
}

// constellations + nav rate for one of the two operating modes
static void ubx_config_mode(bool perf) {
    if (perf) {
        // GPS + QZSS only (u-blox: keep QZSS with GPS) - allows max rate
        const uint8_t gnss[] = {
            0x00, 0x00, 0xFF, 0x07,
            0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,  // GPS     on
            0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01,  // SBAS    off
            0x02, 0x04, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01,  // Galileo off
            0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01,  // BeiDou  off
            0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x03, 0x01,  // IMES    off
            0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x05, 0x01,  // QZSS    on
            0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x01, 0x01,  // GLONASS off
        };
        ubx_send(0x06, 0x3E, gnss, sizeof(gnss));
        const uint8_t vs_sig[] = {
            0x1F, 0x00, 0x31, 0x10, 0x01,       // GPS  on
            0x20, 0x00, 0x31, 0x10, 0x00,       // SBAS off
            0x21, 0x00, 0x31, 0x10, 0x00,       // GAL  off
            0x22, 0x00, 0x31, 0x10, 0x00,       // BDS  off
            0x24, 0x00, 0x31, 0x10, 0x01,       // QZSS on
            0x25, 0x00, 0x31, 0x10, 0x00,       // GLO  off
        };
        valset_send(vs_sig, sizeof(vs_sig));
    } else {
        // cruise: everything on except BeiDou/IMES (M8 handles 3 concurrent
        // + SBAS/QZSS augmentation; best TTFF and urban reliability)
        const uint8_t gnss[] = {
            0x00, 0x00, 0xFF, 0x07,
            0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,  // GPS     on
            0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,  // SBAS    on
            0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,  // Galileo on
            0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01,  // BeiDou  off
            0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x03, 0x01,  // IMES    off
            0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x05, 0x01,  // QZSS    on
            0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01,  // GLONASS on
        };
        ubx_send(0x06, 0x3E, gnss, sizeof(gnss));
        const uint8_t vs_sig[] = {
            0x1F, 0x00, 0x31, 0x10, 0x01,       // GPS  on
            0x20, 0x00, 0x31, 0x10, 0x01,       // SBAS on
            0x21, 0x00, 0x31, 0x10, 0x01,       // GAL  on
            0x22, 0x00, 0x31, 0x10, 0x00,       // BDS  off
            0x24, 0x00, 0x31, 0x10, 0x01,       // QZSS on
            0x25, 0x00, 0x31, 0x10, 0x01,       // GLO  on
        };
        valset_send(vs_sig, sizeof(vs_sig));
    }
    vTaskDelay(pdMS_TO_TICKS(500));   // GNSS engine restarts after this

    // nav rate: ladder of legacy CFG-RATE requests, first ACK wins
    // (perf: 25/18/10Hz, cruise: 10/5Hz); VALSET fallback for M10
    const uint16_t perf_rates[]   = {40, 55, 100};
    const uint16_t cruise_rates[] = {100, 200};
    const uint16_t *rates = perf ? perf_rates : cruise_rates;
    size_t n_rates = perf ? 3 : 2;

    for (size_t i = 0; i < n_rates; i++) {
        uint8_t rate[] = {(uint8_t)(rates[i] & 0xFF), (uint8_t)(rates[i] >> 8),
                          0x01, 0x00, 0x01, 0x00};
        ack_cfg_rate = -1;
        ubx_send(0x06, 0x08, rate, sizeof(rate));
        for (int w = 0; w < 20 && ack_cfg_rate == -1; w++) vTaskDelay(pdMS_TO_TICKS(25));
        if (ack_cfg_rate == 1) {
            ESP_LOGI(TAG, "%s: nav rate %u ms accepted", perf ? "PERF" : "CRUISE", rates[i]);
            break;
        }
    }
    const uint8_t vs_rate[] = {0x01, 0x00, 0x21, 0x30,
                               (uint8_t)(rates[0] & 0xFF), (uint8_t)(rates[0] >> 8)};
    valset_send(vs_rate, sizeof(vs_rate));
}

// desired vs applied operating mode (set from any task, applied here)
static volatile bool perf_mode_requested = false;

void gps_set_perf_mode(bool enable) {
    perf_mode_requested = enable;
}

// persistent config worker: runs outside the reader task so the reader
// can parse the ACK/NAK answers the config waits for
static void gps_cfg_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(1500));   // let the 115200 stream settle

    ubx_config_navigation();
    bool applied = perf_mode_requested;
    ubx_config_mode(applied);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        if (perf_mode_requested != applied) {
            applied = perf_mode_requested;
            ESP_LOGI(TAG, "switching to %s mode", applied ? "PERF" : "CRUISE");
            ubx_config_mode(applied);
        }
    }
}
#endif // GPS_AUTO_CONFIG

// verify "$....*hh" checksum
static bool nmea_checksum_ok(const char *line) {
    const char *star = strrchr(line, '*');
    if (line[0] != '$' || !star || star - line < 2) return false;
    uint8_t sum = 0;
    for (const char *p = line + 1; p < star; p++) sum ^= (uint8_t)*p;
    return sum == (uint8_t)strtol(star + 1, NULL, 16);
}

// "ddmm.mmmm"/"dddmm.mmmm" + hemisphere -> decimal degrees
static double nmea_coord(const char *field, const char *hemi) {
    double v = atof(field);
    int deg = (int)(v / 100.0);
    double d = deg + (v - deg * 100.0) / 60.0;
    if (hemi[0] == 'S' || hemi[0] == 'W') d = -d;
    return d;
}

// split on commas in place; returns field count (empty fields preserved)
static int split_fields(char *line, char *fields[], int max_fields) {
    int n = 0;
    char *p = line;
    while (n < max_fields) {
        fields[n++] = p;
        p = strchr(p, ',');
        if (!p) break;
        *p++ = '\0';
    }
    return n;
}

static volatile uint32_t last_ok_tick = 0;   // last valid NMEA line or UBX frame

static void handle_line(char *line) {
    if (!nmea_checksum_ok(line)) return;
    last_ok_tick = xTaskGetTickCount();

    // cut the checksum off so it doesn't stick to the last field
    char *star = strrchr(line, '*');
    if (star) *star = '\0';

    char *f[16];
    int n = split_fields(line, f, 16);
    if (n < 7) return;

    const char *type = f[0] + 3;   // skip "$GN"/"$GP"
    double lat = 0, lon = 0;
    bool fix = false;

    if (strncmp(type, "RMC", 3) == 0) {
        if (f[2][0] != 'A') return;            // V = no fix yet
        lat = nmea_coord(f[3], f[4]);
        lon = nmea_coord(f[5], f[6]);
        if (n > 7 && f[7][0] != '\0') {
            gps_speed_kmh = atof(f[7]) * 1.852f;   // knots -> km/h
        }
        fix = true;
    } else if (strncmp(type, "GGA", 3) == 0) {
        if (n > 7) gps_sat_count = atoi(f[7]);
        if (atoi(f[6]) == 0) return;           // fix quality 0 = invalid
        lat = nmea_coord(f[2], f[3]);
        lon = nmea_coord(f[4], f[5]);
        fix = true;
    }

    if (fix && lat != 0.0 && lon != 0.0) {
        new_latitude = (float)lat;
        new_longitude = (float)lon;
        receiving_data = true;
        data_ready = true;
    }
}

// ---------------------------------------------------------------- UBX
// Minimal UBX parser: extracts position from NAV-PVT (class 0x01, id 0x07),
// which every flight-controller-configured module streams by default.
static struct {
    int state;          // 0 sync1, 1 sync2, 2 class, 3 id, 4 len1, 5 len2, 6 payload, 7 ck_a, 8 ck_b
    uint8_t cls, id;
    uint16_t len, pos;
    uint8_t payload[100];
    uint8_t ck_a, ck_b;
} ubx;

static void ubx_handle_frame(void) {
    last_ok_tick = xTaskGetTickCount();

    // config command responses: ACK (0x05/0x01) or NAK (0x05/0x00)
    if (ubx.cls == 0x05 && ubx.len >= 2) {
        ESP_LOGI(TAG, "%s for cmd 0x%02X/0x%02X",
                 ubx.id == 0x01 ? "ACK" : "NAK", ubx.payload[0], ubx.payload[1]);
        if (ubx.payload[0] == 0x06) {
            if (ubx.payload[1] == 0x08) ack_cfg_rate = (ubx.id == 0x01);
            if (ubx.payload[1] == 0x3E) ack_cfg_gnss = (ubx.id == 0x01);
        }
        return;
    }

    // diagnostics: log each distinct message type once
    static uint16_t seen[12];
    static int nseen = 0;
    uint16_t key = ((uint16_t)ubx.cls << 8) | ubx.id;
    bool known = false;
    for (int i = 0; i < nseen; i++) {
        if (seen[i] == key) { known = true; break; }
    }
    if (!known && nseen < 12) {
        seen[nseen++] = key;
        ESP_LOGI(TAG, "UBX msg class=0x%02X id=0x%02X len=%u", ubx.cls, ubx.id, ubx.len);
    }

    if (ubx.cls != 0x01 || ubx.id != 0x07 || ubx.len < 36) return;   // NAV-PVT only

    // measured NAV-PVT rate (works without a fix); logged when it changes
    {
        static uint32_t win_start = 0;
        static int win_count = 0;
        static float logged_hz = 0.0f;
        uint32_t now = xTaskGetTickCount();
        if (win_start == 0) win_start = now;
        win_count++;
        if (now - win_start >= pdMS_TO_TICKS(10000)) {
            float hz = win_count * 1000.0f / (float)((now - win_start) * portTICK_PERIOD_MS);
            if (logged_hz == 0.0f || hz > logged_hz * 1.2f || hz < logged_hz * 0.8f) {
                ESP_LOGI(TAG, "NAV-PVT stream: %.1f Hz", hz);
                logged_hz = hz;
            }
            win_start = now;
            win_count = 0;
        }
    }

    uint8_t fix_type = ubx.payload[20];
    uint8_t flags = ubx.payload[21];
    uint8_t num_sv = ubx.payload[23];
    gps_sat_count = num_sv;

    // UTC date/time: year u2 @4, month @6, day @7, h/m/s @8-10; valid
    // flags @11 (bit0 date, bit1 time). Time usually locks before the
    // position fix, so grab it here rather than after the fix gates.
    if ((ubx.payload[11] & 0x03) == 0x03) {
        gps_utc_year  = (unsigned short)(ubx.payload[4] | (ubx.payload[5] << 8));
        gps_utc_month = ubx.payload[6];
        gps_utc_day   = ubx.payload[7];
        gps_utc_hour  = ubx.payload[8];
        gps_utc_min   = ubx.payload[9];
        gps_utc_sec   = ubx.payload[10];
        gps_time_valid = true;
    }

    // progress log every ~5s until the first valid fix
    static bool fix_logged = false;
    static uint32_t last_log = 0;
    uint32_t now = xTaskGetTickCount();
    if (!fix_logged && now - last_log > pdMS_TO_TICKS(5000)) {
        last_log = now;
        ESP_LOGI(TAG, "searching: fixType=%d satellites=%d", fix_type, num_sv);
    }
    if (!fix_logged && fix_type >= 2) {
        fix_logged = true;
        ESP_LOGI(TAG, "first fix: type=%d satellites=%d", fix_type, num_sv);
    }
    if (fix_type < 2 || !(flags & 0x01)) return;   // need a valid 2D/3D fix

    // velocity block: iTOW + NED velocity + Doppler ground speed.
    // fed to the performance meter on every valid fix, before the
    // position accuracy gate (timing needs the full sample stream)
    if (ubx.len >= 92) {
        uint32_t itow;
        int32_t vel_n, vel_e, vel_d, gspeed_mms;
        memcpy(&itow, &ubx.payload[0], 4);
        memcpy(&vel_n, &ubx.payload[48], 4);
        memcpy(&vel_e, &ubx.payload[52], 4);
        memcpy(&vel_d, &ubx.payload[56], 4);
        memcpy(&gspeed_mms, &ubx.payload[60], 4);
        (void)vel_n; (void)vel_e; (void)vel_d;   // parsed for future use (slope etc.)
        perf_feed(itow, gspeed_mms);
    }

    // cold-start fixes can scatter wildly before settling - only pass
    // positions the module itself rates better than 50m horizontal accuracy
    if (ubx.len >= 44) {
        uint32_t h_acc_mm;
        memcpy(&h_acc_mm, &ubx.payload[40], 4);
        if (h_acc_mm > 50000) {
            static uint32_t acc_log = 0;
            uint32_t now = xTaskGetTickCount();
            if (now - acc_log > pdMS_TO_TICKS(5000)) {
                acc_log = now;
                ESP_LOGI(TAG, "fix acquired, waiting for accuracy (%lum)", (unsigned long)(h_acc_mm / 1000));
            }
            return;
        }
    }

    int32_t lon_e7, lat_e7;
    memcpy(&lon_e7, &ubx.payload[24], 4);
    memcpy(&lat_e7, &ubx.payload[28], 4);
    if (lat_e7 == 0 && lon_e7 == 0) return;   // null island = not a real fix

    if (ubx.len >= 64) {
        int32_t gspeed_mms;             // ground speed, mm/s
        memcpy(&gspeed_mms, &ubx.payload[60], 4);
        gps_speed_kmh = gspeed_mms * 0.0036f;
    }

    new_latitude = (float)(lat_e7 * 1e-7);
    new_longitude = (float)(lon_e7 * 1e-7);
    receiving_data = true;
    data_ready = true;
}

static void ubx_parse_byte(uint8_t b) {
    switch (ubx.state) {
    case 0: if (b == 0xB5) ubx.state = 1; break;
    case 1: ubx.state = (b == 0x62) ? 2 : 0; break;
    case 2: ubx.cls = b; ubx.ck_a = b; ubx.ck_b = b; ubx.state = 3; break;
    case 3: ubx.id = b; ubx.ck_a += b; ubx.ck_b += ubx.ck_a; ubx.state = 4; break;
    case 4: ubx.len = b; ubx.ck_a += b; ubx.ck_b += ubx.ck_a; ubx.state = 5; break;
    case 5:
        ubx.len |= (uint16_t)b << 8;
        ubx.ck_a += b; ubx.ck_b += ubx.ck_a;
        ubx.pos = 0;
        ubx.state = (ubx.len == 0) ? 7 : 6;
        break;
    case 6:
        if (ubx.pos < sizeof(ubx.payload)) ubx.payload[ubx.pos] = b;
        ubx.pos++;
        ubx.ck_a += b; ubx.ck_b += ubx.ck_a;
        if (ubx.pos >= ubx.len) ubx.state = 7;
        break;
    case 7: ubx.state = (b == ubx.ck_a) ? 8 : 0; break;
    case 8:
        if (b == ubx.ck_b && ubx.len <= sizeof(ubx.payload)) ubx_handle_frame();
        ubx.state = 0;
        break;
    }
}

// listen briefly and classify what (if anything) arrives
static probe_result_t probe_baud(int baud) {
    uart_set_baudrate(GPS_UART_NUM, baud);
    uart_flush_input(GPS_UART_NUM);

    uint8_t buf[512];
    size_t got = 0;
    bool ubx_seen = false;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
    while (xTaskGetTickCount() < end && got < sizeof(buf) - 1) {
        int r = uart_read_bytes(GPS_UART_NUM, buf + got, sizeof(buf) - 1 - got, pdMS_TO_TICKS(200));
        if (r > 0) {
            got += r;
            buf[got] = '\0';
            if (strstr((char *)buf, "$G")) {
                return PROBE_NMEA;
            }
            for (size_t i = 0; i + 1 < got; i++) {
                if (buf[i] == 0xB5 && buf[i + 1] == 0x62) {
                    ubx_seen = true;
                }
            }
        }
    }
    if (ubx_seen) return PROBE_UBX;
    if (got > 0) {
        ESP_LOGI(TAG, "baud %d: %u bytes but no valid protocol (wrong baud?)", baud, (unsigned)got);
        return PROBE_GARBAGE;
    }
    return PROBE_SILENT;
}

static void gps_task(void *arg) {
    bool config_done = false;   // configure once per boot, not per resync

    for (;;) {
        // auto-baud: cycle candidates until a protocol shows up (module
        // may be unpowered or cold at boot - keep trying forever)
        int baud = 0;
        while (baud == 0) {
            bool anything = false;
            for (size_t i = 0; i < sizeof(baud_candidates) / sizeof(baud_candidates[0]); i++) {
                probe_result_t res = probe_baud(baud_candidates[i]);
                if (res != PROBE_SILENT) anything = true;

                if (res == PROBE_UBX) {
                    ESP_LOGI(TAG, "UBX protocol at %d baud - using NAV-PVT", baud_candidates[i]);
                    baud = baud_candidates[i];
                    break;
                }
                if (res == PROBE_NMEA) {
                    baud = baud_candidates[i];
                    break;
                }
            }
            if (baud == 0) {
                if (anything) {
                    ESP_LOGW(TAG, "GPS data seen but no valid protocol yet, retrying...");
                } else {
                    ESP_LOGW(TAG, "no data on RX at all - check wiring (GPS TX -> RXD pad), retrying...");
                }
            }
        }
        ESP_LOGI(TAG, "GPS detected at %d baud", baud);

        if (!config_done) {
            config_done = true;
#if GPS_AUTO_CONFIG
            ubx_config_link();
            xTaskCreatePinnedToCore(gps_cfg_task, "gps_cfg", 3072, NULL, 5, NULL, 1);
#else
            // make sure NMEA output is on for modules a FC switched to UBX-only
            ubx_enable_nmea_output();
#endif
        }

        char line[LINE_MAX];
        size_t len = 0;
        uint8_t chunk[128];
        last_ok_tick = xTaskGetTickCount();

        // read until the stream goes quiet (e.g. a baud change the module
        // didn't follow), then fall back to a fresh scan
        while (xTaskGetTickCount() - last_ok_tick < pdMS_TO_TICKS(8000)) {
            int r = uart_read_bytes(GPS_UART_NUM, chunk, sizeof(chunk), pdMS_TO_TICKS(200));
            for (int i = 0; i < r; i++) {
                char c = (char)chunk[i];

                // both parsers run on the stream: NMEA ignores binary,
                // the UBX state machine ignores text
                ubx_parse_byte((uint8_t)c);

                if (c == '\n' || c == '\r') {
                    if (len > 6) {
                        line[len] = '\0';
                        handle_line(line);
                    }
                    len = 0;
                } else if (len < LINE_MAX - 1) {
                    line[len++] = c;
                } else {
                    len = 0;   // oversized garbage, resync
                }
            }
        }
        ESP_LOGW(TAG, "GPS stream went quiet, rescanning baud rates");
    }
}

void gps_uart_start(void) {
    const uart_config_t cfg = {
        .baud_rate = baud_candidates[0],
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_UART_TX_GPIO, GPS_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // core 1 = measurement world (GPS + IMU), away from LVGL/SD on core 0:
    // the perf math is iTOW-timestamped so UI load can't skew it, but a
    // dedicated core removes even scheduling jitter from the sample stream
    xTaskCreatePinnedToCore(gps_task, "gps_uart", 4096, NULL, 6, NULL, 1);
}
