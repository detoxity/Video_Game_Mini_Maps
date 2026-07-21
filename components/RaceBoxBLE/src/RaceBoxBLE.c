#include <string.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_bt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "RaceBoxBLE.h"
#include "QMI8658_IMU.h"
#include "AXP2101_PMU.h"

static const char *TAG = "racebox";

// Nordic UART service, the transport a RaceBox Mini uses
// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E (NimBLE wants the bytes reversed)
#define NUS_UUID_BYTES(b12, b13) \
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
    0x93, 0xF3, 0xA3, 0xB5, (b12), (b13), 0x40, 0x6E

static const ble_uuid128_t nus_svc_uuid = BLE_UUID128_INIT(NUS_UUID_BYTES(0x01, 0x00));
static const ble_uuid128_t nus_tx_uuid  = BLE_UUID128_INIT(NUS_UUID_BYTES(0x03, 0x00));
static const ble_uuid128_t nus_rx_uuid  = BLE_UUID128_INIT(NUS_UUID_BYTES(0x02, 0x00));

// RaceBox data message: B5 62 FF 01 50 00 | 80-byte payload | ckA ckB
#define RB_PAYLOAD_LEN  80
#define RB_FRAME_LEN    (6 + RB_PAYLOAD_LEN + 2)

static uint8_t  own_addr_type;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t tx_val_handle;
static bool     notify_on = false;
static bool     streaming = true;
static char     device_name[32];

bool racebox_ble_connected(void) {
    return conn_handle != BLE_HS_CONN_HANDLE_NONE && notify_on;
}

void racebox_ble_set_streaming(bool on) {
    streaming = on;
}

// ------------------------------------------------------------- commands
// The official app will not stay connected unless its commands are
// answered; RaceChrono never sends any, which is why it worked already.
// Message numbers follow the RaceBox BLE Protocol, revision 8.
// Set 1 to trace the BLE handshake and every command exchange over the
// console. Off by default: this prints from the NimBLE host task, and
// console writes stall the LVGL thread, which is why logging is compiled
// out in this project to begin with.
#define RB_RX_DEBUG 0

#if RB_RX_DEBUG
#define RB_TRACE(...) printf(__VA_ARGS__)
#else
#define RB_TRACE(...) ((void)0)
#endif

typedef struct {
    uint8_t len;
    uint8_t buf[24];
} rb_reply_t;

// Replies go through NimBLE's own event queue rather than a private task:
// notifying from inside the GATT callback would re-enter the host mutex,
// and a dedicated task needs a stack this build has no internal RAM to
// spare for - an earlier attempt failed to start and answered nothing.
#define RB_REPLY_SLOTS 4
static rb_reply_t reply_slot[RB_REPLY_SLOTS];
static volatile uint8_t reply_w, reply_r;
static struct ble_npl_event reply_ev;

static void reply_ev_cb(struct ble_npl_event *ev)
{
    (void)ev;
    while (reply_r != reply_w) {
        rb_reply_t *r = &reply_slot[reply_r % RB_REPLY_SLOTS];
        reply_r++;
        if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            continue;
        }
        struct os_mbuf *om = ble_hs_mbuf_from_flat(r->buf, r->len);
        int rc = om ? ble_gatts_notify_custom(conn_handle, tx_val_handle, om)
                    : BLE_HS_ENOMEM;
#if RB_RX_DEBUG
        printf("[rb] tx rc=%d:", rc);
        for (int i = 0; i < r->len; i++) printf(" %02X", r->buf[i]);
        printf("\n");
#else
        (void)rc;
#endif
    }
}

static void rb_queue_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint8_t plen)
{
    if ((unsigned)plen + 8u > sizeof(reply_slot[0].buf)) {
        return;
    }
    rb_reply_t *r = &reply_slot[reply_w % RB_REPLY_SLOTS];
    r->len = plen + 8;
    r->buf[0] = 0xB5; r->buf[1] = 0x62;
    r->buf[2] = cls;  r->buf[3] = id;
    r->buf[4] = plen; r->buf[5] = 0x00;
    if (plen) {
        memcpy(&r->buf[6], payload, plen);
    }
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < r->len - 2; i++) {
        ck_a += r->buf[i];
        ck_b += ck_a;
    }
    r->buf[r->len - 2] = ck_a;
    r->buf[r->len - 1] = ck_b;
    reply_w++;

    ble_npl_eventq_put(nimble_port_get_dflt_eventq(), &reply_ev);
}

// ACK and NACK carry the class and ID of the command they answer.
static void rb_handle_command(const uint8_t *f, uint16_t plen)
{
    const uint8_t cls = f[2], id = f[3];
    const uint8_t ack[2] = { cls, id };

    // Class 0x06 is plain u-blox config aimed at the receiver - the app
    // sends CFG-VALSET to set the dynamic platform model. It expects the
    // receiver's own UBX-ACK-ACK back, not a RaceBox reply.
    if (cls == 0x06) {
        rb_queue_frame(0x05, 0x01, ack, 2);
        return;
    }
    if (cls != 0xFF) {
        return;
    }

    switch (id) {
    case 0x27:   // GNSS receiver configuration
        if (plen == 0) {
            // dynamic model 4 (automotive), no 3D speed, no accuracy limit
            const uint8_t cfg[3] = { 4, 0, 0 };
            rb_queue_frame(0xFF, 0x27, cfg, 3);
        } else {
            rb_queue_frame(0xFF, 0x02, ack, 2);
        }
        break;

    case 0x22:   // standalone recording status
    case 0x23:   // recorded data download
    case 0x24:   // recorded data erase
    case 0x25:   // standalone recording configuration
        // This build has no session memory, which the protocol lists as a
        // NACK condition outright. Tracks are logged to the SD card instead.
        rb_queue_frame(0xFF, 0x03, ack, 2);
        break;

    case 0x30:   // unlock memory - nothing is locked, so confirm
        rb_queue_frame(0xFF, 0x02, ack, 2);
        break;

    default:
        rb_queue_frame(0xFF, 0x03, ack, 2);
        break;
    }
}

static int gatt_rx_cb(uint16_t ch, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)ch; (void)attr; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return 0;
    }

    uint8_t buf[64];
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len > sizeof(buf)) len = sizeof(buf);
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL) != 0) {
        return 0;
    }
#if RB_RX_DEBUG
    printf("[rb] rx %u:", (unsigned)len);
    for (uint16_t i = 0; i < len; i++) printf(" %02X", buf[i]);
    printf("\n");
#endif

    // a write may carry more than one frame; walk them
    uint16_t off = 0;
    while (off + 8 <= len) {
        if (buf[off] != 0xB5 || buf[off + 1] != 0x62) {
            off++;
            continue;
        }
        uint16_t plen = buf[off + 4] | ((uint16_t)buf[off + 5] << 8);
        if (off + plen + 8u > len) {
            break;
        }
        rb_handle_command(&buf[off], plen);
        off += plen + 8;
    }
    return 0;
}

// Device Information Service. RaceChrono only needs the UART service, but
// the official app reads these to identify the unit - without them it
// refuses the connection.
static char serial_str[16];

static int dis_read_cb(uint16_t ch, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)ch; (void)attr; (void)arg;
    const char *s;
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);
    switch (uuid) {
    // Values match a real unit as the reference emulator reports them. The
    // app's page labels a Mini "Model: R01" no matter what we send here, so
    // it clearly does not display this string.
    case 0x2A29: s = "RaceBox";      break;   // manufacturer
    case 0x2A24: s = "RaceBox Mini"; break;   // model
    case 0x2A25: s = serial_str;     break;   // serial number
    case 0x2A27: s = "1";            break;   // hardware revision
    // 3.2, not 3.3: from 3.3 on a unit also carries a second UART service
    // streaming NMEA 0183, and we have no NMEA to give. Everything else in
    // the 3.x line we do implement, and the protocol has no breaking
    // changes across 1.x, 2.x and 3.x, so nothing is lost by saying so.
    case 0x2A26: s = "3.2";          break;   // firmware revision
    default:     return BLE_ATT_ERR_UNLIKELY;
    }
    // Confirmed the app reads all of these when the device entry is fresh,
    // so no tracing here: five printfs back to back on the host task are
    // stack pressure this callback does not need.
    return os_mbuf_append(ctxt->om, s, strlen(s)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int bas_read_cb(uint16_t ch, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)ch; (void)attr; (void)arg;
    int pct = pmu_battery_percent();
    if (pct < 0) {
        int mv = pmu_battery_voltage_mv();
        pct = (mv > 0) ? (mv - 3300) * 100 / 900 : 0;
    }
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    uint8_t level = (uint8_t)pct;
    return os_mbuf_append(ctxt->om, &level, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Order matches the reference emulator: the UART service is registered
// first, so it lands on the same handles a real unit uses, and device
// information follows it.
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &nus_tx_uuid.u,
                .access_cb = gatt_rx_cb,
                .val_handle = &tx_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &nus_rx_uuid.u,
                .access_cb = gatt_rx_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 }
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),   // Device Information
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = BLE_UUID16_DECLARE(0x2A24), .access_cb = dis_read_cb, .flags = BLE_GATT_CHR_F_READ },
            { .uuid = BLE_UUID16_DECLARE(0x2A25), .access_cb = dis_read_cb, .flags = BLE_GATT_CHR_F_READ },
            { .uuid = BLE_UUID16_DECLARE(0x2A26), .access_cb = dis_read_cb, .flags = BLE_GATT_CHR_F_READ },
            { .uuid = BLE_UUID16_DECLARE(0x2A27), .access_cb = dis_read_cb, .flags = BLE_GATT_CHR_F_READ },
            { .uuid = BLE_UUID16_DECLARE(0x2A29), .access_cb = dis_read_cb, .flags = BLE_GATT_CHR_F_READ },
            { 0 }
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F),   // Battery
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = BLE_UUID16_DECLARE(0x2A19), .access_cb = bas_read_cb,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY },
            { 0 }
        },
    },
    { 0 }
};

// ---------------------------------------------------------------- GAP
static void start_advertising(void);

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        RB_TRACE("[rb] connect status=%d\n", event->connect.status);
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            // Android caches the service table per address, and this one has
            // changed shape across firmware revisions - handles moved when
            // the device information and battery services were added. Tell
            // the client to discover again instead of trusting its cache.
            ble_svc_gatt_changed(0x0001, 0xFFFF);
        } else {
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        // reason tells the story: 0x13 remote hung up, 0x08 supervision
        // timeout, 0x05/0x06 authentication or encryption demanded
        RB_TRACE("[rb] disconnect reason=0x%02X\n", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        notify_on = false;
        start_advertising();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        RB_TRACE("[rb] subscribe attr=%u notify=%d indicate=%d\n",
                 (unsigned)event->subscribe.attr_handle,
                 event->subscribe.cur_notify, event->subscribe.cur_indicate);
        if (event->subscribe.attr_handle == tx_val_handle) {
            notify_on = event->subscribe.cur_notify;
        }
        break;

    case BLE_GAP_EVENT_MTU:
        RB_TRACE("[rb] mtu=%u\n", (unsigned)event->mtu.value);
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        RB_TRACE("[rb] enc change status=%d\n", event->enc_change.status);
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;

    default:
        break;   // NOTIFY_TX fires per packet - far too noisy to trace
    }
    return 0;
}

static void start_advertising(void)
{
    // The 128-bit UUID (18B) and the full name (25B) cannot share one
    // 31-byte advertisement. Apps scan for the service UUID, so that goes in
    // the advertisement and the name goes in the scan response - putting
    // both in the advertisement silently dropped the UUID and made the
    // device invisible to RaceChrono/Solostorm.
    struct ble_hs_adv_fields adv_fields = {0};
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.uuids128 = (ble_uuid128_t *)&nus_svc_uuid;
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;
    // Device Information is advertised alongside it because the official app
    // looks for it when discovering RaceBox units. 3 bytes of flags plus an
    // 18-byte 128-bit UUID plus this 4-byte 16-bit UUID still fit in 31.
    static const ble_uuid16_t dis_uuid = BLE_UUID16_INIT(0x180A);
    adv_fields.uuids16 = (ble_uuid16_t *)&dis_uuid;
    adv_fields.num_uuids16 = 1;
    adv_fields.uuids16_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGW(TAG, "adv fields failed (%d)", rc);
    }

    struct ble_hs_adv_fields rsp_fields = {0};
    rsp_fields.name = (uint8_t *)device_name;
    rsp_fields.name_len = strlen(device_name);
    rsp_fields.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGW(TAG, "scan rsp failed (%d)", rc);
    }

    struct ble_gap_adv_params adv = {0};
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv, gap_event, NULL);
}

static void on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &own_addr_type);
    start_advertising();
    ESP_LOGI(TAG, "advertising as \"%s\"", device_name);
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ---------------------------------------------------------------- packet
void racebox_ble_publish(const uint8_t *nav_pvt, uint16_t len)
{
    if (!streaming || !racebox_ble_connected() || !nav_pvt || len < 80) {
        return;
    }

    uint8_t f[RB_FRAME_LEN];
    f[0] = 0xB5; f[1] = 0x62; f[2] = 0xFF; f[3] = 0x01;
    f[4] = RB_PAYLOAD_LEN; f[5] = 0x00;

    uint8_t *p = &f[6];
    memset(p, 0, RB_PAYLOAD_LEN);

    // NAV-PVT maps almost 1:1 - the RaceBox payload is the same fields with
    // the NED velocity block (bytes 48..59) removed.
    memcpy(p +  0, nav_pvt +  0, 48);   // iTOW, date/time, fix, lat/lon, alt, acc
    memcpy(p + 48, nav_pvt + 60, 16);   // gSpeed, headMot, sAcc, headAcc
    memcpy(p + 64, nav_pvt + 76, 2);    // pDOP
    p[66] = nav_pvt[78] & 0x01;         // flags3 bit0: invalid lat/lon

    // Sampled once a second, not once per frame: on the AXP2101 board the
    // gauge read is an I2C transaction, and running it 25 times a second on
    // the GPS task puts a blocking bus access in the measurement path for a
    // value that moves over minutes.
    static uint8_t  batt_byte = 0;
    static bool     batt_valid = false;
    static uint32_t batt_tick = 0;
    uint32_t tick = xTaskGetTickCount();
    if (!batt_valid || tick - batt_tick >= pdMS_TO_TICKS(1000)) {
        batt_tick = tick;
        batt_valid = true;
        int pct = pmu_battery_percent();
        if (pct < 0) {
            int mv = pmu_battery_voltage_mv();   // no gauge: estimate 3.3-4.2V
            pct = (mv > 0) ? (mv - 3300) * 100 / 900 : 0;
        }
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        batt_byte = (uint8_t)pct | (pmu_vbus_present() ? 0x80 : 0x00);
    }
    p[67] = batt_byte;

    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    imu_get_accel(&ax, &ay, &az);
    imu_get_gyro(&gx, &gy, &gz);
    // g-force in milli-g, rotation in centi-degrees/second
    const float MG = 1000.0f / 9.80665f;
    int16_t v[6] = {
        (int16_t)(ax * MG), (int16_t)(ay * MG), (int16_t)(az * MG),
        (int16_t)(gx * 100.0f), (int16_t)(gy * 100.0f), (int16_t)(gz * 100.0f),
    };
    for (int i = 0; i < 6; i++) {
        p[68 + i * 2]     = (uint8_t)(v[i] & 0xFF);
        p[68 + i * 2 + 1] = (uint8_t)((v[i] >> 8) & 0xFF);
    }

    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < RB_FRAME_LEN - 2; i++) {
        ck_a += f[i];
        ck_b += ck_a;
    }
    f[RB_FRAME_LEN - 2] = ck_a;
    f[RB_FRAME_LEN - 1] = ck_b;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(f, sizeof(f));
    if (om) {
        ble_gatts_notify_custom(conn_handle, tx_val_handle, om);
    }
}

// ---------------------------------------------------------------- start
void racebox_ble_start(const char *serial)
{
    // The "RaceBox Mini " prefix has to stay exactly as-is (the app matches
    // on it), so only the ID is configurable. Without one, derive it from the
    // MAC - kept under 4000000000, since the app refuses higher IDs.
    if (serial && serial[0]) {
        snprintf(serial_str, sizeof(serial_str), "%s", serial);
    } else {
        uint8_t mac[6] = {0};
        esp_read_mac(mac, ESP_MAC_BT);
        uint64_t n = ((uint64_t)mac[1] << 32) | ((uint64_t)mac[2] << 24) |
                     ((uint64_t)mac[3] << 16) | ((uint64_t)mac[4] << 8) | mac[5];
        // Land in [1000000000, 3999999999]: ten digits without zero padding,
        // and still under the 4000000000 the app refuses. Padding to width
        // with zeros is what has to be avoided - the app strips leading
        // zeros, then treats the padded and stripped forms as two devices.
        n = 1000000000ULL + (n % 3000000000ULL);
        snprintf(serial_str, sizeof(serial_str), "%llu", (unsigned long long)n);
    }
    snprintf(device_name, sizeof(device_name), "RaceBox Mini %s", serial_str);

    if (nimble_port_init() != ESP_OK) {
        ESP_LOGW(TAG, "NimBLE init failed - BLE streaming disabled");
        return;
    }

    // Turn the radio up, as the reference emulator's NimBLE port does. The
    // phone is usually in a cradle a metre away with a body and a dashboard
    // in between, and a dropped link costs a run.
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);

    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    if (ble_gatts_count_cfg(gatt_svcs) != 0 || ble_gatts_add_svcs(gatt_svcs) != 0) {
        ESP_LOGW(TAG, "GATT registration failed");
        return;
    }
    ble_svc_gap_device_name_set(device_name);

    ble_npl_event_init(&reply_ev, reply_ev_cb, NULL);

    nimble_port_freertos_init(host_task);
}
