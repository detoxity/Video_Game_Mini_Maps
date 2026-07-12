#ifdef __cplusplus
extern "C" {
#endif

#include "driver/twai.h"

// ESP32-S3-Touch-LCD-1.85: GPIO43/44 (UART header on the back) are the only
// free exposed pins - wire them to an external CAN transceiver (e.g. SN65HVD230).
// The serial console is moved to USB-Serial-JTAG in sdkconfig.defaults.
#define CAN_TX_GPIO     (gpio_num_t)43
#define CAN_RX_GPIO     (gpio_num_t)44
#define CANBUS_SPEED    500000   // 500kbps

#define CAN_QUEUE_LENGTH 32
#define CAN_QUEUE_ITEM_SIZE sizeof(twai_message_t)
#define TAG "TWAI"

extern bool receiving_data;
extern void (*can_message_handler)(twai_message_t *message);

void canbus_init(void);
void start_can_tasks(void);

#ifdef __cplusplus
}
#endif