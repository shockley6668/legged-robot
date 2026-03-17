#ifndef PROTOCOL_TASK_H
#define PROTOCOL_TASK_H

#include "FreeRTOS.h"
#include "main.h"
#include "queue.h"
#include "stdint.h"

#define MAVLINK_MAX_PACKET_LEN 128

typedef struct {
  uint8_t data[MAVLINK_MAX_PACKET_LEN];
  uint32_t len;
} usb_msg_t;

#ifdef __cplusplus
extern "C" {
#endif

void Protocol_task_entry(void const *argument);
void USB_Tx_task_entry(void const *argument);

void mavlink_send_msg(uint32_t msgid, const uint8_t *payload, uint8_t len);
void mavlink_send_msg_from_isr(uint32_t msgid, const uint8_t *payload,
                               uint8_t len);

void mavlink_handle_msg(uint8_t *data, uint32_t len);

extern QueueHandle_t usb_tx_queue;

#ifdef __cplusplus
}
#endif

#endif
