#include "protocol_task.h"
#include "chassisR_task.h"
#include "cmsis_os.h"
#include "dm4310_drv.h"
#include "fdcan.h"
#include "string.h"
#include "usbd_cdc_if.h"

#define MAVLINK_STX 0xFD

QueueHandle_t usb_tx_queue;

extern chassis_t chassis_move;

/* CRC16 X.25 for MAVLink */
static uint16_t crc_accumulate(uint8_t data, uint16_t crc) {
  uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
  tmp ^= (tmp << 4);
  return (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

static void pack_mavlink(usb_msg_t *msg, uint32_t msgid, const uint8_t *payload,
                         uint8_t len) {
  uint16_t crc = 0xFFFF;
  uint8_t i = 0;

  msg->data[i++] = MAVLINK_STX;
  msg->data[i++] = len;
  msg->data[i++] = 0; // inc_flags
  msg->data[i++] = 0; // cmp_flags
  static uint8_t seq = 0;
  msg->data[i++] = seq++;
  msg->data[i++] = 1; // sysid
  msg->data[i++] = 1; // compid
  msg->data[i++] = (uint8_t)(msgid & 0xFF);
  msg->data[i++] = (uint8_t)((msgid >> 8) & 0xFF);
  msg->data[i++] = (uint8_t)((msgid >> 16) & 0xFF);

  for (uint8_t j = 1; j < i; j++)
    crc = crc_accumulate(msg->data[j], crc);
  if (len > 0 && payload != NULL) {
    memcpy(&msg->data[i], payload, len);
    for (uint8_t j = 0; j < len; j++)
      crc = crc_accumulate(payload[j], crc);
    i += len;
  }
  msg->data[i++] = (uint8_t)(crc & 0xFF);
  msg->data[i++] = (uint8_t)((crc >> 8) & 0xFF);
  msg->len = i;
}

void mavlink_send_msg(uint32_t msgid, const uint8_t *payload, uint8_t len) {
  usb_msg_t msg;
  pack_mavlink(&msg, msgid, payload, len);
  xQueueSend(usb_tx_queue, &msg, 2);
}

void mavlink_send_msg_from_isr(uint32_t msgid, const uint8_t *payload,
                               uint8_t len) {
  usb_msg_t msg;
  pack_mavlink(&msg, msgid, payload, len);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(usb_tx_queue, &msg, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#include "usbd_cdc.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

void USB_Tx_task_entry(void const *argument) {
  usb_msg_t msg;
  while (1) {
    if (xQueueReceive(usb_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {
      // 1. Start transmission
      while (CDC_Transmit_HS(msg.data, msg.len) == USBD_BUSY) {
        osDelay(1);
      }
      // 2. WAIT for transmission to complete (TxState == 0)
      // This is crucial because CDC_Transmit_HS is zero-copy and 'msg' is a
      // local buffer stack variable. If we loop back and xQueueReceive
      // overwrites 'msg' before USB DMA is done, data is corrupted.
      USBD_CDC_HandleTypeDef *hcdc =
          (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
      while (hcdc != NULL && hcdc->TxState != 0) {
        // Busy wait is acceptable here as this is a high-priority communication
        // task and USB HS transmission is very fast (microseconds) osDelay(1)
        // would be too slow (limiting to 1000Hz), so we just burn cycles or
        // yield taskYIELD();
      }
    }
  }
}

#define RX_BUF_SIZE 512
static uint8_t rx_buf[RX_BUF_SIZE];
static uint16_t rx_len = 0;

// Handle received MAVLink message (called from USB ISR)
void mavlink_handle_msg(uint8_t *data, uint32_t len) {
  // 1. Append new data to rx_buf
  for (uint32_t i = 0; i < len; i++) {
    if (rx_len < RX_BUF_SIZE) {
      rx_buf[rx_len++] = data[i];
    } else {
      // Buffer overflow protection: drop everything and restart
      rx_len = 0;
      rx_buf[rx_len++] = data[i];
    }
  }

  // 2. Process complete messages from the buffer
  while (rx_len > 0) {
    // Find STX (0xFD)
    uint16_t stx_idx = 0;
    while (stx_idx < rx_len && rx_buf[stx_idx] != MAVLINK_STX) {
      stx_idx++;
    }

    // Drop invalid bytes before STX
    if (stx_idx > 0) {
      rx_len -= stx_idx;
      memmove(rx_buf, &rx_buf[stx_idx], rx_len);
    }

    // A minimal packet without payload: 10 bytes header + 2 bytes CRC = 12 bytes
    if (rx_len < 12) {
      break; // Need more data
    }

    uint8_t payload_len = rx_buf[1];
    uint16_t total_frame_len = 12 + payload_len; 

    if (rx_len < total_frame_len) {
      break; // Need more data for the full payload + CRC
    }

    // 3. Verify CRC
    uint16_t crc_calc = 0xFFFF;
    for (uint16_t i = 1; i < total_frame_len - 2; i++) {
      crc_calc = crc_accumulate(rx_buf[i], crc_calc);
    }

    uint16_t crc_recv = rx_buf[total_frame_len - 2] | (rx_buf[total_frame_len - 1] << 8);

    if (crc_calc == crc_recv) {
      // CRC Match - Parse complete message
      uint32_t msgid = rx_buf[7] | (rx_buf[8] << 8) | (rx_buf[9] << 16);
      uint8_t *payload = &rx_buf[10];

      switch (msgid) {
      case 0x02: // Batch Position Command (20 bytes target)
      {
        for (uint8_t i = 0; i < 10; ++i) {
          Joint_Motor_t *motor = &chassis_move.joint_motor[i];
          uint8_t offset = i * 2;

          // Parse Position (2 bytes)
          motor->para.p_int_test = (payload[offset] << 8) | payload[offset + 1];

          // Set Fixed Parameters (V=0, P=15, D=0.5, T=0)
          // Note: Using driver helper float_to_uint. Assuming included.
          if (i == 0 || i == 4 || i == 5 || i == 9) { // DM6006
            motor->para.v_int_test = float_to_uint(0.0f, V_MIN3, V_MAX3, 12);
            motor->para.kp_int_test = float_to_uint(15.0f, KP_MIN3, KP_MAX3, 12);
            motor->para.kd_int_test = float_to_uint(0.5f, KD_MIN3, KD_MAX3, 12);
            motor->para.t_int_test = float_to_uint(0.0f, T_MIN3, T_MAX3, 12);
          } else { // DM8006
            motor->para.v_int_test = float_to_uint(0.0f, V_MIN4, V_MAX4, 12);
            motor->para.kp_int_test = float_to_uint(17.0f, KP_MIN4, KP_MAX4, 12);
            motor->para.kd_int_test = float_to_uint(1.5f, KD_MIN4, KD_MAX4, 12);
            motor->para.t_int_test = float_to_uint(0.0f, T_MIN4, T_MAX4, 12);
          }
        }
      } break;

      case 0xFF: // MsgID: Save Zero Point ('S' 'T' [ID])
      {
        if (payload[0] == 0x53 && payload[1] == 0x54) { // 'S', 'T'
          uint8_t target_id = payload[2];
          for (uint8_t i = 0; i < 10; i++) {
            if (target_id == 0xFF || target_id == i) {
              Joint_Motor_t *m_ptr = &chassis_move.joint_motor[i];
              // Send save command via CAN
              if (i < 5)
                save_motor_zero(&hfdcan2, m_ptr->para.id, m_ptr->mode);
              else
                save_motor_zero(&hfdcan1, m_ptr->para.id, m_ptr->mode);
            }
          }
        }
      } break;
      }
      
      // Remove the processed valid frame
      rx_len -= total_frame_len;
      memmove(rx_buf, &rx_buf[total_frame_len], rx_len);
    } else {
      // CRC Error - Drop just the STX byte to re-sync
      rx_len -= 1;
      memmove(rx_buf, &rx_buf[1], rx_len);
    }
  }
}

// Batch send all 10 motors periodically (100Hz)
void Protocol_task_entry(void const *argument) {
  extern chassis_t chassis_move;
  // TickType_t xLastWakeTime = xTaskGetTickCount();
  // const TickType_t xPeriod = pdMS_TO_TICKS(10); // 100Hz = 10ms

  while (1) {
    // Wait for next cycle (100Hz)
    // vTaskDelayUntil(&xLastWakeTime, xPeriod);
    osDelay(10);

    // Pack all 10 motors into a single 60-byte payload
    // MOVED TO CHASSIS_TASK for better sync
  }
}
