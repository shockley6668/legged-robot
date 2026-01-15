#ifndef _CAN_BSP_H
#define _CAN_BSP_H


#include "main.h"


#define CAN_CLASS   0
#define CAN_FD_BRS  1

#define CAN_BR_125K 0
#define CAN_BR_200K 1
#define CAN_BR_250K 2
#define CAN_BR_500K 3
#define CAN_BR_1M   4
#define CAN_BR_2M   5
#define CAN_BR_2M5  6
#define CAN_BR_3M2  7
#define CAN_BR_4M   8
#define CAN_BR_5M   9


typedef FDCAN_HandleTypeDef hcan_t;

extern void FDCAN1_Config(void);
extern void FDCAN2_Config(void);

extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

extern void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud);

#endif

