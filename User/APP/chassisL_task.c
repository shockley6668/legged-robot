/**
  *********************************************************************
  * @file      chassisL_task.c/h
  * @brief     该任务控制左腿的五个电机，ID 2,3,4是DM8006，ID
  1,5是DM6006，这五个电机挂载在can2总线上
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "chassisL_task.h"
#include "body_task.h"
#include "cmsis_os.h"
#include "dm4310_drv.h"
#include "fdcan.h"
#include "math.h"
#include "protocol_task.h"
#include "usbd_cdc_if.h"

extern chassis_t chassis_move;

// 将周期还原为 10ms，即 100Hz
uint32_t CHASSL_TIME = 10;

float my_kd = 0.0f;
float my_vel = 0.0f;
float my_kp = 0.0f;
float my_pos = 0.0f;
int b = 0;
float fric = 0.0;
void ChassisL_task(void) {
  osDelay(2000);
  ChassisL_init(&chassis_move);

  dm6006_fbdata_init(&chassis_move.joint_motor[0]); // ID 5
  dm8006_fbdata_init(&chassis_move.joint_motor[1]); // ID 4
  dm8006_fbdata_init(&chassis_move.joint_motor[2]); // ID 3
  dm8006_fbdata_init(&chassis_move.joint_motor[3]); // ID 2
  dm6006_fbdata_init(&chassis_move.joint_motor[4]); // ID 1
  // chassis_move.joint_motor[0].para.p_int_test=float_to_uint(0.0f, P_MIN3, P_MAX3, 16);
  // chassis_move.joint_motor[0].para.v_int_test = float_to_uint(0.0f, V_MIN3, V_MAX3, 12);
  // chassis_move.joint_motor[0].para.kp_int_test = float_to_uint(15.0f, KP_MIN3, KP_MAX3, 12);
  // chassis_move.joint_motor[0].para.kd_int_test = float_to_uint(0.5f, KD_MIN3, KD_MAX3, 12);
  // chassis_move.joint_motor[0].para.t_int_test = float_to_uint(0.0f, T_MIN3, T_MAX3, 12);
  // save_motor_zero(&hfdcan2, 0x03, MIT_MODE);
  while (1) {
    if (chassis_move.start_flag == 1) {
      // 根据从上位机收到的指令控制电机
      mit_ctrl_test(&hfdcan2, 0x05, &chassis_move.joint_motor[4]); // ID 5
      mit_ctrl_test(&hfdcan2, 0x04, &chassis_move.joint_motor[3]); // ID 4
      mit_ctrl_test(&hfdcan2, 0x03, &chassis_move.joint_motor[2]); // ID 3
      mit_ctrl_test(&hfdcan2, 0x02, &chassis_move.joint_motor[1]); // ID 2
      mit_ctrl_test(&hfdcan2, 0x01, &chassis_move.joint_motor[0]); // ID 1
    } else {
      mit_ctrl3(&hfdcan2, 0x05, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // left_pitch

      mit_ctrl4(&hfdcan2, 0x04, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // left_yaw

      mit_ctrl4(&hfdcan2, 0x03, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // left_roll

      mit_ctrl4(&hfdcan2, 0x02, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // left_calf

      mit_ctrl3(&hfdcan2, 0x01, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // left_foot
    }

    if (b == 1) {
      // save_motor_zero(&hfdcan2,0x05, MIT_MODE);
      //			osDelay(CHASSL_TIME);
      //	save_motor_zero(&hfdcan2,0x04, MIT_MODE);
      //			osDelay(CHASSL_TIME);
      // save_motor_zero(&hfdcan2,0x03, MIT_MODE);
      //			osDelay(CHASSL_TIME);
      //	save_motor_zero(&hfdcan2,0x02, MIT_MODE);
      //			osDelay(CHASSL_TIME);
      save_motor_zero(&hfdcan2, 0x01, MIT_MODE);
      osDelay(CHASSL_TIME);
    }

    // Send ALL 10 motors in one optimized packet (MsgID 0x03) at 100Hz
    // Payload Size: 10 motors * 4 bytes = 40 bytes.
    // Packet Size: 10(Header) + 40(Payload) + 2(CRC) = 52 bytes (< 64 bytes,
    // SAFE)
    uint8_t payload[40];
    for (uint8_t i = 0; i < 10; i++) {
      Joint_Motor_t *motor = &chassis_move.joint_motor[i];
      uint8_t offset = i * 4;

      // Position (16-bit)
      payload[offset + 0] = (uint8_t)(motor->para.p_int >> 8);
      payload[offset + 1] = (uint8_t)(motor->para.p_int & 0xFF);

      // Velocity (send 16-bit directly, assuming v_int fits or is padded)
      payload[offset + 2] = (uint8_t)(motor->para.v_int >> 8);
      payload[offset + 3] = (uint8_t)(motor->para.v_int & 0xFF);
    }
    mavlink_send_msg(0x03, payload, 40);

    osDelay(CHASSL_TIME);
  }
}

void ChassisL_init(chassis_t *chassis) {
  joint_motor_init(&chassis->joint_motor[0], 1, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[1], 2, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[2], 3, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[3], 4, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[4], 5, MIT_MODE);
  osDelay(20);
  // 1. 强制切换模式并清报警
  for (int i = 0; i < 5; i++) {
    uint16_t current_id = chassis->joint_motor[i].para.id;
    for (int j = 0; j < 5; j++) {
      switch_control_mode(&hfdcan2, current_id, 1);
      osDelay(5);
    }
    disable_motor_mode(&hfdcan2, current_id, MIT_MODE);
    osDelay(10);
  }

  // 2. 依次使能电机
  for (int i = 0; i < 5; i++) {
    uint16_t current_id = chassis->joint_motor[i].para.id;
    for (int j = 0; j < 10; j++) {
      enable_motor_mode(&hfdcan2, current_id, MIT_MODE);
      osDelay(10);
    }
  }
}
