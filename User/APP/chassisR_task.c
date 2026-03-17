/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右腿的五个电机，ID 2,3,4是DM8006，ID
  1,5是DM6006，这五个电机挂载在can1总线上
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "chassisR_task.h"
#include "cmsis_os.h"
#include "dm4310_drv.h"
#include "fdcan.h"
#include "math.h"
#include "protocol_task.h"

chassis_t chassis_move;

uint32_t CHASSR_TIME = 10;

float my_kd2 = 0.0f;
float my_vel2 = 0.0f;
float my_kp2 = 0.0f;
float my_pos2 = 0.0f;
float my_tor2 = 0.0f;
int a = 0;
uint8_t cc = 1;

void ChassisR_task(void) {
  osDelay(2000);
  // chassis_move.start_flag = 1; // Removed to match L task behavior (wait for
  // control) 
  ChassisR_init(&chassis_move);

  dm6006_fbdata_init(&chassis_move.joint_motor[5]); // ID 1
  dm8006_fbdata_init(&chassis_move.joint_motor[6]); // ID 2
  dm8006_fbdata_init(&chassis_move.joint_motor[7]); // ID 3
  dm8006_fbdata_init(&chassis_move.joint_motor[8]); // ID 4
  dm6006_fbdata_init(&chassis_move.joint_motor[9]); // ID 5

  // chassis_move.joint_motor[5].para.p_int_test=float_to_uint(0.0f, P_MIN3, P_MAX3, 16);
  // chassis_move.joint_motor[5].para.v_int_test = float_to_uint(0.0f, V_MIN3, V_MAX3, 12);
  // chassis_move.joint_motor[5].para.kp_int_test = float_to_uint(15.0f, KP_MIN3, KP_MAX3, 12);
  // chassis_move.joint_motor[5].para.kd_int_test = float_to_uint(0.5f, KD_MIN3, KD_MAX3, 12);
  // chassis_move.joint_motor[5].para.t_int_test = float_to_uint(0.0f, T_MIN3, T_MAX3, 12);
  chassis_move.start_flag=1;
  // save_motor_zero(&hfdcan1, 0x03, MIT_MODE);
  while (1) {
    if (chassis_move.start_flag == 1) {
      // 根据从收到的指令控制电机
      mit_ctrl_test(&hfdcan1, 0x05, &chassis_move.joint_motor[9]); // ID 5
      mit_ctrl_test(&hfdcan1, 0x04, &chassis_move.joint_motor[8]); // ID 4
      mit_ctrl_test(&hfdcan1, 0x03, &chassis_move.joint_motor[7]); // ID 3
      mit_ctrl_test(&hfdcan1, 0x02, &chassis_move.joint_motor[6]); // ID 2
      mit_ctrl_test(&hfdcan1, 0x01, &chassis_move.joint_motor[5]); // ID 1
    } else {
      mit_ctrl3(&hfdcan1, 0x05, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // right_pitch

      mit_ctrl4(&hfdcan1, 0x04, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // right_roll

      mit_ctrl4(&hfdcan1, 0x03, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // right_yaw

      mit_ctrl4(&hfdcan1, 0x02, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // right_calf
      // void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float
      // vel,float kp, float kd, float torq)
      mit_ctrl3(&hfdcan1, 0x01, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // right_foot
    }
    if (a == 1) {
      // save_motor_zero(&hfdcan1,0x05, MIT_MODE);
      //			osDelay(CHASSR_TIME);
      //	save_motor_zero(&hfdcan1,0x04, MIT_MODE);
      //			osDelay(CHASSR_TIME);
      //	save_motor_zero(&hfdcan1,0x03, MIT_MODE);
      //			osDelay(CHASSR_TIME);
      //	save_motor_zero(&hfdcan1,0x02, MIT_MODE);
      //			osDelay(CHASSR_TIME);
      save_motor_zero(&hfdcan1, 0x01, MIT_MODE);
      osDelay(CHASSR_TIME);
    }
    osDelay(CHASSR_TIME);
  }
}

void ChassisR_init(chassis_t *chassis) {
  joint_motor_init(&chassis->joint_motor[5], 1, MIT_MODE); // 发送id为1
  joint_motor_init(&chassis->joint_motor[6], 2, MIT_MODE); // 发送id为2
  joint_motor_init(&chassis->joint_motor[7], 3, MIT_MODE); // 发送id为3
  joint_motor_init(&chassis->joint_motor[8], 4, MIT_MODE); // 发送id为4
  joint_motor_init(&chassis->joint_motor[9], 5, MIT_MODE); // 发送id为5

  osDelay(20);

  // 1. 强制切换模式并清报警
  for (int i = 0; i < 5; i++) {
    uint16_t current_id = chassis->joint_motor[5 + i].para.id;
    for (int j = 0; j < 5; j++) {
      switch_control_mode(&hfdcan1, current_id, 1);
      osDelay(5);
    }
    disable_motor_mode(&hfdcan1, current_id, MIT_MODE);
    osDelay(10);
  }

  // 2. 依次使能电机
  for (int i = 0; i < 5; i++) {
    uint16_t current_id = chassis->joint_motor[5 + i].para.id;
    for (int j = 0; j < 10; j++) {
      enable_motor_mode(&hfdcan1, current_id, MIT_MODE);
      osDelay(10);
    }
  }
}

void mySaturate(float *in, float min, float max) {
  if (*in < min) {
    *in = min;
  } else if (*in > max) {
    *in = max;
  }
}
