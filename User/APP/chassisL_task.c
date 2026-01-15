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
#include "fdcan.h"

#include "cmsis_os.h"

extern chassis_t chassis_move;

uint32_t CHASSL_TIME = 1;

float my_kd = 0.0f;
float my_vel = 0.0f;
float my_kp = 0.0f;
float my_pos = 0.0f;
int b = 0;
float fric = 0.0;
void ChassisL_task(void) {
  osDelay(2000);
  ChassisL_init(&chassis_move);

  dm6006_fbdata_init(&chassis_move.joint_motor[0]);
  dm8006_fbdata_init(&chassis_move.joint_motor[1]);
  dm8006_fbdata_init(&chassis_move.joint_motor[2]);
  dm8006_fbdata_init(&chassis_move.joint_motor[3]);
  dm6006_fbdata_init(&chassis_move.joint_motor[4]);

  while (1) {
    if (chassis_move.start_flag == 1) {
      mit_ctrl_test(&hfdcan2, 0x05, &chassis_move.joint_motor[0]);

      mit_ctrl_test(&hfdcan2, 0x04, &chassis_move.joint_motor[1]);

      mit_ctrl_test(&hfdcan2, 0x03, &chassis_move.joint_motor[2]);

      mit_ctrl_test(&hfdcan2, 0x02, &chassis_move.joint_motor[3]);

      mit_ctrl_test(&hfdcan2, 0x01, &chassis_move.joint_motor[4]);

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
    osDelay(CHASSL_TIME);
  }
}

void ChassisL_init(chassis_t *chassis) {
  joint_motor_init(&chassis->joint_motor[0], 5, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[1], 4, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[2], 3, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[3], 2, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[4], 1, MIT_MODE);

  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[0].para.id,
                      chassis->joint_motor[0].mode);
    osDelay(20);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[1].para.id,
                      chassis->joint_motor[1].mode);
    osDelay(20);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[2].para.id,
                      chassis->joint_motor[2].mode);
    osDelay(20);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[3].para.id,
                      chassis->joint_motor[3].mode);
    osDelay(20);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[4].para.id,
                      chassis->joint_motor[4].mode);
    osDelay(20);
  }
}
