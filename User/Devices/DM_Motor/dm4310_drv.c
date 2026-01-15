#include "dm4310_drv.h"

#include "arm_math.h"
#include "fdcan.h"

float Hex_To_Float(uint32_t *Byte, int num) // 十六进制到浮点数
{
  return *((float *)Byte);
}

uint32_t FloatTohex(float HEX) // 浮点数到十六进制转换
{
  return *(uint32_t *)&HEX;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max]
* 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits) {
  /* Converts a float to an unsigned int, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max]
* 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  /* converts unsigned int to float, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t mode) {
  motor->mode = mode;
  motor->para.id = id;
}

void wheel_motor_init(Wheel_Motor_t *motor, uint16_t id, uint16_t mode) {
  motor->mode = mode;
  motor->para.id = id;
}

/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/
void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (data_len == FDCAN_DLC_BYTES_8) { // 返回的数据有8个字节
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos =
        uint_to_float(motor->para.p_int, P_MIN1, P_MAX1, 16); // (-12.5,12.5)
    motor->para.vel =
        uint_to_float(motor->para.v_int, V_MIN1, V_MAX1, 12); // (-30.0,30.0)
    motor->para.tor =
        uint_to_float(motor->para.t_int, T_MIN1, T_MAX1, 12); // (-10.0,10.0)
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

void dm4340_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (data_len == FDCAN_DLC_BYTES_8) { // 返回的数据有8个字节
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN2, P_MAX2, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN2, V_MAX2, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN2, T_MAX2, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

void dm6006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (data_len == FDCAN_DLC_BYTES_8) { // 返回的数据有8个字节
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN3, P_MAX3, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN3, V_MAX3, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN3, T_MAX3, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

void dm8006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (data_len == FDCAN_DLC_BYTES_8) { // 返回的数据有8个字节
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN4, P_MAX4, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN4, V_MAX4, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN4, T_MAX4, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

void dm3507_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (data_len == FDCAN_DLC_BYTES_8) { // 返回的数据有8个字节
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN5, P_MAX5, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN5, V_MAX5, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN5, T_MAX5, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

// 这是测试电脑发给单片机的数据
void dm3507_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data) {
  motor->para.p_int_test = (rx_data[2] << 8) | rx_data[3];

  motor->para.v_int_test =
      (((int16_t)rx_data[4]) << 4) | ((rx_data[5] & 0xF0) >> 4);

  motor->para.kp_int_test = (((int16_t)(rx_data[5] & 0x0F)) << 8) | rx_data[6];

  motor->para.kd_int_test =
      (((int16_t)rx_data[7]) << 4) | ((rx_data[8] & 0xF0) >> 4);

  motor->para.t_int_test = (((int16_t)(rx_data[8] & 0x0F)) << 8) | rx_data[9];

  motor->para.pos_set =
      uint_to_float(motor->para.p_int_test, P_MIN5, P_MAX5, 16);
  motor->para.vel_set =
      uint_to_float(motor->para.v_int_test, V_MIN5, V_MAX5, 12);
  motor->para.tor_set =
      uint_to_float(motor->para.t_int_test, T_MIN5, T_MAX5, 12);
  motor->para.kp_test =
      uint_to_float(motor->para.kp_int_test, KP_MIN5, KP_MAX5, 12);
  motor->para.kd_test =
      uint_to_float(motor->para.kd_int_test, KD_MIN5, KD_MAX5, 12);
}

void dm4310_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data) {
  motor->para.p_int_test = (rx_data[2] << 8) | rx_data[3];

  motor->para.v_int_test =
      (((int16_t)rx_data[4]) << 4) | ((rx_data[5] & 0xF0) >> 4);

  motor->para.kp_int_test = (((int16_t)(rx_data[5] & 0x0F)) << 8) | rx_data[6];

  motor->para.kd_int_test =
      (((int16_t)rx_data[7]) << 4) | ((rx_data[8] & 0xF0) >> 4);

  motor->para.t_int_test = (((int16_t)(rx_data[8] & 0x0F)) << 8) | rx_data[9];

  motor->para.pos_set =
      uint_to_float(motor->para.p_int_test, P_MIN1, P_MAX1, 16);
  motor->para.vel_set =
      uint_to_float(motor->para.v_int_test, V_MIN1, V_MAX1, 12);
  motor->para.tor_set =
      uint_to_float(motor->para.t_int_test, T_MIN1, T_MAX1, 12);
  motor->para.kp_test =
      uint_to_float(motor->para.kp_int_test, KP_MIN1, KP_MAX1, 12);
  motor->para.kd_test =
      uint_to_float(motor->para.kd_int_test, KD_MIN1, KD_MAX1, 12);
}

void dm4340_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data) {
  motor->para.p_int_test = (rx_data[2] << 8) | rx_data[3];

  motor->para.v_int_test =
      (((int16_t)rx_data[4]) << 4) | ((rx_data[5] & 0xF0) >> 4);

  motor->para.kp_int_test = (((int16_t)(rx_data[5] & 0x0F)) << 8) | rx_data[6];

  motor->para.kd_int_test =
      (((int16_t)rx_data[7]) << 4) | ((rx_data[8] & 0xF0) >> 4);

  motor->para.t_int_test = (((int16_t)(rx_data[8] & 0x0F)) << 8) | rx_data[9];

  motor->para.pos_set =
      uint_to_float(motor->para.p_int_test, P_MIN2, P_MAX2, 16);
  motor->para.vel_set =
      uint_to_float(motor->para.v_int_test, V_MIN2, V_MAX2, 12);
  motor->para.tor_set =
      uint_to_float(motor->para.t_int_test, T_MIN2, T_MAX2, 12);
  motor->para.kp_test =
      uint_to_float(motor->para.kp_int_test, KP_MIN2, KP_MAX2, 12);
  motor->para.kd_test =
      uint_to_float(motor->para.kd_int_test, KD_MIN2, KD_MAX2, 12);
}

void dm6006_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data) {
  motor->para.p_int_test = (rx_data[2] << 8) | rx_data[3];

  motor->para.v_int_test =
      (((int16_t)rx_data[4]) << 4) | ((rx_data[5] & 0xF0) >> 4);

  motor->para.kp_int_test = (((int16_t)(rx_data[5] & 0x0F)) << 8) | rx_data[6];

  motor->para.kd_int_test =
      (((int16_t)rx_data[7]) << 4) | ((rx_data[8] & 0xF0) >> 4);

  motor->para.t_int_test = (((int16_t)(rx_data[8] & 0x0F)) << 8) | rx_data[9];

  motor->para.pos_set =
      uint_to_float(motor->para.p_int_test, P_MIN3, P_MAX3, 16);
  motor->para.vel_set =
      uint_to_float(motor->para.v_int_test, V_MIN3, V_MAX3, 12);
  motor->para.tor_set =
      uint_to_float(motor->para.t_int_test, T_MIN3, T_MAX3, 12);
  motor->para.kp_test =
      uint_to_float(motor->para.kp_int_test, KP_MIN3, KP_MAX3, 12);
  motor->para.kd_test =
      uint_to_float(motor->para.kd_int_test, KD_MIN3, KD_MAX3, 12);
}

void dm8006_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data) {
  motor->para.p_int_test = (rx_data[2] << 8) | rx_data[3];

  motor->para.v_int_test =
      (((int16_t)rx_data[4]) << 4) | ((rx_data[5] & 0xF0) >> 4);

  motor->para.kp_int_test = (((int16_t)(rx_data[5] & 0x0F)) << 8) | rx_data[6];

  motor->para.kd_int_test =
      (((int16_t)rx_data[7]) << 4) | ((rx_data[8] & 0xF0) >> 4);

  motor->para.t_int_test = (((int16_t)(rx_data[8] & 0x0F)) << 8) | rx_data[9];

  motor->para.pos_set =
      uint_to_float(motor->para.p_int_test, P_MIN4, P_MAX4, 16);
  motor->para.vel_set =
      uint_to_float(motor->para.v_int_test, V_MIN4, V_MAX4, 12);
  motor->para.tor_set =
      uint_to_float(motor->para.t_int_test, T_MIN4, T_MAX4, 12);
  motor->para.kp_test =
      uint_to_float(motor->para.kp_int_test, KP_MIN4, KP_MAX4, 12);
  motor->para.kd_test =
      uint_to_float(motor->para.kd_int_test, KD_MIN4, KD_MAX4, 12);
}

void enable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id) {
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFC;

  canx_send_data(hcan, id, data, 8);
}

void save_motor_zero(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id) {
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFE;

  canx_send_data(hcan, id, data, 8);
}

void switch_control_mode(hcan_t *hcan, uint16_t motor_id, uint8_t mode) {
  uint8_t data[8];

  data[0] = (motor_id & 0xff);
  data[1] = (motor_id >> 8);
  data[2] = 0x55;
  data[3] = 0x0A;
  data[4] = mode;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;

  canx_send_data(hcan, 0x7FF, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void disable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id) {
  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFD;

  canx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:
*	指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp,
              float kd, float torq) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos, P_MIN1, P_MAX1, 16);
  vel_tmp = float_to_uint(vel, V_MIN1, V_MAX1, 12);
  kp_tmp = float_to_uint(kp, KP_MIN1, KP_MAX1, 12);
  kd_tmp = float_to_uint(kd, KD_MIN1, KD_MAX1, 12);
  tor_tmp = float_to_uint(torq, T_MIN1, T_MAX1, 12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:
*	指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void pos_speed_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel) {
  uint16_t id;
  uint8_t *pbuf, *vbuf;
  uint8_t data[8];

  id = motor_id + POS_MODE;
  pbuf = (uint8_t *)&pos;
  vbuf = (uint8_t *)&vel;

  data[0] = *pbuf;
  data[1] = *(pbuf + 1);
  data[2] = *(pbuf + 2);
  data[3] = *(pbuf + 3);

  data[4] = *vbuf;
  data[5] = *(vbuf + 1);
  data[6] = *(vbuf + 2);
  data[7] = *(vbuf + 3);

  canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void speed_ctrl(hcan_t *hcan, uint16_t motor_id, float vel) {
  uint16_t id;
  uint8_t *vbuf;
  uint8_t data[4];

  id = motor_id + SPEED_MODE;
  vbuf = (uint8_t *)&vel;

  data[0] = *vbuf;
  data[1] = *(vbuf + 1);
  data[2] = *(vbuf + 2);
  data[3] = *(vbuf + 3);

  canx_send_data(hcan, id, data, 4);
}

// 4340
void mit_ctrl2(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp,
               float kd, float torq) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos, P_MIN2, P_MAX2, 16);
  vel_tmp = float_to_uint(vel, V_MIN2, V_MAX2, 12);
  kp_tmp = float_to_uint(kp, KP_MIN2, KP_MAX2, 12);
  kd_tmp = float_to_uint(kd, KD_MIN2, KD_MAX2, 12);
  tor_tmp = float_to_uint(torq, T_MIN2, T_MAX2, 12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

// 6006
void mit_ctrl3(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp,
               float kd, float torq) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos, P_MIN3, P_MAX3, 16);
  vel_tmp = float_to_uint(vel, V_MIN3, V_MAX3, 12);
  kp_tmp = float_to_uint(kp, KP_MIN3, KP_MAX3, 12);
  kd_tmp = float_to_uint(kd, KD_MIN3, KD_MAX3, 12);
  tor_tmp = float_to_uint(torq, T_MIN3, T_MAX3, 12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

// 8006
void mit_ctrl4(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp,
               float kd, float torq) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos, P_MIN4, P_MAX4, 16);
  vel_tmp = float_to_uint(vel, V_MIN4, V_MAX4, 12);
  kp_tmp = float_to_uint(kp, KP_MIN4, KP_MAX4, 12);
  kd_tmp = float_to_uint(kd, KD_MIN4, KD_MAX4, 12);
  tor_tmp = float_to_uint(torq, T_MIN4, T_MAX4, 12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

// 3507
void mit_ctrl5(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp,
               float kd, float torq) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos, P_MIN5, P_MAX5, 16);
  vel_tmp = float_to_uint(vel, V_MIN5, V_MAX5, 12);
  kp_tmp = float_to_uint(kp, KP_MIN5, KP_MAX5, 12);
  kd_tmp = float_to_uint(kd, KD_MIN5, KD_MAX5, 12);
  tor_tmp = float_to_uint(torq, T_MIN5, T_MAX5, 12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

void mit_ctrl_test(hcan_t *hcan, uint16_t motor_id, Joint_Motor_t *motor) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = motor->para.p_int_test;
  vel_tmp = motor->para.v_int_test;
  kp_tmp = motor->para.kp_int_test;
  kd_tmp = motor->para.kd_int_test;
  tor_tmp = motor->para.t_int_test;

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

// 一开始初始化，不然初值不为0
void dm4310_fbdata_init(Joint_Motor_t *motor) {
  motor->para.p_int_test = float_to_uint(0.0f, P_MIN1, P_MAX1, 16);
  motor->para.v_int_test = float_to_uint(0.0f, V_MIN1, V_MAX1, 12);
  motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN1, KP_MAX1, 12);
  motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN1, KD_MAX1, 12);
  motor->para.t_int_test = float_to_uint(0.0f, T_MIN1, T_MAX1, 12);
}

void dm4340_fbdata_init(Joint_Motor_t *motor) {
  motor->para.p_int_test = float_to_uint(0.0f, P_MIN2, P_MAX2, 16);
  motor->para.v_int_test = float_to_uint(0.0f, V_MIN2, V_MAX2, 12);
  motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN2, KP_MAX2, 12);
  motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN2, KD_MAX2, 12);
  motor->para.t_int_test = float_to_uint(0.0f, T_MIN2, T_MAX2, 12);
}

void dm6006_fbdata_init(Joint_Motor_t *motor) {
  motor->para.p_int_test = float_to_uint(0.0f, P_MIN3, P_MAX3, 16);
  motor->para.v_int_test = float_to_uint(0.0f, V_MIN3, V_MAX3, 12);
  motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN3, KP_MAX3, 12);
  motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN3, KD_MAX3, 12);
  motor->para.t_int_test = float_to_uint(0.0f, T_MIN3, T_MAX3, 12);
}

void dm8006_fbdata_init(Joint_Motor_t *motor) {
  motor->para.p_int_test = float_to_uint(0.0f, P_MIN4, P_MAX4, 16);
  motor->para.v_int_test = float_to_uint(0.0f, V_MIN4, V_MAX4, 12);
  motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN4, KP_MAX4, 12);
  motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN4, KD_MAX4, 12);
  motor->para.t_int_test = float_to_uint(0.0f, T_MIN4, T_MAX4, 12);
}

void dm3507_fbdata_init(Joint_Motor_t *motor) {
  motor->para.p_int_test = float_to_uint(0.0f, P_MIN5, P_MAX5, 16);
  motor->para.v_int_test = float_to_uint(0.0f, V_MIN5, V_MAX5, 12);
  motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN5, KP_MAX5, 12);
  motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN5, KD_MAX5, 12);
  motor->para.t_int_test = float_to_uint(0.0f, T_MIN5, T_MAX5, 12);
}
