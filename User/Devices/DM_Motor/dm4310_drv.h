#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

//4310
#define P_MIN1 -12.5f
#define P_MAX1 12.5f
#define V_MIN1 -30.0f
#define V_MAX1 30.0f
#define KP_MIN1 0.0f
#define KP_MAX1 500.0f
#define KD_MIN1 0.0f
#define KD_MAX1 5.0f
#define T_MIN1 -10.0f
#define T_MAX1 10.0f

//4340
#define P_MIN2 -12.5f
#define P_MAX2 12.5f
#define V_MIN2 -10.0f
#define V_MAX2 10.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -28.0f
#define T_MAX2 28.0f

//6006
#define P_MIN3 -12.5f
#define P_MAX3 12.5f
#define V_MIN3 -45.0f
#define V_MAX3 45.0f
#define KP_MIN3 0.0f
#define KP_MAX3 500.0f
#define KD_MIN3 0.0f
#define KD_MAX3 5.0f
#define T_MIN3 -12.0f
#define T_MAX3 12.0f

//8006
#define P_MIN4 -12.5f
#define P_MAX4 12.5f
#define V_MIN4 -45.0f
#define V_MAX4 45.0f
#define KP_MIN4 0.0f
#define KP_MAX4 500.0f
#define KD_MIN4 0.0f
#define KD_MAX4 5.0f
#define T_MIN4 -20.0f
#define T_MAX4 20.0f

//小关节3507
#define P_MIN5 -12.5f
#define P_MAX5 12.5f
#define V_MIN5 -50.0f
#define V_MAX5 50.0f
#define KP_MIN5 0.0f
#define KP_MAX5 500.0f
#define KD_MIN5 0.0f
#define KD_MAX5 5.0f
#define T_MIN5 -10.0f
#define T_MAX5 10.0f

typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float kp;
	float kd;
	float Tmos;
	float Tcoil;
	
	float kp_test;
	float kd_test;
	float tor_set;
	float pos_set;
	float vel_set;
	int kp_int_test;
	int kd_int_test;
	int p_int_test;
	int v_int_test;
	int t_int_test;
}motor_fbpara_t;


typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
}Joint_Motor_t ;

typedef struct
{
	uint16_t mode;
	float wheel_T;//轮毂电机的输出扭矩，单位为N
	
	motor_fbpara_t para;	
}Wheel_Motor_t ;


extern void dm6006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void dm4340_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void dm8006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void dm3507_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);

extern void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

//4310
extern void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
extern void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
extern void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);

//4340
extern void mit_ctrl2(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
//6006
extern void mit_ctrl3(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
//8006
extern void mit_ctrl4(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
//3507
extern void mit_ctrl5(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);


extern void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode);
extern void wheel_motor_init(Wheel_Motor_t *motor,uint16_t id,uint16_t mode);
	
extern float Hex_To_Float(uint32_t *Byte,int num);//十六进制到浮点数
extern uint32_t FloatTohex(float HEX);//浮点数到十六进制转换

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);


extern void save_motor_zero(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void switch_control_mode(hcan_t* hcan, uint16_t motor_id,uint8_t mode);

extern void dm3507_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);
extern void dm4310_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);
extern void dm4340_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);
extern void dm6006_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);
extern void dm8006_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data);

extern void mit_ctrl_test(hcan_t* hcan, uint16_t motor_id,Joint_Motor_t *motor);

extern void dm4310_fbdata_init(Joint_Motor_t *motor);
extern void dm4340_fbdata_init(Joint_Motor_t *motor);
extern void dm6006_fbdata_init(Joint_Motor_t *motor);
extern void dm8006_fbdata_init(Joint_Motor_t *motor);
extern void dm3507_fbdata_init(Joint_Motor_t *motor);

#endif /* __DM4310_DRV_H__ */



