#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm4310_drv.h"

typedef struct
{
    float myPithR;
    uint8_t recover_flag;
    float leg_set;
    float last_leg_set;
    uint8_t prejump_flag;
    uint8_t jump_flag;
    uint8_t jump_flag2;
    float v_set;
    float x_set;
    float x_filter;
    float turn_set;
    float total_yaw;
    float roll_set;
    uint8_t start_flag;
    
    Joint_Motor_t joint_motor[10];

} chassis_t;

extern void ChassisR_task(void);
extern void ChassisR_init(chassis_t *chassis);
extern void mySaturate(float *in, float min, float max);

#endif
