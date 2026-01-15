#ifndef __BODY_TASK_H
#define __BODY_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "chassisL_task.h"
#include "INS_task.h"



typedef struct
{
  Joint_Motor_t arm_motor[8];
  Joint_Motor_t loin_motor;
	uint8_t start_flag;//∆Ù∂Ø±Í÷æ
	
} body_t;


extern void body_init(body_t *body);
extern void Body_task(void);




#endif




