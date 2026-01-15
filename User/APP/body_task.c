/**
  *********************************************************************
  * @file      body_task.c/h
  * @brief     该任务控制腰部的一个电机，是DM6006，这个电机挂载在can3总线上
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "body_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "math.h"
#include "chassisR_task.h"


															
body_t robot_body;

uint32_t BODY_TIME=1;	

//float my_kp_shoulder=60.0f;
//float my_kd_shoulder=0.4f;
//float my_kp_arm=10.0f;
//float my_kd_arm=0.1f;
//float my_kp_loin=130.0f;
//float my_kd_loin=1.1f;
float my_kp_shoulder=0.0f;
float my_kd_shoulder=0.0f;
float my_kp_arm=0.0f;
float my_kd_arm=0.0f;
float my_kp_loin=0.0f;
float my_kd_loin=0.0f;
float my_pos3[9]={0.0f};
float my_pos3_set[9]={0.0f};

int c=0;
uint8_t record_flag=0;
FDCAN_ProtocolStatusTypeDef fdcan_protocol_status;
FDCAN_ErrorCountersTypeDef err_cnt;
void slope_following(float *target,float *set,float acc)
{
	if(*target > *set)
	{
		*set = *set + acc;
		if(*set >= *target)
		{
			*set = *target;
		}		
	}
	else if(*target < *set)
	{
		*set = *set - acc;
		if(*set <= *target)
		{
			*set = *target;
		}	
	}
}


uint8_t swing_flag=0;
uint64_t time=0;
void Body_task(void)
{
	osDelay(2000);
	robot_body.start_flag=1;
  body_init(&robot_body);
  dm4310_fbdata_init(&robot_body.arm_motor[0]);
  dm4310_fbdata_init(&robot_body.arm_motor[1]);
	dm3507_fbdata_init(&robot_body.arm_motor[2]);
	dm3507_fbdata_init(&robot_body.arm_motor[3]);
	
	dm4310_fbdata_init(&robot_body.arm_motor[4]);
  dm4310_fbdata_init(&robot_body.arm_motor[5]);
	dm3507_fbdata_init(&robot_body.arm_motor[6]);
	dm3507_fbdata_init(&robot_body.arm_motor[7]);
	
	dm6006_fbdata_init(&robot_body.loin_motor);
	
	while(1)
	{		
		if(robot_body.start_flag==1)	
		{		
				mit_ctrl_test(&hfdcan3,0x01,&robot_body.arm_motor[0]);		
//				mit_ctrl_test(&hfdcan3,0x02,&robot_body.arm_motor[1]);
//				mit_ctrl_test(&hfdcan3,0x03,&robot_body.arm_motor[2]);			
//				mit_ctrl_test(&hfdcan3,0x04,&robot_body.arm_motor[3]);
				pos_speed_ctrl(&hfdcan3,0x02, 0.0f, 0.3f);
				pos_speed_ctrl(&hfdcan3,0x03, 0.0f, 0.3f);
				pos_speed_ctrl(&hfdcan3,0x04, 0.0f, 0.3f);
			
				mit_ctrl_test(&hfdcan3,0x05,&robot_body.arm_motor[4]);
//				mit_ctrl_test(&hfdcan3,0x06,&robot_body.arm_motor[5]);			
//				mit_ctrl_test(&hfdcan3,0x07,&robot_body.arm_motor[6]);				
//				mit_ctrl_test(&hfdcan3,0x08,&robot_body.arm_motor[7]);
				pos_speed_ctrl(&hfdcan3,0x06, 0.0f, 0.3f);
				pos_speed_ctrl(&hfdcan3,0x07, 0.0f, 0.3f);
				pos_speed_ctrl(&hfdcan3,0x08, 0.0f, 0.3f);
				
				//mit_ctrl_test(&hfdcan3,0x09,&robot_body.loin_motor);
				pos_speed_ctrl(&hfdcan3,0x09, 0.0f, 0.3f);			
		}
		else
		{ //void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)		
			mit_ctrl(&hfdcan3,0x01, 0.0f, 0.0f,my_kp_shoulder,my_kd_shoulder, 0.0f);//4310
			mit_ctrl(&hfdcan3,0x02, 0.0f, 0.0f,my_kp_shoulder,my_kd_shoulder, 0.0f);//4310
			mit_ctrl5(&hfdcan3,0x03, 0.0f, 0.0f,my_kp_arm,my_kd_arm, 0.0f);//3507
			mit_ctrl5(&hfdcan3,0x04, 0.0f, 0.0f,0.0f,0.0f, 0.0f);//3507
//			pos_speed_ctrl(&hfdcan3,0x02, 0.0f, 0.3f);
//			pos_speed_ctrl(&hfdcan3,0x03, 0.0f, 0.3f);
//			pos_speed_ctrl(&hfdcan3,0x04, 0.0f, 0.3f);

			
			mit_ctrl(&hfdcan3,0x05, 0.0f, 0.0f,my_kp_shoulder,my_kd_shoulder, 0.0f);//4310
			mit_ctrl(&hfdcan3,0x06, 0.0f, 0.0f,my_kp_shoulder,my_kd_shoulder, 0.0f);//4310
			mit_ctrl5(&hfdcan3,0x07, 0.0f, 0.0f,my_kp_arm,my_kd_arm, 0.0f);//3507
			mit_ctrl5(&hfdcan3,0x08, 0.0f, 0.0f,0.0f,0.0f, 0.0f);//3507
//			pos_speed_ctrl(&hfdcan3,0x06, 0.0f, 0.3f);
//			pos_speed_ctrl(&hfdcan3,0x07, 0.0f, 0.3f);
//			pos_speed_ctrl(&hfdcan3,0x08, 0.0f, 0.3f);
			
			//腰电机
			mit_ctrl3(&hfdcan3,0x09, 0.0f, 0.0f,my_kp_loin, my_kd_loin,0.0f);//6006 腰电机		
			//pos_speed_ctrl(&hfdcan3,0x09, 0.0f, 0.3f);

		}
		if(c==1)
		{
		  save_motor_zero(&hfdcan3,0x04, MIT_MODE);
		  osDelay(BODY_TIME);			
		}
		
		osDelay(BODY_TIME);
	}
}

void body_init(body_t *body)
{
	//右臂
	joint_motor_init(&body->arm_motor[0],1,MIT_MODE);//shoulder pitch
	joint_motor_init(&body->arm_motor[1],2,MIT_MODE);//shoulder roll
	joint_motor_init(&body->arm_motor[2],3,MIT_MODE);//shoulder yaw
	joint_motor_init(&body->arm_motor[3],4,MIT_MODE);//arm pitch
	
	//左臂
	joint_motor_init(&body->arm_motor[4],5,MIT_MODE);//shoulder pitch
	joint_motor_init(&body->arm_motor[5],6,MIT_MODE);//shoulder roll
	joint_motor_init(&body->arm_motor[6],7,MIT_MODE);//shoulder yaw
	joint_motor_init(&body->arm_motor[7],8,MIT_MODE);//arm pitch
	
	joint_motor_init(&body->loin_motor,0x09,MIT_MODE);//腰电机
	
	
	
	//切换模式
	for(int j=0;j<5;j++)
	{
	 switch_control_mode(&hfdcan3, body->arm_motor[0].para.id,1);//1是mit模式编码
	 osDelay(3);
	}
	for(int j=0;j<5;j++)
	{
	 switch_control_mode(&hfdcan3, body->arm_motor[1].para.id,2);//1是mit模式编码
	 osDelay(3);
	}
	for(int j=0;j<5;j++)
	{
	 switch_control_mode(&hfdcan3, body->arm_motor[2].para.id,2);//1是mit模式编码
	 osDelay(3);
	}
	for(int j=0;j<5;j++)
	{
	 switch_control_mode(&hfdcan3, body->arm_motor[3].para.id,2);//1是mit模式编码
	 osDelay(3);
	}
	
	for(int j=0;j<5;j++)
	{
	 switch_control_mode(&hfdcan3, body->arm_motor[4].para.id,1);//1是mit模式编码
	 osDelay(3);
	}
	for(int j=0;j<5;j++)
	{
	 switch_control_mode(&hfdcan3, body->arm_motor[5].para.id,2);//1是mit模式编码
	 osDelay(3);
	}
	for(int j=0;j<5;j++)
	{
	 switch_control_mode(&hfdcan3, body->arm_motor[6].para.id,2);//1是mit模式编码
	 osDelay(3);
	}
	for(int j=0;j<5;j++)
	{
	 switch_control_mode(&hfdcan3, body->arm_motor[7].para.id,2);//1是mit模式编码
	 osDelay(3);
	}

	for(int j=0;j<5;j++)
	{
	 switch_control_mode(&hfdcan3, body->loin_motor.para.id,2);//1是mit模式编码
	 osDelay(3);
	}
	
	
	
	//右臂
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan3,body->arm_motor[0].para.id,body->arm_motor[0].mode);
	  osDelay(20);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan3,body->arm_motor[1].para.id,body->arm_motor[1].mode);
	  osDelay(20);
	}
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan3,body->arm_motor[2].para.id,body->arm_motor[2].mode);
	  osDelay(20);
	}
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan3,body->arm_motor[3].para.id,body->arm_motor[3].mode);
	  osDelay(20);
	}
	
	//左臂
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan3,body->arm_motor[4].para.id,body->arm_motor[4].mode);
	  osDelay(20);
	}	
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan3,body->arm_motor[5].para.id,body->arm_motor[5].mode);
	  osDelay(20);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan3,body->arm_motor[6].para.id,body->arm_motor[6].mode);
	  osDelay(20);
	}
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan3,body->arm_motor[7].para.id,body->arm_motor[7].mode);
	  osDelay(20);
	}

	//腰电机
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan3,body->loin_motor.para.id,body->loin_motor.mode);
	  osDelay(20);
	}
}








