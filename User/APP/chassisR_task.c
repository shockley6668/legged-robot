/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右腿的五个电机，都是DM4340，这五个电机挂载在can1总线上
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
#include "fdcan.h"
#include "cmsis_os.h"
													
chassis_t chassis_move;

uint32_t CHASSR_TIME=1;	

float my_kd2=0.0f;
float my_vel2=0.0f;
float my_kp2=0.0f;
float my_pos2=0.0f;
float my_tor2=0.0f;
int a=0;
uint8_t cc=1;

void ChassisR_task(void)
{

	osDelay(2000);
  chassis_move.start_flag=1;
  ChassisR_init(&chassis_move);

	dm4340_fbdata_init(&chassis_move.joint_motor[5]);
  dm4340_fbdata_init(&chassis_move.joint_motor[6]);
	dm4340_fbdata_init(&chassis_move.joint_motor[7]);
	dm4340_fbdata_init(&chassis_move.joint_motor[8]);
	dm4340_fbdata_init(&chassis_move.joint_motor[9]);
  
	while(1)
	{	
		if(chassis_move.start_flag==1)	
		{
			mit_ctrl_test(&hfdcan1,0x05,&chassis_move.joint_motor[5]);
			
			mit_ctrl_test(&hfdcan1,0x04,&chassis_move.joint_motor[6]);
			
			mit_ctrl_test(&hfdcan1,0x03,&chassis_move.joint_motor[7]);
			
			mit_ctrl_test(&hfdcan1,0x02,&chassis_move.joint_motor[8]);
			
			mit_ctrl_test(&hfdcan1,0x01,&chassis_move.joint_motor[9]);
		}
		else
		{ 
			mit_ctrl2(&hfdcan1,0x05, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right_pitch
				
			mit_ctrl2(&hfdcan1,0x04, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right_roll
		
		  mit_ctrl2(&hfdcan1,0x03, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right_yaw
			
			mit_ctrl2(&hfdcan1,0x02, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right_calf
			//void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
			mit_ctrl2(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right_foot			
		}
		if(a==1)
		{
		 //save_motor_zero(&hfdcan1,0x05, MIT_MODE);
//			osDelay(CHASSR_TIME);
		//	save_motor_zero(&hfdcan1,0x04, MIT_MODE);
//			osDelay(CHASSR_TIME);
		//	save_motor_zero(&hfdcan1,0x03, MIT_MODE);
//			osDelay(CHASSR_TIME);
		//	save_motor_zero(&hfdcan1,0x02, MIT_MODE);
//			osDelay(CHASSR_TIME);
			save_motor_zero(&hfdcan1,0x01, MIT_MODE);
			osDelay(CHASSR_TIME);			
		}
		osDelay(CHASSR_TIME);
	}
}

void ChassisR_init(chassis_t *chassis)
{
	joint_motor_init(&chassis->joint_motor[5],5,MIT_MODE);//发送id为5
	joint_motor_init(&chassis->joint_motor[6],4,MIT_MODE);//发送id为4
	
	joint_motor_init(&chassis->joint_motor[7],3,MIT_MODE);//发送id为3
	joint_motor_init(&chassis->joint_motor[8],2,MIT_MODE);//发送id为2
	joint_motor_init(&chassis->joint_motor[9],1,MIT_MODE);//发送id为1
	
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[5].para.id,chassis->joint_motor[5].mode);
	  osDelay(20);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[6].para.id,chassis->joint_motor[6].mode);
	  osDelay(20);
	}

	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->joint_motor[7].para.id,chassis->joint_motor[7].mode);
	  osDelay(20);
	}
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->joint_motor[8].para.id,chassis->joint_motor[8].mode);
	  osDelay(20);
	}
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->joint_motor[9].para.id,chassis->joint_motor[9].mode);
	  osDelay(20);
	}
}


void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}





