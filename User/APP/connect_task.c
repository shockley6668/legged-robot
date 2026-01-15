/**
  *********************************************************************
  * @file      connect_task.c/h
  * @brief     该任务是与电脑通信
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "connect_task.h"
#include "body_task.h"
#include "cmsis_os.h"
#include "bsp_usart1.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

extern UART_HandleTypeDef huart1;
extern send_data_t send_data;
														 															 

extern chassis_t chassis_move;																 															 														 
extern body_t robot_body;

uint32_t CONNECT_TIME=1;//任务周期是1ms	
															 
void 	Connect_task(void)
{
  while(1)
	{  	
		send_data.tx[0]=FRAME_HEADER;
		for(int i=0;i<10;i++)
		{
			send_data.tx[1+i*5]=chassis_move.joint_motor[i].para.p_int>>8;
			send_data.tx[2+i*5]=chassis_move.joint_motor[i].para.p_int;
			send_data.tx[3+i*5]=chassis_move.joint_motor[i].para.v_int>>4;
			send_data.tx[4+i*5]=((chassis_move.joint_motor[i].para.v_int&0x0F)<<4)|(chassis_move.joint_motor[i].para.t_int>>8);
			send_data.tx[5+i*5]=chassis_move.joint_motor[i].para.t_int;
		}
		
		#if USE_ARM
			for(int i=10;i<11;i++)
			{//左臂
				send_data.tx[1+i*5]=robot_body.arm_motor[4].para.p_int>>8;
				send_data.tx[2+i*5]=robot_body.arm_motor[4].para.p_int;
				send_data.tx[3+i*5]=robot_body.arm_motor[4].para.v_int>>4;
				send_data.tx[4+i*5]=((robot_body.arm_motor[4].para.v_int&0x0F)<<4)|(robot_body.arm_motor[4].para.t_int>>8);
				send_data.tx[5+i*5]=robot_body.arm_motor[4].para.t_int;
			}
			for(int i=11;i<12;i++)
			{//右臂
				send_data.tx[1+i*5]=robot_body.arm_motor[0].para.p_int>>8;
				send_data.tx[2+i*5]=robot_body.arm_motor[0].para.p_int;
				send_data.tx[3+i*5]=robot_body.arm_motor[0].para.v_int>>4;
				send_data.tx[4+i*5]=((robot_body.arm_motor[0].para.v_int&0x0F)<<4)|(robot_body.arm_motor[0].para.t_int>>8);
				send_data.tx[5+i*5]=robot_body.arm_motor[0].para.t_int;
			}
		#endif
			
		send_data.tx[SEND_DATA_SIZE-1]=Check_Sum(SEND_DATA_SIZE-1,send_data.tx); 
		
		//HAL_UART_Transmit_DMA(&huart1, (uint8_t *)send_data.tx, sizeof(send_data.tx));
		
		CDC_Transmit_HS((uint8_t *)send_data.tx,sizeof(send_data.tx));
		
	  osDelay(CONNECT_TIME);
	}
}


