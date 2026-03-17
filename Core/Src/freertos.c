/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "INS_task.h"
#include "body_task.h"
#include "chassisL_task.h"
#include "chassisR_task.h"
#include "protocol_task.h"
#include "vbus_check.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId defaultTaskHandle;
osThreadId CHASSISR_TASKHandle;
osThreadId CHASSISL_TASKHandle;
osThreadId BODY_TASKHandle;
osThreadId VBUS_CHECK_TASKHandle;
osThreadId PROTOCOL_TASKHandle;
osThreadId USB_TX_TASKHandle;
osThreadId INS_TASKHandle;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void const *argument);
void ChassisR_Task(void const *argument);
void ChassisL_Task(void const *argument);
void Body_Task(void const *argument);
void VBUS_CheckTask(void const *argument);
void Protocol_task(void const *argument);
void USB_Tx_task(void const *argument);
void INS_Task(void const *argument);
/* USER CODE END FunctionPrototypes */

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 队列已经由你在 main.c 中安全建立 */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of CHASSISR_TASK */
  osThreadDef(CHASSISR_TASK, ChassisR_Task, osPriorityHigh, 0, 1024);
  CHASSISR_TASKHandle = osThreadCreate(osThread(CHASSISR_TASK), NULL);

  /* definition and creation of CHASSISL_TASK */
  osThreadDef(CHASSISL_TASK, ChassisL_Task, osPriorityHigh, 0, 1024);
  CHASSISL_TASKHandle = osThreadCreate(osThread(CHASSISL_TASK), NULL);

  /* definition and creation of BODY_TASK */
  // osThreadDef(BODY_TASK, Body_Task, osPriorityAboveNormal, 0, 512);
  // BODY_TASKHandle = osThreadCreate(osThread(BODY_TASK), NULL);

  /* definition and creation of VBUS_CHECK_TASK */
  osThreadDef(VBUS_CHECK_TASK, VBUS_CheckTask, osPriorityNormal, 0, 128);
  VBUS_CHECK_TASKHandle = osThreadCreate(osThread(VBUS_CHECK_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* PROTOCOL_TASK for batch sending motor feedback at 100Hz */
  // osThreadDef(PROTOCOL_TASK, Protocol_task, osPriorityHigh, 0,
  //             1024); // Reduced stack
  // PROTOCOL_TASKHandle = osThreadCreate(osThread(PROTOCOL_TASK), NULL);

  /* definition and creation of USB_TX_TASK */
  osThreadDef(USB_TX_TASK, USB_Tx_task, osPriorityRealtime, 0, 4096);
  USB_TX_TASKHandle = osThreadCreate(osThread(USB_TX_TASK), NULL);

  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, INS_Task, osPriorityNormal, 0, 1024);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ChassisR_Task */
/**
 * @brief Function implementing the CHASSISR_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ChassisR_Task */
void ChassisR_Task(void const *argument) {
  /* USER CODE BEGIN ChassisR_Task */
  ChassisR_task();
  /* USER CODE END ChassisR_Task */
}

/* USER CODE BEGIN Header_ChassisL_Task */
/**
 * @brief Function implementing the CHASSISL_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ChassisL_Task */
void ChassisL_Task(void const *argument) {
  /* USER CODE BEGIN ChassisL_Task */
  ChassisL_task();
  /* USER CODE END ChassisL_Task */
}

/* USER CODE BEGIN Header_CONNECT_Task */
/**
 * @brief Function implementing the CONNECT_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CONNECT_Task */

/* USER CODE BEGIN Header_Body_Task */
/**
 * @brief Function implementing the BODY_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Body_Task */
void Body_Task(void const *argument) {
  /* USER CODE BEGIN Body_Task */
  Body_task();
  /* USER CODE END Body_Task */
}

/* USER CODE BEGIN Header_VBUS_CheckTask */
/**
 * @brief Function implementing the VBUS_CHECK_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_VBUS_CheckTask */
void VBUS_CheckTask(void const *argument) {
  /* USER CODE BEGIN VBUS_CheckTask */
  VBUS_Check_task();
  /* USER CODE END VBUS_CheckTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Protocol_task(void const *argument) { Protocol_task_entry(argument); }
void USB_Tx_task(void const *argument) { USB_Tx_task_entry(argument); }
void INS_Task(void const *argument) { INS_task(); }
/* USER CODE END Application */
