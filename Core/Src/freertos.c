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
#include "body_task.h"
#include "chassisL_task.h"
#include "chassisR_task.h"
#include "cmsis_os.h"
#include "connect_task.h"
#include "main.h"
#include "task.h"
#include "vbus_check.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId CHASSISR_TASKHandle;
osThreadId CHASSISL_TASKHandle;
osThreadId CONNECT_TASKHandle;
osThreadId BODY_TASKHandle;
osThreadId VBUS_CHECK_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void ChassisR_Task(void const *argument);
void ChassisL_Task(void const *argument);
void CONNECT_Task(void const *argument);
void Body_Task(void const *argument);
void VBUS_CheckTask(void const *argument);

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of CHASSISR_TASK */
  osThreadDef(CHASSISR_TASK, ChassisR_Task, osPriorityHigh, 0, 512);
  CHASSISR_TASKHandle = osThreadCreate(osThread(CHASSISR_TASK), NULL);

  /* definition and creation of CHASSISL_TASK */
  osThreadDef(CHASSISL_TASK, ChassisL_Task, osPriorityHigh, 0, 512);
  CHASSISL_TASKHandle = osThreadCreate(osThread(CHASSISL_TASK), NULL);

  /* definition and creation of CONNECT_TASK */
  osThreadDef(CONNECT_TASK, CONNECT_Task, osPriorityHigh, 0, 512);
  CONNECT_TASKHandle = osThreadCreate(osThread(CONNECT_TASK), NULL);

  /* definition and creation of BODY_TASK */
  osThreadDef(BODY_TASK, Body_Task, osPriorityAboveNormal, 0, 512);
  BODY_TASKHandle = osThreadCreate(osThread(BODY_TASK), NULL);

  /* definition and creation of VBUS_CHECK_TASK */
  osThreadDef(VBUS_CHECK_TASK, VBUS_CheckTask, osPriorityNormal, 0, 128);
  VBUS_CHECK_TASKHandle = osThreadCreate(osThread(VBUS_CHECK_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
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
void CONNECT_Task(void const *argument) {
  /* USER CODE BEGIN CONNECT_Task */
  Connect_task();
  /* USER CODE END CONNECT_Task */
}

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

/* USER CODE END Application */
