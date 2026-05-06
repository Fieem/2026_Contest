/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task_public.h"
#include "algorithm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for INSTask_rtos */
osThreadId_t INSTask_rtosHandle;
uint32_t INSTasklBuffer[ 128 ];
osStaticThreadDef_t INSTaskBlock;
const osThreadAttr_t INSTask_rtos_attributes = {
  .name = "INSTask_rtos",
  .cb_mem = &INSTaskBlock,
  .cb_size = sizeof(INSTaskBlock),
  .stack_mem = &INSTasklBuffer[0],
  .stack_size = sizeof(INSTasklBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ReceiveTask */
osThreadId_t ReceiveTaskHandle;
uint32_t myTask03Buffer[ 256 ];
osStaticThreadDef_t myTask03ControlBlock;
const osThreadAttr_t ReceiveTask_attributes = {
  .name = "ReceiveTask",
  .cb_mem = &myTask03ControlBlock,
  .cb_size = sizeof(myTask03ControlBlock),
  .stack_mem = &myTask03Buffer[0],
  .stack_size = sizeof(myTask03Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GimbalTask */
osThreadId_t GimbalTaskHandle;
uint32_t myTask04Buffer[ 256 ];
osStaticThreadDef_t myTask04ControlBlock;
const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .cb_mem = &myTask04ControlBlock,
  .cb_size = sizeof(myTask04ControlBlock),
  .stack_mem = &myTask04Buffer[0],
  .stack_size = sizeof(myTask04Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ScreenTask */
osThreadId_t ScreenTaskHandle;
uint32_t myTask05Buffer[ 256 ];
osStaticThreadDef_t myTask05ControlBlock;
const osThreadAttr_t ScreenTask_attributes = {
  .name = "ScreenTask",
  .cb_mem = &myTask05ControlBlock,
  .cb_size = sizeof(myTask05ControlBlock),
  .stack_mem = &myTask05Buffer[0],
  .stack_size = sizeof(myTask05Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TTSTask */
osThreadId_t TTSTaskHandle;
uint32_t myTask06Buffer[ 128 ];
osStaticThreadDef_t myTask06ControlBlock;
const osThreadAttr_t TTSTask_attributes = {
  .name = "TTSTask",
  .cb_mem = &myTask06ControlBlock,
  .cb_size = sizeof(myTask06ControlBlock),
  .stack_mem = &myTask06Buffer[0],
  .stack_size = sizeof(myTask06Buffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for imuBinarySem01 */
osSemaphoreId_t imuBinarySem01Handle;
const osSemaphoreAttr_t imuBinarySem01_attributes = {
  .name = "imuBinarySem01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMU_task(void *argument);
extern void StartReceiveTask(void *argument);
extern void StartGimbalTask(void *argument);
extern void StartScreenTask(void *argument);
extern void StartTTSTask(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of imuBinarySem01 */
  imuBinarySem01Handle = osSemaphoreNew(1, 0, &imuBinarySem01_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of INSTask_rtos */
  INSTask_rtosHandle = osThreadNew(IMU_task, NULL, &INSTask_rtos_attributes);

  /* creation of ReceiveTask */
  ReceiveTaskHandle = osThreadNew(StartReceiveTask, NULL, &ReceiveTask_attributes);

  /* creation of GimbalTask */
  GimbalTaskHandle = osThreadNew(StartGimbalTask, NULL, &GimbalTask_attributes);

  /* creation of ScreenTask */
  ScreenTaskHandle = osThreadNew(StartScreenTask, NULL, &ScreenTask_attributes);

  /* creation of TTSTask */
  TTSTaskHandle = osThreadNew(StartTTSTask, NULL, &TTSTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  vofa_start();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMU_task */
/**
* @brief Function implementing the INSTask_rtos thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_task */
__weak void IMU_task(void *argument)
{
  /* USER CODE BEGIN IMU_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

