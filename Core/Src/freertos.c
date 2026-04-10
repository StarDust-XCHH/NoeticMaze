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
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotionTask */
osThreadId_t MotionTaskHandle;
const osThreadAttr_t MotionTask_attributes = {
  .name = "MotionTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LidarRouteTask */
osThreadId_t LidarRouteTaskHandle;
const osThreadAttr_t LidarRouteTask_attributes = {
  .name = "LidarRouteTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for AlgorithmBrain */
osThreadId_t AlgorithmBrainHandle;
const osThreadAttr_t AlgorithmBrain_attributes = {
  .name = "AlgorithmBrain",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskPrint */
osThreadId_t TaskPrintHandle;
const osThreadAttr_t TaskPrint_attributes = {
  .name = "TaskPrint",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LidarQueue */
osMessageQueueId_t LidarQueueHandle;
const osMessageQueueAttr_t LidarQueue_attributes = {
  .name = "LidarQueue"
};
/* Definitions for PrintfMutex */
osMutexId_t PrintfMutexHandle;
const osMutexAttr_t PrintfMutex_attributes = {
  .name = "PrintfMutex"
};
/* Definitions for MapDataMutex */
osMutexId_t MapDataMutexHandle;
const osMutexAttr_t MapDataMutex_attributes = {
  .name = "MapDataMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void StartMotionTask(void *argument);
extern void StartImuTask(void *argument);
extern void StartLidarRouteTask(void *argument);
extern void StartAlgorithmBrain(void *argument);
extern void StartTaskPrint(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of PrintfMutex */
  PrintfMutexHandle = osMutexNew(&PrintfMutex_attributes);

  /* creation of MapDataMutex */
  MapDataMutexHandle = osMutexNew(&MapDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of LidarQueue */
  LidarQueueHandle = osMessageQueueNew (3, 4, &LidarQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotionTask */
  MotionTaskHandle = osThreadNew(StartMotionTask, NULL, &MotionTask_attributes);

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(StartImuTask, NULL, &ImuTask_attributes);

  /* creation of LidarRouteTask */
  LidarRouteTaskHandle = osThreadNew(StartLidarRouteTask, NULL, &LidarRouteTask_attributes);

  /* creation of AlgorithmBrain */
  AlgorithmBrainHandle = osThreadNew(StartAlgorithmBrain, NULL, &AlgorithmBrain_attributes);

  /* creation of TaskPrint */
  TaskPrintHandle = osThreadNew(StartTaskPrint, NULL, &TaskPrint_attributes);

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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

