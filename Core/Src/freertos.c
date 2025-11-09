/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"
#include "dj_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern bool write_flag;
extern int16_t control_speed;
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
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId_t WriteControlTaskHandle;
void WriteControlTask(void *argument);
const osThreadAttr_t WriteControlTask_attributes = {
    .name = "WriteControlTask",
    .stack_size = 95 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
osThreadId_t ActionTaskHandle;
void ActionTask(void *argument);
const osThreadAttr_t ActionTask_attributes = {
    .name = "ActionTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
  /* creation of defaultTask */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  WriteControlTaskHandle = osThreadNew(WriteControlTask, NULL, &WriteControlTask_attributes);
  ActionTaskHandle = osThreadNew(ActionTask, NULL, &ActionTask_attributes);
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
  for (;;)
  {
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void WriteControlTask(void *argument)
{
  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;)
  {
    if (start_flag && write_flag)
    {
      static int8_t left_or_right = 1;
      write_flag = 0;
      for (uint8_t i = 0; i < 2; i++)
      {
        DJMotorPtr motorp = &DJMotors[i];
        motorp->valueSet.speed = -(int16_t)signum(left_or_right) * control_speed;
        left_or_right = -left_or_right;
      }
    }
    /* 周期为 10ms，可根据需要调整 */
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
  }
}
void ActionTask(void *argument)
{
  TickType_t last_wake_time = xTaskGetTickCount();
  for (;;)
  {
    if (reachout_flag == true)
    {
      // DJMotors[2].status.findZeroDir = 0;
      // DJMotors[2].motorMode = Zero;
			control_speed = 1500;
			 write_flag = 1;
      DJMotors[2].valueSet.angle = -pos2angle(stroke_pos);
      reachout_flag = false;
    }
    else if (retract_flag == true)
    {
      // DJMotors[2].status.findZeroDir = 1;
      // DJMotors[2].motorMode = Zero;
			control_speed = 1500;
			 write_flag = 1;
      DJMotors[2].valueSet.angle = 0;
      retract_flag = false;
    }
    if (action_falg == true)
    {
      // DJMotors[2].status.findZeroDir = 0;
      // DJMotors[2].motorMode = Zero;
			control_speed = 1500;
			 write_flag = 1;
      DJMotors[2].valueSet.angle = -pos2angle(stroke_pos);
      osDelay(7500);
      // DJMotors[2].status.findZeroDir = 1;
      // DJMotors[2].motorMode = Zero;
      DJMotors[2].valueSet.angle = 0;
			osDelay(7500);
//			DJMotors[2].valueSet.angle = -pos2angle(stroke_pos);
//			osDelay(5000);
//			control_speed = -1000;
//			 write_flag = 1;
//			osDelay(3000);
//			DJMotors[2].valueSet.angle = 0;
			control_speed = 0;
			 write_flag = 1;
      action_falg = false;
    }

    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
  }
}
/* USER CODE END Application */
