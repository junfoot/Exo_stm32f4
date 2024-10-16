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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "semphr.h"
#include "tmotor_ak_actuators.h"
#include "can.h"
#include <stdio.h>

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
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorControl */
osThreadId_t MotorControlHandle;
const osThreadAttr_t MotorControl_attributes = {
  .name = "MotorControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for PCcom */
osThreadId_t PCcomHandle;
const osThreadAttr_t PCcom_attributes = {
  .name = "PCcom",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for sem1 */
osSemaphoreId_t sem1Handle;
const osSemaphoreAttr_t sem1_attributes = {
  .name = "sem1"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMotorControl(void *argument);
void StartPCcom(void *argument);

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
  /* creation of sem1 */
  sem1Handle = osSemaphoreNew(1, 0, &sem1_attributes);

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

  /* creation of MotorControl */
  MotorControlHandle = osThreadNew(StartMotorControl, NULL, &MotorControl_attributes);

  /* creation of PCcom */
  PCcomHandle = osThreadNew(StartPCcom, NULL, &PCcom_attributes);

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
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMotorControl */
/**
* @brief Function implementing the MotorControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControl */
void StartMotorControl(void *argument)
{
  /* USER CODE BEGIN StartMotorControl */
    
    // 初始化电机参数
    MotorParameters motorParams = {
        .positionMin = -12.5f,
        .positionMax = 12.5f,
        .velocityMin = -50.0f,
        .velocityMax = 50.0f,
        .torqueMin = -65.0f,
        .torqueMax = 65.0f,
        .kpMin = 0.0f,
        .kpMax = 500.0f,
        .kdMin = 0.0f,
        .kdMax = 5.0f,
    };

    // 初始化一个电机对象
    AkActuators motor1, motor2;
    AkActuators_init(&motor1, 1, motorParams, can_send_msg);
    AkActuators_init(&motor2, 2, motorParams, can_send_msg);

//    // 启动电机
//    AkActuators_enable(&motor1);
//    AkActuators_enable(&motor2);

//    // 控制电机移动到某一位置
//    float targetPosition = 5.0f;  // 目标位置 (弧度)
//    float targetVelocity = 10.0f; // 目标速度 (弧度/秒)
//    float targetTorque = 15.0f;   // 目标扭矩 (牛顿米)
//    float kp = 2.0f;            // PID 控制器的 kp
//    float kd = 0.2f;              // PID 控制器的 kd
//    AkActuators_move(&motor1, targetPosition, targetVelocity, targetTorque, kp, kd);

//    // 等待一段时间来模拟电机运行 (在实际使用中应使用延迟函数)
//    osDelay(3000);

//    // 关闭电机
//    AkActuators_disable(&motor1);
    
    
  /* Infinite loop */
  for(;;)
  {
      
//      osSemaphoreAcquire(sem1Handle, osWaitForever);
		
			printf("111\r\n");
		
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
      
      osDelay(500);
  }
  /* USER CODE END StartMotorControl */
}

/* USER CODE BEGIN Header_StartPCcom */
/**
* @brief Function implementing the PCcom thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPCcom */
void StartPCcom(void *argument)
{
  /* USER CODE BEGIN StartPCcom */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartPCcom */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

