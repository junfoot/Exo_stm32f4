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
#include "ADXL.h"

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

float adxl01_data[3];
float adxl02_data[3];

uint8_t adxl_select = 0;

extern float GAINX;
extern float GAINY;
extern float GAINZ;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask01 */
osThreadId_t myTask01Handle;
const osThreadAttr_t myTask01_attributes = {
  .name = "myTask01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
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
void StartTask01(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

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

  /* creation of myTask01 */
  myTask01Handle = osThreadNew(StartTask01, NULL, &myTask01_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

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

/* USER CODE BEGIN Header_StartTask01 */
/**
* @brief Function implementing the myTask01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* USER CODE BEGIN StartTask01 */
	
	// motor control
	
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
		
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
		
    osDelay(1000);
  }
  /* USER CODE END StartTask01 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	
	// serial port communication
	
  /* Infinite loop */
  for(;;)
  {
		
		osSemaphoreAcquire(sem1Handle, osWaitForever);
		
		printf("%f,%f,%f,%f,%f,%f\r\n", adxl01_data[0], adxl01_data[1], adxl01_data[2]
															,adxl02_data[0], adxl02_data[1], adxl02_data[2]);
		
//    osDelay(1000);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	
	// -------------------  spi communication  ------------------
 	// spi init
	
	ADXL_InitTypeDef adxl01 = {
		SPIMODE_4WIRE,
		INT_ACTIVEHIGH,
		LPMODE_NORMAL,
		BWRATE_3200,
		RANGE_16G,
		RESOLUTION_FULL,
		JUSTIFY_SIGNED,
		AUTOSLEEPOFF,
		LINKMODEOFF
	};
	
	adxlStatus adxlres;
	
	adxl_select = 0;
	adxlres = ADXL_Init(&adxl01);
	printf("adxl01_state: %d\r\n", adxlres);
	ADXL_Measure(ON);
	
	adxl_select = 1;
	adxlres = ADXL_Init(&adxl01);
	printf("adxl02_state: %d\r\n", adxlres);
	ADXL_Measure(ON);
	
  /* Infinite loop */
  for(;;)
  {
//		uint8_t data[6]={0,0,0,0,0,0};
//		uint8_t address = DATA0;
//		address |= 0x40;
//    address |= (0x80);	
//		
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
////		HAL_SPI_Transmit(&SPIhandler,&address, 1, HAL_MAX_DELAY);
////		HAL_SPI_Receive(&SPIhandler, data, sizeof(data), HAL_MAX_DELAY);
//		HAL_SPI_TransmitReceive(&SPIhandler, &address, data, sizeof(data), HAL_MAX_DELAY);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);

//		adxl01_data[0] = (float)(data[1]<<8 | data[0])*GAINX;
////		adxl01_data[0] = ( (int16_t) ((data[1]*256+data[0])))*GAINX;
//		adxl01_data[1] = ( (int16_t) ((data[3]*256+data[2])))*GAINY;
//		adxl01_data[2] = ( (int16_t) ((data[5]*256+data[4])))*GAINZ;
		
		// =============================================================
		
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
//		HAL_SPI_TransmitReceive(&SPIhandler, &address, data, sizeof(data), HAL_MAX_DELAY);
//		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
//		
//		float * fdata2 = adxl02_data;
//		fdata2[0] = ( (int16_t) ((data[1]*256+data[0])))*GAINX;
//		fdata2[1] = ( (int16_t) ((data[3]*256+data[2])))*GAINY;
//		fdata2[2] = ( (int16_t) ((data[5]*256+data[4])))*GAINZ;

		adxl_select = 0;
		ADXL_getAccel(adxl01_data, OUTPUT_FLOAT);
		
		osDelay(1);
		
		adxl_select = 1;
		ADXL_getAccel(adxl02_data, OUTPUT_FLOAT);

//		printf("%f\r\n", GAINX);
//		printf("%f,%f,%f,%f,%f,%f\r\n", adxl01_data[0], adxl01_data[1], adxl01_data[2]
//																	,adxl02_data[0], adxl02_data[1], adxl02_data[2]);
		
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

