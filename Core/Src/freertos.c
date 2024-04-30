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
#include "usart.h"
#include "tim.h"
#include "i2c.h"

#include "../Lib/timer.h"
#include "../Lib/gps.h"
#include "../Lib/imu.h"
#include "../Lib/blackbox.h"
#include "../Lib/sensordetect.h"

#include "../Driver/ibus.h"
#include "../Driver/mpu6050.h"
#include "../Driver/interrupt.h"
#include "../Lib/compass.h"
#include "../Lib/pwm.h"

#include "../flight/plane.h"


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
osThreadId task1Handle;
osThreadId task2Handle;
osThreadId task3Handle;
osThreadId task4Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ahrs_task(void const * argument);
void blackbox(void const * argument);
void led_indicate(void const * argument);
void read_sensor(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	 timer_start(&htim7);
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
  /* definition and creation of task1 */
  osThreadDef(task1, ahrs_task, osPriorityHigh, 0, 200);
  task1Handle = osThreadCreate(osThread(task1), NULL);

  /* definition and creation of task2 */
  osThreadDef(task2, blackbox, osPriorityLow, 0, 200);
  task2Handle = osThreadCreate(osThread(task2), NULL);

  /* definition and creation of task3 */
  osThreadDef(task3, led_indicate, osPriorityLow, 0, 128);
  task3Handle = osThreadCreate(osThread(task3), NULL);

  /* definition and creation of task4 */
  osThreadDef(task4, read_sensor, osPriorityRealtime, 0, 128);
  task4Handle = osThreadCreate(osThread(task4), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_ahrs_task */
float dt;
int16_t gyro_imu[3];
int16_t acc_imu[3];
int16_t mag_raw[3];
uint16_t stack_task_ahrs;
/**
  * @brief  Function implementing the task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ahrs_task */
void ahrs_task(void const * argument)
{
  /* USER CODE BEGIN ahrs_task */
	uint32_t last_call = micros();
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10;
	xLastWakeTime = xTaskGetTickCount();
	ibus_init(&huart1);
	gps_init(&huart3,57600);
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); // for debug

    dt = (micros() - last_call)*(1e-6);
    last_call = micros();

    ibusFrameComplete();
    update_ahrs(gyro_imu[0],gyro_imu[1],gyro_imu[2],acc_imu[0],acc_imu[1],acc_imu[2],mag_raw[0],mag_raw[1],mag_raw[2],dt);
    vTaskSuspend(NULL);
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    stack_task_ahrs = uxTaskGetStackHighWaterMark( NULL );

  }
  /* USER CODE END ahrs_task */
}

/* USER CODE BEGIN Header_blackbox */

black_box_file_t fdata;
uint16_t stack_task_blackbox;
/**
* @brief Function implementing the task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_blackbox */
void blackbox(void const * argument)
{
  /* USER CODE BEGIN blackbox */
  //vTaskSuspend(task1Handle);
  //vTaskSuspend(NULL);
  black_box_init();
  black_box_create_file(&fdata,"fdata.txt");
  //vTaskResume(task1Handle);

  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
		black_box_pack_int(&fdata,(int)AHRS.roll);
		black_box_pack_str(&fdata," ");

		black_box_pack_int(&fdata,(int)AHRS.pitch);
		black_box_pack_str(&fdata," ");

		black_box_pack_int(&fdata,(int)AHRS.yaw);
		black_box_pack_str(&fdata," ");

		black_box_pack_int(&fdata,(int)(millis()/1000));
		black_box_pack_str(&fdata,"\n");

		black_box_load(&fdata);

		black_box_sync(&fdata);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);


	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    osDelay(1000);
    stack_task_blackbox = uxTaskGetStackHighWaterMark( NULL );
  }
  /* USER CODE END blackbox */
}

/* USER CODE BEGIN Header_led_indicate */
/**
* @brief Function implementing the task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_indicate */
void led_indicate(void const * argument)
{
  /* USER CODE BEGIN led_indicate */
  /* Infinite loop */
  for(;;)
  {
	static uint32_t delay;
	if(fdata.file.err != -1 && ibusChannelData[CH10] > 1700){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		delay = 100;
	}
	else{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		delay = 500;
	}
    osDelay(delay);
  }
  /* USER CODE END led_indicate */
}

/* USER CODE BEGIN Header_read_sensor */
uint16_t stack_task_sensor;
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_read_sensor */
void read_sensor(void const * argument)
{
  /* USER CODE BEGIN readIMU */
 int16_t gyso_offset[3] = {0,0,0};
 axis3_t raw;
 uint8_t sample_count = 0;
 int32_t gyro_add[3] = {0,0,0};
 uint8_t first_loop = 1;
 compassInit();
 mpu6050_init(&hi2c2);
 HAL_Delay(2000);
 imu_calibrate(&gyso_offset[0],&gyso_offset[1],&gyso_offset[2]);

 TickType_t xLastWakeTime;
 const TickType_t xFrequency = 2;
 xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	mpu6050_gyro_get_raw(&raw);
	gyro_add[0] += (raw.x - gyso_offset[0]);
	gyro_add[1] += (raw.y - gyso_offset[1]);
	gyro_add[2] += (raw.z - gyso_offset[2]);
	sample_count ++;
	if(sample_count >= 5){
	   axis3_t mag;
	   compass_get(&mag);
	   mag_raw[0] = mag.x;
	   mag_raw[1] = mag.y;
	   mag_raw[2] = mag.z;

	   gyro_imu[0] = (int16_t)(gyro_add[0]/5);
	   gyro_imu[1] = (int16_t)(gyro_add[1]/5);
	   gyro_imu[2] = (int16_t)(gyro_add[2]/5);
	   gyro_add[0] = 0;
	   gyro_add[1] = 0;
	   gyro_add[2] = 0;
	   sample_count = 0;
	   vTaskResume(task1Handle);
	}

	raw.x = 0;
	raw.y = 0;
	raw.z = 0;

	mpu6050_acc_get_raw(&raw);
	if(first_loop){
		acc_imu[0] = raw.x;
		acc_imu[1] = raw.y;
		acc_imu[2] = raw.z;
		first_loop = 0;
	}
	// low pass filter
	acc_imu[0] += 0.1*(raw.x - acc_imu[0]);
	acc_imu[1] += 0.1*(raw.y - acc_imu[1]);
	acc_imu[2] += 0.1*(raw.z - acc_imu[2]);
    stack_task_sensor = uxTaskGetStackHighWaterMark( NULL );
	vTaskDelayUntil( &xLastWakeTime, xFrequency );
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3); // debug
  }
  /* USER CODE END readIMU */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
