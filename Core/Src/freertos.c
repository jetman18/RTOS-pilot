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
#include "stdlib.h"
#include "usart.h"
#include "tim.h"
#include "i2c.h"
#include "stdio.h"

#include "../Lib/timer.h"
#include "../Lib/gps.h"
#include "../Lib/imu.h"
#include "../Lib/sensordetect.h"
#include "../Lib/compass.h"
#include "../Lib/pwm.h"
#include "../Lib/maths.h"
#include "../Lib/blackbox.h"
#include "../Lib/baro.h"
#include "../Lib/timer.h"


#include "../Driver/ibus.h"
#include "../Driver/mpu6050.h"
#include "../Driver/interrupt.h"
#include "../Driver/ms5611.h"
#include "../Driver/bmp280.h"

#include "../flight/plane.h"

#include "../HIL/dynamic_mode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
}
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

void blackbox_task(void const * argument);
void ahrs_task(void const * argument);
void sensor_task(void const * argument);
void osd_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	timer_start(&htim4);
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
  osThreadDef(task1, blackbox_task, osPriorityLow, 0, 512);
  task1Handle = osThreadCreate(osThread(task1), NULL);

  /* definition and creation of task2 */
  osThreadDef(task2, ahrs_task, osPriorityHigh, 0, 512);
  task2Handle = osThreadCreate(osThread(task2), NULL);

  /* definition and creation of task3 */
  osThreadDef(task3, sensor_task, osPriorityRealtime, 0, 512);
  task3Handle = osThreadCreate(osThread(task3), NULL);

  /* definition and creation of task4 */
  osThreadDef(task4, osd_task, osPriorityLow, 0, 512);
  task4Handle = osThreadCreate(osThread(task4), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_blackbox_task */
#define STACK_DEBUG
#ifdef STACK_DEBUG
uint16_t stack_task_ahrs;
uint16_t stack_task_sensor;
uint16_t stack_task_mavOSD;
uint16_t stack_task_blackbox;
#endif
/***************************/

extern float roll_desired;
extern float pitch_desired;
extern float ab_speed_filted;

extern float pid_roll_velo_scaler;
extern float pid_pitch_velo_scaler;

extern int32_t puts_state;
extern uint16_t servoL,servoR;
uint32_t sdcard_fsize;
uint8_t black_box_reset;
extern int32_t baro_alt;
extern int32_t baro_climb;

extern float velocity_test;
extern float position_test;
/**
  * @brief  Function implementing the task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_blackbox_task */
void blackbox_task(void const * argument)
{
  /* USER CODE BEGIN blackbox_task */
  /* Infinite loop */
	//vTaskSuspend(NULL);
		black_box_init();
		black_box_reset = TRUE;
		TickType_t xLastWakeTime;
		const TickType_t xFrequency = 100;  // 
		xLastWakeTime = xTaskGetTickCount();
	  /* Infinite loop */
	  for(;;)
	  {
			static uint32_t start_time_ms;
			if(black_box_reset){
				black_box_pack_str("----------------------------new data----------------------------------------------------------------\n");
				black_box_reset = FALSE;
				start_time_ms = millis();
			}
			uint32_t time_ms =  millis() - start_time_ms;
			// control thortle 0 -> 100%
			int throtle = ((int)ibusChannelData[CH3] - 1000)*0.1;

			// tx signal 0 -> 100 %
			//int srri = ((int)ibusChannelData[CH11] - 1000)*0.1;

			/*** write time  ***/
			black_box_pack_int(time_ms);
			black_box_pack_char(' ');

			/*---- control parameters ---*/
			black_box_pack_int((int)servoL);
			black_box_pack_char(' ');
			black_box_pack_int((int)servoR);
			black_box_pack_char(' ');
			black_box_pack_int(throtle);
			black_box_pack_char(' ');
#ifndef HIL
			black_box_pack_int(srri);
			black_box_pack_char(' ');
#endif
			/*----- attitude ---------------------*/
			black_box_pack_int((int)(AHRS.roll*100));
			black_box_pack_char(' ');
			black_box_pack_int((int)(roll_desired*100));
			black_box_pack_char(' ');
			black_box_pack_int((int)(AHRS.p*100));
			black_box_pack_char(' ');
			black_box_pack_int((int)(AHRS.roll_rate*100));
			black_box_pack_char(' ');

			black_box_pack_int((int)(AHRS.pitch*100));// cm
			black_box_pack_char(' ');
			black_box_pack_int((int)(pitch_desired*100));
			black_box_pack_char(' ');
			black_box_pack_int((int)(AHRS.q*100));
			black_box_pack_char(' ');
			black_box_pack_int((int)(AHRS.pitch_rate*100));
			black_box_pack_char(' ');

			black_box_pack_int((int)(AHRS.yaw*100));
			black_box_pack_char(' ');
			black_box_pack_int((int)(AHRS.r*100));
			black_box_pack_char(' ');
			black_box_pack_int((int)(AHRS.yaw_rate*100));
			black_box_pack_char(' ');

			/*------- GPS ----------------------*/
#ifndef HIL
			int16_t vx = _gps.velocity[0];  // cm/s
			int16_t vy = _gps.velocity[1];  // cm/s
			int16_t vz = _gps.velocity[2];  // cm/s
			int32_t ground_speed = sqrt(sq(vx) + sq(vy)) ;

			black_box_pack_int(_gps.position[0]);
			black_box_pack_char(' ');
			black_box_pack_int(_gps.position[1]);
			black_box_pack_char(' ');
			black_box_pack_int(_gps.altitude_msl);
			black_box_pack_char(' ');
			black_box_pack_int(_gps.altitude_mgl);
			black_box_pack_char(' ');
			//black_box_pack_int(_gps.numSat);
			//lack_box_pack_char(' ');
			black_box_pack_int(_gps.fix);
			black_box_pack_char(' ');
			black_box_pack_int(ground_speed);
			black_box_pack_char(' ');
			black_box_pack_int((int)(pid_roll_velo_scaler*100)); 
			black_box_pack_char(' ');
			black_box_pack_int((int)(pid_pitch_velo_scaler*100)); 
			black_box_pack_char(' ');
			black_box_pack_int(vz);
			black_box_pack_char(' ');
			black_box_pack_int(baro_climb);   // cm
			black_box_pack_char(' ');
			black_box_pack_int(baro_alt);   // cm
#endif

			/*----- end line && load data to sd card- -----*/
			//sdcard_fsize = black_box_get_file_size();
			black_box_pack_char('\n');
			black_box_load();
            // if write ok
			if(puts_state != -1){
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			}
		vTaskDelayUntil( &xLastWakeTime, xFrequency);
#ifdef STACK_DEBUG
	    stack_task_blackbox = uxTaskGetStackHighWaterMark( NULL );
#endif
	  }
  /* USER CODE END blackbox_task */
}

/* USER CODE BEGIN Header_ahrs_task */
#ifndef HIL
int16_t gyro_imu[3];
int16_t acc_imu[3];
int16_t mag_raw[3];
#endif

int *k;
uint32_t ff;
int kj[10];
/**
* @brief Function implementing the task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ahrs_task */
void ahrs_task(void const * argument)
{
  /* USER CODE BEGIN ahrs_task */
  /* Infinite loop */
	//vTaskSuspend(NULL);
	attitude_ctrl_init();
#ifndef HIL
	initPWM(&htim3);
	compassInit();
	gps_init(&huart3,38400);
	baro_init();
#endif
	mavlinkInit(1,1,&huart1,115200);
	ibus_init(&huart2);

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10; // 100 hz loop
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
#ifndef HIL
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); // for debugging
	//timer_calculate_boottime();
	if(is_baro_calibration() == FALSE){
		baro_zero_calibrate();
	}else{
		baro_update(0.01);
	}
	// get rc channel 
	ibusFrameComplete();
	// gps
	gps_readout();
	update_ahrs(gyro_imu[0],gyro_imu[1],gyro_imu[2],acc_imu[0],acc_imu[1],acc_imu[2],mag_raw[0],mag_raw[1],mag_raw[2],0.01);
	attitude_ctrl_start(0.01);
#endif
	ibusFrameComplete();
    // start dynamic mode
	uint16_t thrust = ibusChannelData[CH3];
	uint16_t servoLL = ibusChannelData[CH2];
	uint16_t servoRR = ibusChannelData[CH1];
	dynamic_control(thrust,servoLL,servoRR);
	dynamic_loop(0.01);
    uint32_t *k = (uint32_t*)0x20021000;
    *k +=1;
    if(k){

    }
/*
	if(ibusChannelData[CH10] > CHANNEL_HIGH && ibusChannelData[CH5] < CHANNEL_HIGH){
		static uint32_t tim_;
		if(millis() - tim_ > 200){
            //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);

			tim_ = millis();
		}
	}else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,0);
	}
*/

	static uint32_t gps_tim_ms;
	if(millis() - gps_tim_ms > 200){
	   if(_gps.fix > 1){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	   }
	   gps_tim_ms = millis();
	}

	if(ibusChannelData[CH6] < CHANNEL_HIGH ){
		vTaskSuspend(task1Handle);
		black_box_reset = TRUE;
	}
	else{
		vTaskResume(task1Handle);
	}

	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	vTaskDelayUntil( &xLastWakeTime, xFrequency );
#ifdef STACK_DEBUG
	stack_task_ahrs = uxTaskGetStackHighWaterMark( NULL );
#endif
    }
  /* USER CODE END ahrs_task */
}

/* USER CODE BEGIN Header_sensor_task */
/**
* @brief Function implementing the task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensor_task */
void sensor_task(void const * argument)
{
  /* USER CODE BEGIN sensor_task */
  /* Infinite loop */
	//vTaskSuspend(NULL);
#ifdef HIL
	vTaskSuspend(NULL);
#else
	int16_t gyso_offset[3] = {0,0,0};
	axis3_t raw;
	uint8_t sample_count = 0;
	int32_t gyro_add[3] = {0,0,0};
	uint8_t first_loop = 1;
	compassInit();
	mpu6050_init(&hi2c2);
	//i2cDectect(&hi2c2);
	HAL_Delay(2000);
	imu_calibrate(&gyso_offset[0],&gyso_offset[1],&gyso_offset[2]);
#endif
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 2;
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{

#ifndef HIL
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
#endif
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
#ifdef STACK_DEBUG
		stack_task_sensor = uxTaskGetStackHighWaterMark( NULL );
#endif
		vTaskDelayUntil( &xLastWakeTime, xFrequency );

	}
  /* USER CODE END sensor_task */
}

/* USER CODE BEGIN Header_osd_task */
/**
* @brief Function implementing the task4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_osd_task */
void osd_task(void const * argument)
{
  /* USER CODE BEGIN osd_task */
  /* Infinite loop */
  for(;;)
  {
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4); // for debug
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	osDelay(1000);
#ifdef STACK_DEBUG
	stack_task_mavOSD = uxTaskGetStackHighWaterMark( NULL );
#endif
    }
  /* USER CODE END osd_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
