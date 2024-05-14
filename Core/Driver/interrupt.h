#ifndef _INTERRUPTHANDLER_H_
#define _INTERRUPTHANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "tim.h"
#include "usart.h"
#include "stm32f1xx_hal.h"
#include "ibus.h"
#include "../Lib/gps.h"
#include "../flight/plane.h"

// IQR function
//----------------------------------IQR--Handler-----------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == ibus_uart_port())
	{
		ibus_calback();
    }
    else if(huart == gps_uart_port()){
        gps_callback();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
	{
		mavlink_tx_cpl_callback();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_8){
        //hc_sr04_callback();
    }
 /*
   else if(GPIO_Pin == GPIO_PIN_0){
        ppmcallback();
    }
    */
}
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == timer_name())
	{
		TIMER_CALLBACK();
	}
}
*/
//----------------------------------IQR--Handler-----------------------------



#ifdef __cplusplus
}
#endif

#endif
