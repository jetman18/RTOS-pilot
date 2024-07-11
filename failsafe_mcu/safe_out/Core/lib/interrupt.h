#ifndef _INTERRUPTHANDLER_H_
#define _INTERRUPTHANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "tim.h"
#include "usart.h"
#include "stm32f1xx_hal.h"
#include "ibus.h"
#include "mavlink_handler.h"
#include "timer.h"

// IQR function
//----------------------------------IQR--Handler-----------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
	{
		ibus_calback();
    }
    else if(huart ==&huart2){
    	 mavlink_callback();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == timer_name())
	{
		TIMER_CALLBACK();
	}
}




#ifdef __cplusplus
}
#endif

#endif
