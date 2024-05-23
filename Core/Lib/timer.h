
#ifndef LIB_TIMER_H_
#define LIB_TIMER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stdio.h"
typedef struct{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
}bootTime_t;

extern TIM_HandleTypeDef *htimmz;
extern uint32_t _micros;
extern bootTime_t boottime;

void timer_callback();
void timer_calculate_boottime();
void timer_start(TIM_HandleTypeDef *htimz);
TIM_HandleTypeDef *timer_name();
void delay_us(uint32_t us);
#define micros() (_micros + (__HAL_TIM_GET_COUNTER(htimmz)))
#define millis() (micros() / 1000)
#define seconds() (micros()/1000000)
#define TIMER_CALLBACK()  (_micros += 65535)
#ifdef __cplusplus
}
#endif
#endif
