#include "timer.h"
#include "stm32f4xx_hal.h"
#include "maths.h"

TIM_HandleTypeDef *htimmz;
bootTime_t boottime;
uint32_t _micros;

static uint16_t setoverFlow(int val,int flow_val){
    uint8_t k,l;
    l =flow_val + 1;
    if(l == 0){
      return 0;
    }
    k = val/l;
    k = val - (l*k);
    return k;
}

void timer_calculate_boottime(){
  static uint16_t sec_L  =0;
  sec_L = millis()/1000;
  boottime.sec   = setoverFlow(sec_L,59);
  boottime.min   = setoverFlow((sec_L/60),59);
  boottime.hour  = setoverFlow((sec_L/3600),23);
}

void timer_start(TIM_HandleTypeDef *htimz){
	htimmz = htimz;
	HAL_TIM_Base_Start_IT(htimmz);
}

void delay_us(uint32_t us){
     const uint32_t t_ = micros();
     while ((int)(micros() - t_) < us)
     {
      /* code */
     }
}

TIM_HandleTypeDef *timer_name(){
  return htimmz;
}

void resetCounter(){
   __HAL_TIM_SET_COUNTER(htimmz,0);
}
void timer_callback(){
    _micros += 65536;
}

