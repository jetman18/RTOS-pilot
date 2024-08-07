#include "timer.h"
#include "stm32f1xx_hal.h"


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


void timer_start(TIM_HandleTypeDef *htimz){
	htimmz = htimz;
	HAL_TIM_Base_Start_IT(htimmz);
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

