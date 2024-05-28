#ifndef _BARO_H_
#define _BARO_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"

void baro_init();
void baro_zero_calibrate();
int8_t is_baro_calibration();
void baro_calculate(float dt);
int32_t baro_get_climbCm();
int32_t baro_get_altCm();
#ifdef __cplusplus
}
#endif

#endif
