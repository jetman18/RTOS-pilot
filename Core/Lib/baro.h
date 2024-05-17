#ifndef _BARO_H_
#define _BARO_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"

void baro_init();
void baro_zero_calibrate();
int8_t is_baro_calibration();
int32_t baro_get_altitude();

#ifdef __cplusplus
}
#endif

#endif
