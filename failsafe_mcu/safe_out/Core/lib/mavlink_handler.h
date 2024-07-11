#ifndef MAVLINK_HANDLER_H_
#define MAVLINK_HANDLER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void mavlink_init(UART_HandleTypeDef *huart);
uint8_t mavlink_check_fc_healthy();
void mavlink_callback();

#ifdef __cplusplus
}
#endif
#endif
