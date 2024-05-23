/*
 * ibus.h
 *
 *  Created on: 10 thg 2, 2023
 *      Author: sudo
 */

#ifndef LIB_IBUS_H_
#define LIB_IBUS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

enum index{
	CH1 = 0,
	CH2,
	CH3,
	CH4,
	CH5,
	CH6,
	CH7,
	CH8,
	CH9,
	CH10,
	CH11, // for RSSI
	CH12,
	CH13
};
#define CHANNEL_HIGH 1700
#define IBUS_MAX_CHANNEL 14
extern uint32_t ibusChannelData[IBUS_MAX_CHANNEL];

void ibus_run();
int ibusFrameComplete();
void ibus_init(UART_HandleTypeDef *uartt);
void ibus_calback();
UART_HandleTypeDef *ibus_uart_port();
#ifdef __cplusplus
}
#endif

#endif /* LIB_IBUS_H_ */
