#ifndef MAINLOOP_H
#define MAINLOOP_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#define FAST_RAM  __attribute__((section("ccmram")))
//#define SIMULATION
// attitude controll


// navigation
//extern uint16_t autopilot_stick;
//extern uint16_t circle_stick;
//extern uint16_t rtHome_stick;
//void Autopilot();
static uint8_t bufee[1000] __attribute__((section("ccmram"))) ;
static void f_f(){
	if(bufee[0]){

	}
}
// mavlink handler
void mavlinkInit(uint8_t syss_id, uint8_t comm_id,UART_HandleTypeDef *uartt,uint32_t baudrate);
void mavlinkCallback();
void mavlink_tx_cpl_callback();
void mavlink_send_heartbeat();
void mavlink_rc_raw(uint16_t thortle,uint16_t servo_L, uint16_t servo_R);
void mavlink_osd();

// attitude contrller
void attitude_ctrl_start(const float dt);
void rate_stabilize(float dt);
void attitude_ctrl_init();
// mainloop
void main_loop();

//estimate 
void estimates_start();
float dynamic_speed_esitmate(float dt);
void position_estimate(float dt);

#ifdef __cplusplus
}
#endif

#endif

