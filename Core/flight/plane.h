#ifndef MAINLOOP_H
#define MAINLOOP_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
//#define SIMULATION
// attitude controll


// navigation
//extern uint16_t autopilot_stick;
//extern uint16_t circle_stick;
//extern uint16_t rtHome_stick;
//void Autopilot();

// mavlink handler
void mavlinkInit(uint8_t syss_id, uint8_t comm_id,UART_HandleTypeDef *uartt,uint32_t baudrate);
void mavlinkCallback();
void mavlink_tx_cpl_callback();
void mavlink_send_heartbeat();
void mavlink_send_attitude(float roll,float pitch, float yaw);
void mavlink_osd();

// attitude contrller
void attitude_ctrl(const uint32_t micros);
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

