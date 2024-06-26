#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "stdio.h"

typedef struct {
	float err;
	float err_fcut;
	float kp;
	float ki;
	float kd;
	float i_term;
	float last_input;
	float D_filted;
	float I_range;
	float f_cut_D;
	uint8_t init;
}pid_;
void pid_init(pid_  *pid_in,float kp, float ki, float kd,float f_cut_err, float f_cut_D, float maxI);
float pid_calculate(pid_ *pid_in,float measurement, float setpoint,float scaler,float dt);
void  pid_reset(pid_ *t);
#ifdef __cplusplus
}
#endif

#endif
