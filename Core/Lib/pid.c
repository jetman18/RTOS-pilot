#include "pid.h"
#include "filter.h"
#include "maths.h"
#include "timer.h"
#include "string.h"

void pid_init(pid_  *pid_in,float kp, float ki, float kd,float f_cut_err, float f_cut_D, float maxI){
  memset(pid_in,0,sizeof(pid_));
  pid_in->err = 0.0f;
  pid_in->err_fcut = f_cut_err;
  pid_in->kp = kp;
  pid_in->ki = ki;
  pid_in->kd = kd;
  pid_in->f_cut_D = f_cut_D;
  pid_in->I_range = maxI;
  pid_in->last_input = 0;
  pid_in->D_filted = 0;
  pid_in->init = 1;
}

float pid_calculate(pid_ *pid_in,float measurement, float setpoint,float scaler,float dt){
   if(pid_in->init){
       pid_in->last_input = measurement;
       pid_in->init = 0;
       return 0.0f;
   }
   // Caculate P term
   float error = setpoint - measurement;
   pid_in->err += pt1FilterGain(pid_in->err_fcut,dt)*(error - pid_in->err);
   float output = pid_in->err*pid_in->kp*scaler;

   // Caculate I term
   if(pid_in->ki > 0){
      pid_in->i_term += pid_in->err *pid_in->ki *dt;
      pid_in->i_term = constrainf(pid_in->i_term,-pid_in->I_range,pid_in->I_range);
      output += pid_in->i_term;
   }
   // Caculate D term
   if(pid_in->kd > 0){
        float delta =  (measurement - pid_in->last_input)*pid_in->kd/dt;
        pid_in->last_input = measurement;
        // low pass filter
        pid_in->D_filted += pt1FilterGain(pid_in->f_cut_D,dt)*(delta - pid_in->D_filted);
        output -= pid_in->D_filted*scaler;
   }
   return output;
}

void pid_reset(pid_ *pid_in)
{
	pid_in->i_term = 0.0f;
	pid_in->last_input = 0.0f;
   pid_in->D_filted = 0.0f;
   pid_in->err = 0;
}


/*

float pid_calculate(pid_ *pid_in,float measurement, float setpoint,float dt){
   if(pid_in->init){
       pid_in->last_input = measurement;
       pid_in->init = 0;
       return 0.0f;
   }

   float error = measurement - setpoint;
   float output = error*pid_in->kp;

   if(pid_in->ki > 0){
      pid_in->i_term += error *pid_in->ki *dt;
      pid_in->i_term = constrainf(pid_in->i_term,-pid_in->I_range,pid_in->I_range);
      output += pid_in->i_term;
   }
   if(pid_in->kd > 0){
        // low pass filter
        float RC = 1.0f / (2 *M_PIf *pid_in->f_cut_D);
        float gain_lpf = dt/(RC + dt);
        float delta =  (measurement - pid_in->last_input)*pid_in->kd;
        pid_in->last_input = measurement;
        delta /= dt;
        pid_in->D_filted += gain_lpf*(delta - pid_in->D_filted);
        output += pid_in->D_filted;
   }
   return output;
}
*/
