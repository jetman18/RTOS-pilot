#include "plane.h"
#include "pid_profile.h"

#include "../Lib/pid.h"
#include "../Lib/imu.h"
#include "../Lib/maths.h"
#include "../Lib/pwm.h"
#include "../Lib/timer.h"
#include "../Lib/gps.h"
#include "../Lib/filter.h"
#include "../Lib/utils.h"

#include "../Driver/ibus.h"

#define SERVO_MAX_PWM    2000
#define SERVO_MIN_PWM    1000
#define MAX_SPEED        30    //m/s
#define MAX_WAIT_TIME    0.1f
#define MINIMUN_SPEED    12    /* m/s  */
#define MAXIMUN_SPEED    33    /* m/s  */

#define MAX_ROLL_DESIRED 70
#define MAX_PITCH_DESIRED 70

#define MIN_PID_SPEED_SCALE 0.4f
#define MAX_PID_SPEED_SCALE 1.0f
#define ERROR_RESET_I_TERM  2.5f


typedef enum{
    ROLL_PITCH_CTRL = 0,
    SPEED_CTRL,
    YAW_CTRL,
    ALT
}attitude_ctr_priority_e;

extern attitude_t AHRS;

float roll_desired;
float pitch_desired;
uint16_t servoL,servoR;

int8_t speed_filter_reset;
// angle stabilize
static pid_t roll_rate_pid,pitch_rate_pid;
static pid_t roll_angle_pid,pitch_angle_pid;
// rate stabilize
static pid_t roll_rate_t,pitch_rate_t;

static int16_t smooth_ch1=0, smooth_ch2=0;

float roll_pid_rc_gain,pitch_pid_rc_gain;
float roll_trim,pitch_trim;

float ab_speed_filted;

float pid_velo_scale;

/*
 *  init pid controller
 **/
void attitude_ctrl_init(){

   speed_filter_reset = TRUE;
   ab_speed_filted = 0.0f;
   // init pid 
   pid_init(&roll_angle_pid, pid_file_1.roll_angle_Kp,0,0,5,0,0);
   pid_init(&roll_rate_pid, pid_file_1.roll_rate_Kp, pid_file_1.roll_rate_Ki, pid_file_1.roll_rate_Kd,
            pid_file_1.roll_fcut_err  , pid_file_1.roll_f_cut_rate_D, pid_file_1.roll_max_I);

   pid_init(&pitch_angle_pid, pid_file_1.pitch_angle_Kp,0,0,5,0,0);
   pid_init(&pitch_rate_pid,pid_file_1.pitch_rate_Kp,pid_file_1.pitch_rate_Ki,pid_file_1.pitch_rate_Kd,
            pid_file_1.roll_fcut_err,  pid_file_1.pitch_f_cut_rate_D,pid_file_1.pitch_max_I);
}

void attitude_ctrl(const uint32_t micros){
    static uint32_t last_time_us; 
    float dt = (micros - last_time_us)*(1e-6f);
    last_time_us = micros;
    if(dt < 0 || dt > MAX_WAIT_TIME){
        return;
    }

    const float roll_rate_measurement = AHRS.roll_rate;
    const float pitch_rate_measurement = AHRS.pitch_rate;

    const float roll_measurement = AHRS.roll;
    const float pitch_measurement = AHRS.pitch;

    static float roll_pid_smooth = 0.0f;
    static float pitch_pid_smooth = 0.0f;
    
    /* calculate roll && pitch desired
    */

	if(ibusChannelData[CH9] > CHANNEL_HIGH ){
		roll_pid_rc_gain = ((int)ibusChannelData[CH7] - 1000)*0.002f;
		roll_trim = ((int)ibusChannelData[CH8] - 1500)*-0.1f;
	}else{
		pitch_pid_rc_gain = ((int)ibusChannelData[CH7] - 1000)*0.002f;
		pitch_trim = ((int)ibusChannelData[CH8] - 1500)*-0.1f;
	}

    roll_desired = ((int)ibusChannelData[0] - 1500)*0.15f    + roll_trim;   /*  -50 <-  -> +50  */
	pitch_desired = ((int)ibusChannelData[1] - 1500)*-0.15f + pitch_trim ;/*  -75 <-  -> +75  */

    /*---- pid scale with velocity  -----*/
    if(_gps.fix > 1){
        float vn = (float)_gps.velocity[0]/100;  // m
        float ve = (float)_gps.velocity[1]/100;  // m
        //float vd = (float)_gps.velocity[2]/100;  // m

        float absolute_velocity = sqrtf(sq(vn) + sq(ve));// + sq(vd));
        absolute_velocity = constrainf(absolute_velocity,0,MAX_SPEED); 
        if(speed_filter_reset){
            ab_speed_filted = absolute_velocity;
            speed_filter_reset = FALSE;
        }
        ab_speed_filted += pt1FilterGain(10,dt)*(absolute_velocity - ab_speed_filted);
        float speed_temp = constrainf(ab_speed_filted,MINIMUN_SPEED,MAXIMUN_SPEED);
        pid_velo_scale = (float)MINIMUN_SPEED/((float)MINIMUN_SPEED + sq(speed_temp - MINIMUN_SPEED)*0.04f);
    }
    else{
        speed_filter_reset = TRUE;
    	if(ibusChannelData[CH6] > CHANNEL_HIGH){
    		pid_velo_scale = 1;
    	}else{
    		pid_velo_scale = 0.4f;
    	}
    }

    const float pid_roll_vel_scale  = constrainf(pid_velo_scale,MIN_PID_SPEED_SCALE,MAX_PID_SPEED_SCALE);
    const float pid_pitch_vel_scale = constrainf(pid_velo_scale,MIN_PID_SPEED_SCALE + 0.2,MAX_PID_SPEED_SCALE);

    // stabilize mode
    if(ibusChannelData[CH5] > CHANNEL_HIGH ){
        /*----- roll axis pid   -----*/
        float roll_rate_desired =  pid_calculate(&roll_angle_pid,roll_measurement,roll_desired,1.0f,dt);
        // limit rate
        roll_rate_desired = constrainf(roll_rate_desired, -pid_file_1.roll_rate_limit, pid_file_1.roll_rate_limit);
        float r_rate_pid  =  pid_calculate(&roll_rate_pid, roll_rate_measurement,roll_rate_desired,pid_roll_vel_scale,dt);
        // reset I term
        if(abs(roll_angle_pid.err) < ERROR_RESET_I_TERM){
            roll_rate_pid.i_term = 0.0f;
        }
        // feed forward
        float FF_roll = roll_rate_desired*pid_file_1.roll_FF_gain;
        r_rate_pid = r_rate_pid + FF_roll;
        r_rate_pid = constrainf(r_rate_pid, -pid_file_1.roll_max_pid, pid_file_1.roll_max_pid);
        // filter pid  LPF
        roll_pid_smooth += pt1FilterGain(pid_file_1.roll_pid_fcut,dt)*(r_rate_pid - roll_pid_smooth);


        /*-----  pitch axis pid  ---------*/
        float pitch_rate_desired =  pid_calculate(&pitch_angle_pid,pitch_measurement,pitch_desired,1.0f,dt);
        // limit rate
        pitch_rate_desired = constrainf(pitch_rate_desired, -pid_file_1.pitch_rate_limit, pid_file_1.pitch_rate_limit);
        float p_rate_pid  =  pid_calculate(&pitch_rate_pid, pitch_rate_measurement,pitch_rate_desired,pid_pitch_vel_scale ,dt);
        // reset I term
        if(abs(pitch_angle_pid.err) < ERROR_RESET_I_TERM){
            pitch_rate_pid.i_term = 0.0f;
        }
        // feed forward
        float FF_pitch = pitch_rate_desired*pid_file_1.pitch_FF_gain;;
        p_rate_pid = p_rate_pid + FF_pitch;
        p_rate_pid = constrainf(p_rate_pid,- pid_file_1.pitch_max_pid, pid_file_1.pitch_max_pid);
        // filter pid  LPF
        pitch_pid_smooth += pt1FilterGain(pid_file_1.pitch_pid_fcut,dt)*(p_rate_pid - pitch_pid_smooth);
        

        /*-------------- mix channel --------------------------*/
		if(ibusChannelData[CH9] > CHANNEL_HIGH ){
                // roll stabilize
				int pitch_rc = 1500 - ibusChannelData[CH2];

				servoL = 1500 - roll_pid_smooth + pitch_rc;
				servoR = 1500 + roll_pid_smooth + pitch_rc;
		}else{
               // pitch stabilize
				int roll_rc = 1500 - ibusChannelData[CH1];

				servoL = 1500 +  roll_rc*0.5 + pitch_pid_smooth;
				servoR = 1500 -  roll_rc*0.5 + pitch_pid_smooth;
		}

        // enable or disable I term, for test only
		if(ibusChannelData[CH10] > CHANNEL_HIGH ){
			pitch_rate_pid.i_term = 0;
			roll_rate_pid.i_term = 0;
		}
        
    }
    // manual mode
    else{
        int s1 = 1500 - ibusChannelData[CH1];
        int s2 = 1500 - ibusChannelData[CH2];

        smooth_ch1 += 0.8*(s1*0.5 - smooth_ch1);
        smooth_ch2 += 0.8*(s2 - smooth_ch2);
            
        servoL = 1500 + smooth_ch1 + smooth_ch2;
        servoR = 1500 - smooth_ch1 + smooth_ch2;
        
    }

    servoL = constrain(servoL,SERVO_MIN_PWM,SERVO_MAX_PWM);
    servoR = constrain(servoR,SERVO_MIN_PWM,SERVO_MAX_PWM);
    write_pwm_ctrl(ibusChannelData[CH3],servoL,servoR);

}


static void Altitude_control(){

}

/*
        // roll axis
        float roll_rate_desired =  pid_calculate(&roll_angle_pid,roll_measurement,roll_desired + roll_trim,dt);
        float r_rate_pid  = -pid_calculate(&roll_rate_pid,-roll_rate_measurement,roll_rate_desired,dt);
        float FF_roll = roll_rate_desired*ff_roll_gain;
        r_rate_pid = r_rate_pid + FF_roll;
        //pitch axis
        float pitch_rate_desired =  pid_calculate(&pitch_angle_pid,pitch_measurement,pitch_desired + pitch_trim,dt);
        float p_rate_pid  = -pid_calculate(&pitch_rate_pid,-pitch_rate_measurement,pitch_rate_desired,dt);


void rate_stabilize(float dt){
    uint16_t servoL;
    uint16_t servoR;

    v_estimate = dynamic_speed_esitmate(dt);

    if(ibusChannelData[CH5] > CHANNEL_HIGH ){
        float roll_rate_measurement = AHRS.roll_rate;
        float pitch_rate_measurement = AHRS.pitch_rate;

        float roll_rate_desired = ((int)ibusChannelData[0] - 1500)*0.5f;
        float pitch_rate_desired = ((int)ibusChannelData[1] - 1500)*-0.5f;

        // pid scale with velocity
        float pid_velo_scale = 1.0/(1 + sq(v_estimate)*0.0035f);

        float r_rate  =  pid_calculate(&roll_rate_t, roll_rate_measurement,roll_rate_desired,1.0,dt);
        float p_rate  =  pid_calculate(&pitch_rate_t, pitch_rate_measurement,pitch_rate_desired,1.0,dt);
        
        r_rate *= pid_velo_scale;
        p_rate *= pid_velo_scale;

        //roll_pid_smooth += 0.4*(r_rate - roll_pid_smooth);
        //pitch_pid_smooth += 0.4*(p_rate - pitch_pid_smooth);

        int s1 = 1500 - ibusChannelData[CH2];

        servoL = 1500 - r_rate + s1;// - pitch_pid_filted;
        servoR = 1500 + r_rate + s1;// - pitch_pid_filted;
        
     }else{
        int s1 = 1500 - ibusChannelData[CH1];
        int s2 = 1500 - ibusChannelData[CH2];
            
        servoL = 1500 + s1 + s2;
        servoR = 1500 - s1 + s2;
    }

    servoL = constrain(servoL,SERVO_MIN_PWM,SERVO_MAX_PWM);
    servoR = constrain(servoR,SERVO_MIN_PWM,SERVO_MAX_PWM);

    write_pwm_ctrl(ibusChannelData[CH3],servoL,servoR);

}
*/
