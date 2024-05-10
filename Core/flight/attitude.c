#include "plane.h"
#include "../Lib/pid.h"
#include "../Lib/imu.h"
#include "../Lib/maths.h"
#include "../Lib/pwm.h"
#include "../Lib/timer.h"
#include "../Driver/ibus.h"
#include "../Lib/gps.h"
#include "../Lib/filter.h"


#define LOOP_TIM 0.01f  // Seconds
#define SERVO_MAX_PWM 2000
#define SERVO_MIN_PWM 1000
#define MAX_SPEED 30 //m/s
#define MAX_WAIT_TIME 0.1f
#define F_CUT_OFF  0
const float ff_pitch_gain = 1;
const float ff_roll_gain = 1;


extern attitude_t AHRS;

float roll_desired;
float pitch_desired;

int8_t speed_filter_reset;

static pid_t roll_rate_pid,pitch_rate_pid;
static pid_t roll_angle_pid,pitch_angle_pid;

uint16_t servoL,servoR;
static int16_t smooth_ch1=0, smooth_ch2=0;

float roll_pid_rc_gain;
float roll_trim;

float pitch_pid_rc_gain;
float pitch_trim;

float ab_speed_filted;

void attitude_ctrl_init(){

   speed_filter_reset = TRUE;
   ab_speed_filted = 0.0f;
   // init pid 
   pid_init(&roll_angle_pid,6,0,0,F_CUT_OFF,100);
   pid_init(&roll_rate_pid,5,2.5,0,F_CUT_OFF,300);

   pid_init(&pitch_angle_pid,4,0,0,F_CUT_OFF,100);
   pid_init(&pitch_rate_pid,5,2.5,0,F_CUT_OFF,300);

}

void attitude_ctrl(float dt){ 
    if(dt < 0 || dt > MAX_WAIT_TIME){
        return;
    }

    float roll_rate_measurement = AHRS.roll_rate;
    float pitch_rate_measurement = AHRS.pitch_rate;

    float roll_measurement = AHRS.roll;
    float pitch_measurement = AHRS.pitch;


    roll_desired = ((int)ibusChannelData[0] - 1500)*0.15f;
	pitch_desired = ((int)ibusChannelData[1] - 1500)*-0.15f;

    // pid scale with velocity
	float pid_velo_scale;

    if(_gps.fix > 1){
        float vn = (float)_gps.velocity[0]/100;  // m
        float ve = (float)_gps.velocity[1]/100;  // m
        float vd = (float)_gps.velocity[2]/100;  // m

        float absolute_velocity = sqrtf(sq(vn) + sq(ve) + sq(vd));
        absolute_velocity = constrainf(absolute_velocity,0,MAX_SPEED); 
        if(speed_filter_reset){
            ab_speed_filted = absolute_velocity;
            speed_filter_reset = FALSE;
        }
        ab_speed_filted += pt1FilterGain(10,dt)*(absolute_velocity - ab_speed_filted);
        pid_velo_scale = 1.0/(1 + sq(ab_speed_filted)*0.0035f);
    }
    else{
        speed_filter_reset = TRUE;

    	if(ibusChannelData[CH6] > CHANNEL_HIGH){
    		pid_velo_scale = 1;
    	}else{
    		pid_velo_scale = 0.3f;
    	}
    }

    float pid_roll_vel_scale = constrainf(pid_velo_scale,0.3f,1.0f);
    float pid_pitch_vel_scale = constrainf(pid_velo_scale,0.3f,1.0f);

    // stablize mode
    if(ibusChannelData[CH5] > CHANNEL_HIGH ){

		if(ibusChannelData[CH9] > CHANNEL_HIGH ){
			roll_pid_rc_gain = ((int)ibusChannelData[CH7] - 1000)*0.002f;
			roll_trim = ((int)ibusChannelData[CH8] - 1500)*-0.1f;
		}else{
			pitch_pid_rc_gain = ((int)ibusChannelData[CH7] - 1000)*0.002f;
			pitch_trim = ((int)ibusChannelData[CH8] - 1500)*-0.1f;
		}
        
        // roll axis
        float r_angle_pid =  pid_calculate(&roll_angle_pid,roll_measurement,roll_desired + roll_trim,dt);
        float r_rate_pid  = -pid_calculate(&roll_rate_pid,-roll_rate_measurement,r_angle_pid,dt);
        //float FF_roll = r_angle_pid*ff_roll_gain;
        //r_rate_pid = r_rate_pid - FF_roll;
        //pitch axis
        float p_angle_pid =  pid_calculate(&pitch_angle_pid,pitch_measurement,pitch_desired + pitch_trim,dt);
        float p_rate_pid  = -pid_calculate(&pitch_rate_pid,-pitch_rate_measurement,p_angle_pid,dt);
        //float FF_pitch = p_angle_pid*ff_pitch_gain;
        //p_rate_pid = p_rate_pid - FF_pitch;
        r_rate_pid = r_rate_pid * pid_roll_vel_scale  * roll_pid_rc_gain;
        p_rate_pid = p_rate_pid * pid_pitch_vel_scale * pitch_pid_rc_gain;

		if(ibusChannelData[CH9] > CHANNEL_HIGH ){
				int s1 = 1500 - ibusChannelData[CH2];

				servoL = 1500 + r_rate_pid + s1;// - pitch_pid_filted;
				servoR = 1500 - r_rate_pid + s1;// - pitch_pid_filted;
		}else{
				int s1 = 1500 - ibusChannelData[CH1];

				servoL = 1500 +  s1 - p_rate_pid;
				servoR = 1500 -  s1 - p_rate_pid;
		}

		if(ibusChannelData[CH10] > CHANNEL_HIGH ){
			pitch_rate_pid.i_term = 0;
			roll_rate_pid.i_term = 0;
		}
        
    }
    // manual mode
    else{
        int s1 = 1500 - ibusChannelData[CH1];
        int s2 = 1500 - ibusChannelData[CH2];

        smooth_ch1 += 0.8*(s1 - smooth_ch1);
        smooth_ch2 += 0.8*(s2 - smooth_ch2);
            
        servoL = 1500 + smooth_ch1 + smooth_ch2;
        servoR = 1500 - smooth_ch1 + smooth_ch2;
        
    }

    servoL = constrain(servoL,SERVO_MIN_PWM,SERVO_MAX_PWM);
    servoR = constrain(servoR,SERVO_MIN_PWM,SERVO_MAX_PWM);

    write_pwm_ctrl(ibusChannelData[CH3],servoL,servoR);

}



