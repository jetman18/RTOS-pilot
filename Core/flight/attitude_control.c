#include "plane.h"
#include "../Lib/pid.h"
#include "../Lib/imu.h"
#include "../Lib/maths.h"
#include "../Lib/pwm.h"
#include "../Lib/timer.h"
#include "../Driver/ibus.h"
#include "../Lib/gps.h"

#define ROLL_TEST
//#define PITCH_TEST

#define LOOP_TIM 0.01  // Seconds
#define SERVO_MAX_PWM 1900
#define SERVO_MIN_PWM 1100


extern attitude_t AHRS;

float roll_cmd;
float pitch_cmd;

static pid_t roll_rate,pitch_rate;//,yaw_rate;
static pid_t roll_angle,pitch_angle;//,yaw_angle;
static float roll_pid_filted= 0,pitch_pid_filted = 0;

static uint8_t gps_lost;

float const acc_threshold; 
uint16_t servoL,servoR;
static int16_t smooth_ch1=0, smooth_ch2=0;

float roll_pid_gain;
float roll_trim;

float pitch_pid_gain;
float pitch_trim;

void attitude_ctrl_init(){
   gps_lost = 1;

   // init pid 
   pid_init(&roll_angle,6,0,0,100,100,LOOP_TIM);
   pid_init(&roll_rate,3,0,0.2,2,100,LOOP_TIM);

   pid_init(&pitch_angle,4,0,0,100,100,LOOP_TIM);
   pid_init(&pitch_rate,3,0,0,2,100,LOOP_TIM);

}

void attitude_ctrl(){ 

    float roll_r = AHRS.roll_rate;
    float pitch_r = AHRS.pitch_rate;
   // float yaw_r = AHRS.yaw_rate;
    float roll = AHRS.roll;
    float pitch = AHRS.pitch;
    //float yaw = AHRS.yaw;
    roll_cmd = ((int)ibusChannelData[0] - 1500)*0.15f;
	pitch_cmd = ((int)ibusChannelData[1] - 1500)*-0.15f;

	float pid_velo_scale;

    // pid scale with gps velocity 
    if(_gps.fix > 1){
        float vn = (float)_gps.velocity[0]/100;  // m
        float ve = (float)_gps.velocity[1]/100;  // m
        float vd = (float)_gps.velocity[2]/100;  // m
        //float vd = (float)_gps.velocity[3]/100;

        float absolute_velocity = sqrtf(vn*vn + ve*ve + vd*vd);
       // if(gps_lost){
        //    last_ab_velocity = absolute_velocity;
       //     gps_lost = 0;
        //}
        // max speed 120 km/h -> 33 m/s
        absolute_velocity = constrainf(absolute_velocity,0,33); 
        // calculate acceleration 
        //float acc_ = (absolute_velocity - last_ab_velocity);
        // apply threshold
        //if (abs(acc_) > acc_threshold){
        //    absolute_velocity = last_ab_velocity + sign(acc_)*0.3f;
        //}
        // apply filter 
        //absolute_velocity_filter += 0.4f*(absolute_velocity - absolute_velocity_filter);
        //last_ab_velocity = absolute_velocity;
        pid_velo_scale = 1.0f/(1 + absolute_velocity*absolute_velocity*0.0026f);
        
    }
    else{
        pid_velo_scale = 1.0f;
    }

    // stablize mode
    if(ibusChannelData[CH5] > 1600 ){


#ifdef ROLL_TEST
        roll_pid_gain = ((int)ibusChannelData[CH7] - 1000)*0.002f;
        roll_trim = ((int)ibusChannelData[CH8] - 1500)*-0.1f;
#endif

#ifdef PITCH_TEST
        pitch_pid_gain = ((int)ibusChannelData[CH7] - 1000)*0.002f;
        pitch_trim = ((int)ibusChannelData[CH8] - 1500)*-0.1f;
#endif

		//const float pitch_pid_gain = ((int)ibusChannelData[CH8] - 1000)*0.001f;
        
        // roll axis
        float r_angle_pid = pid_cal(&roll_angle,roll,roll_cmd + roll_trim);
        float r_rate_pid  = -pid_cal(&roll_rate,-roll_r,r_angle_pid);
        //pitch axis
        float p_angle_pid = pid_cal(&pitch_angle,pitch,pitch_cmd + pitch_trim);
        float p_rate_pid  = -pid_cal(&pitch_rate,-pitch_r,p_angle_pid);

        pid_velo_scale = constrainf(pid_velo_scale,0.3f,1);
        r_rate_pid = r_rate_pid*pid_velo_scale*roll_pid_gain;
        p_rate_pid = p_rate_pid*pid_velo_scale*pitch_pid_gain;
        
        roll_pid_filted  += 0.4*(r_rate_pid - roll_pid_filted);
        pitch_pid_filted += 0.4*(p_rate_pid - pitch_pid_filted);
		

#ifdef ROLL_TEST
		int s1 = 1500 - ibusChannelData[CH2];

        servoL = 1500 + roll_pid_filted + s1;// - pitch_pid_filted;
        servoR = 1500 - roll_pid_filted + s1;// - pitch_pid_filted;
#endif
#ifdef PITCH_TEST
		int s1 = 1500 - ibusChannelData[CH1];

        servoL = 1500 +  s1 - pitch_pid_filted;
        servoR = 1500 -  s1 - pitch_pid_filted;
#endif
        
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



