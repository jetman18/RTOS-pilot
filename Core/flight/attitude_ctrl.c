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
#include "../Lib/baro.h"

#include "../Driver/ibus.h"
#include "string.h"
#define SERVO_MAX_PWM    2000
#define SERVO_MIN_PWM    1000
#define MAX_SPEED        30    //m/s
#define MAX_WAIT_TIME    0.1f
#define MINIMUN_SPEED    12    /* m/s  */
#define MAXIMUN_SPEED    33    /* m/s  */

#define MIN_ROLL_PID_SPEED_SCALE 0.4f
#define MIN_PITCH_PID_SPEED_SCALE 0.3f
#define MAX_PID_SPEED_SCALE 1.0f
#define ERROR_RESET_I_TERM  2.5f

#define SAFETY_ANGLE_ROLL  60
#define SAFETY_ANGLE_PITCH  60


typedef enum{
    ROLL_PITCH_CTRL = 0,
    SPEED_CTRL,
    YAW_CTRL,
    ALT
}attitude_ctr_priority_e;

extern attitude_t AHRS;

float roll_desired;
float pitch_desired;
int servoL,servoR;

int8_t speed_filter_reset;
// angle stabilize
static pid_ roll_rate_pid,pitch_rate_pid;
static pid_ roll_angle_pid,pitch_angle_pid;
// rate stabilize
//static pid_t roll_rate_t,pitch_rate_t;

static int16_t smooth_ch1=0, smooth_ch2=0;

float roll_pid_rc_gain,pitch_pid_rc_gain;

float ab_speed_filted;

float pid_roll_velo_scaler;
float pid_pitch_velo_scaler;

int8_t manual_trim_state = 1;
int16_t manual_trim_roll,manual_trim_pitch;

/*
 *  init pid controller
 */
void attitude_ctrl_init(){
   speed_filter_reset = TRUE;
   ab_speed_filted = 0.0f;
   // init roll pid 
   pid_init(&roll_angle_pid, pid_profile_1.roll_angle_Kp,0,0,10,0,0);
   pid_init(&roll_rate_pid, pid_profile_1.roll_rate_Kp, 
            pid_profile_1.roll_rate_Ki,pid_profile_1.roll_rate_Kd,
            pid_profile_1.roll_fcut_err  , pid_profile_1.roll_f_cut_rate_D,
            pid_profile_1.roll_max_I);
   // init roll pid 
   pid_init(&pitch_angle_pid, pid_profile_1.pitch_angle_Kp,0,0,10,0,0);
   pid_init(&pitch_rate_pid,pid_profile_1.pitch_rate_Kp,
            pid_profile_1.pitch_rate_Ki,pid_profile_1.pitch_rate_Kd,
            pid_profile_1.roll_fcut_err,  pid_profile_1.pitch_f_cut_rate_D,
            pid_profile_1.pitch_max_I);
}


int32_t baro_alt;
int32_t baro_climb;
float Kp_alt = 0.04f;
float Ki_alt = 0.0f;
float i_term_alt = 0.0f;
const float max_i_alt = 10.0f;
/*
 *  Altitude control using PI controller
 *  Return pitch desired
 *  Alt target in cm
 *  dt in sec
 */
float altitude_Pid(int32_t alt_target,float dt){
	if(is_baro_calibration() == FALSE){
		baro_zero_calibrate();
	}else{
		baro_alt = baro_get_altCm();    //cm
		baro_climb = baro_get_climbCm();//cm 
	}
    float out = 0.0f;
    float  error_alt = alt_target/100.0f - baro_alt/100.0f;
    error_alt = constrainf(error_alt,-20,20);
    out += error_alt*Kp_alt;
    // I
    i_term_alt += error_alt*dt*Ki_alt;
    i_term_alt = constrainf(i_term_alt,-max_i_alt,max_i_alt);
    out += i_term_alt;

    return out;
}

void altitude_pid_reset(){
    i_term_alt = 0.0f;
}


/*
 *   Atittude control function
 */
void attitude_ctrl_start(const float dt){
    static float roll_pid_smooth = 0.0f;
    static float pitch_pid_smooth = 0.0f;

    if(dt < 0 || dt > MAX_WAIT_TIME){
        servoL = 1500;
        servoR = 1500;
        //write_pwm_ctrl(1000,servoL,servoR);
        return;
    }

   // calculate trim value
   int16_t roll_trim  = (int)ibusChannelData[CH8] - 1500;
   int16_t pitch_trim = (int)ibusChannelData[CH7] - 1500;

   if(manual_trim_state){
        manual_trim_roll = roll_trim;
        manual_trim_pitch = pitch_trim;
   }
    // stabilize mode
   if(ibusChannelData[CH5] > CHANNEL_HIGH ){
        manual_trim_state = 0;
        /* calculate roll && pitch desired */
        roll_desired  = ((int)ibusChannelData[0] - 1500)* 0.16f  + roll_trim*0.05f;   /*  -60 <-  -> +60  */
        pitch_desired = ((int)ibusChannelData[1] - 1500)*-0.16f  + 10;  /*    -60 <-  -> +60  */
        roll_desired = constrainf(roll_desired,-80,80);
        pitch_desired = constrainf(pitch_desired,-80,80);

        /*---- pid scaler with velocity  -----*/
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
            pid_roll_velo_scaler = (float)MINIMUN_SPEED/((float)MINIMUN_SPEED + sq(speed_temp - MINIMUN_SPEED)*0.09f);
            pid_pitch_velo_scaler = (float)MINIMUN_SPEED/((float)MINIMUN_SPEED + sq(speed_temp - MINIMUN_SPEED)*0.1f);
        }
        else{
            speed_filter_reset = TRUE;
            pid_roll_velo_scaler = 0.5f;
            pid_pitch_velo_scaler = 0.5f;
            //pid_velo_scale = 1.0;
        }


        float roll_measurement = AHRS.roll;
        float pitch_measurement = AHRS.pitch;
        //if(abs(roll_measurement) > SAFETY_ANGLE_ROLL && abs(pitch_measurement) < SAFETY_ANGLE_PITCH){
        //    // priority to roll
        //}
        
        const float pid_roll_vel_scale  = constrainf(pid_roll_velo_scaler,MIN_ROLL_PID_SPEED_SCALE,MAX_PID_SPEED_SCALE);
        const float pid_pitch_vel_scale = constrainf(pid_pitch_velo_scaler,MIN_PITCH_PID_SPEED_SCALE,MAX_PID_SPEED_SCALE);

        /*----- roll axis pid   -----*/
        const float roll_rate_measurement = AHRS.p;
        float roll_rate_desired =  pid_calculate(&roll_angle_pid,roll_measurement,roll_desired,1.0f,dt);
        // limit rate
        roll_rate_desired = constrainf(roll_rate_desired, -pid_profile_1.roll_rate_limit, pid_profile_1.roll_rate_limit);
        float r_rate_pid  =  pid_calculate(&roll_rate_pid, roll_rate_measurement,roll_rate_desired,pid_roll_vel_scale,dt);
        // reset I term
        if(abs(roll_angle_pid.err) < ERROR_RESET_I_TERM){
            roll_rate_pid.i_term = 0.0f;
        }
        // feed forward
        float FF_roll = roll_rate_desired*pid_profile_1.roll_FF_gain;
        r_rate_pid = r_rate_pid + FF_roll;
        r_rate_pid = constrainf(r_rate_pid, -pid_profile_1.roll_max_pid, pid_profile_1.roll_max_pid);
        // filter pid  LPF
        roll_pid_smooth += pt1FilterGain(pid_profile_1.roll_pid_fcut,dt)*(r_rate_pid - roll_pid_smooth);

        /* altitude control*/
        /*
        #define DEFAULT_ALT 300 // 30 mm

        if(ibusChannelData[CH10] > CHANNEL_HIGH && ibusChannelData[CH5] > CHANNEL_HIGH){
            if(abs(roll_measurement) < 40 && abs(pitch_measurement) < 40){
                pitch_measurement  += altitude_Pid(DEFAULT_ALT,0.01);
            }
        }else{
            altitude_pid_reset();
        }
        */
        

        /*-----  pitch axis pid  ---------*/
        const float pitch_rate_measurement = AHRS.q;
        float pitch_rate_desired =  pid_calculate(&pitch_angle_pid,pitch_measurement,pitch_desired,1.0,dt);
        // limit rate
        pitch_rate_desired = constrainf(pitch_rate_desired, -pid_profile_1.pitch_rate_limit, pid_profile_1.pitch_rate_limit);
        float p_rate_pid  =  pid_calculate(&pitch_rate_pid, pitch_rate_measurement,pitch_rate_desired,pid_pitch_vel_scale ,dt);
        // reset I term
        //if(abs(pitch_angle_pid.err) < ERROR_RESET_I_TERM){
        //    pitch_rate_pid.i_term = 0.0f;
        //}
        // feed forward
        float FF_pitch = pitch_rate_desired*pid_profile_1.pitch_FF_gain;;
        p_rate_pid = p_rate_pid + FF_pitch;
        p_rate_pid = constrainf(p_rate_pid,- pid_profile_1.pitch_max_pid, pid_profile_1.pitch_max_pid);
        // filter pid  LPF
        pitch_pid_smooth += pt1FilterGain(pid_profile_1.pitch_pid_fcut,dt)*(p_rate_pid - pitch_pid_smooth);
        
        // enable && disable I term, for test only
		if(ibusChannelData[CH10] > CHANNEL_HIGH ){
			pitch_rate_pid.i_term = 0;
			roll_rate_pid.i_term = 0;
		}

        /*-------------- mix channel --------------------------*/
		if(ibusChannelData[CH9] > CHANNEL_HIGH ){
                // roll stabilize
				int pitch_rc = 1500 - ibusChannelData[CH2] + pitch_trim;
				servoL = 1500 + roll_pid_smooth - pitch_rc;// + roll_trim  + pitch_trim;
				servoR = 1500 - roll_pid_smooth - pitch_rc;// - roll_trim  + pitch_trim;
                //servoL = 1500 + roll_pid_smooth;// + roll_trim  + pitch_trim;
				//servoR = 1500 - pitch_rc - manual_trim_roll;;// - roll_trim  + pitch_trim;
		}else{
               // pitch stabilize
				int roll_rc = 1500 - ibusChannelData[CH1] + roll_trim ;
				servoL = 1500 -  roll_rc*0.5 - pitch_pid_smooth;//    + roll_trim  + pitch_trim;
				servoR = 1500 +  roll_rc*0.5 - pitch_pid_smooth;//    - roll_trim  + pitch_trim;
                //servoL = 1500 -  roll_rc + manual_trim_pitch;
				//servoR = 1500 +  pitch_pid_smooth;
		}

        //servoL = 1500 - roll_pid_smooth - pitch_pid_smooth;
		//servoR = 1500 + roll_pid_smooth - pitch_pid_smooth;
        
    }
    // manual mode
    else{
        // reset pid
        pid_reset(&pitch_rate_pid);
        pid_reset(&roll_rate_pid);

        int s1 = 1500 - ibusChannelData[CH1];
        int s2 = 1500 - ibusChannelData[CH2];

        smooth_ch1 += 0.7*(s1 - smooth_ch1);
        smooth_ch2 += 0.7*(s2 - smooth_ch2);

        servoL = 1500 - smooth_ch1 - smooth_ch2  - manual_trim_roll  + manual_trim_pitch;
        servoR = 1500 + smooth_ch1 - smooth_ch2  + manual_trim_roll  + manual_trim_pitch;
        
        //servoL = 1500 - smooth_ch1  + manual_trim_pitch;
        //servoR = 1500 + smooth_ch2  - manual_trim_roll;
    }
    servoL = constrain(servoL,SERVO_MIN_PWM,SERVO_MAX_PWM);
    servoR = constrain(servoR,SERVO_MIN_PWM,SERVO_MAX_PWM);
   //write_pwm_ctrl(1000,servoL,servoR);
    mavlink_rc_raw(1000,servoL,servoR);
}


/*
* this function check plane is flying
*/
bool isFlying(){
    static uint32_t timer = 0;
    static uint8_t fly_cond_cnt = 0;
    static uint8_t fly_state = 0;
    uint16_t throtle = ibusChannelData[CH3];
    if(throtle > 1800 && fly_state == 0){
        if((millis() - timer) > 1500){  // 1.5 s 
              fly_cond_cnt ++;
        }
    }else{
        timer = millis();
        return 0;
    }
    int32_t vn = _gps.velocity[0]; 
    int32_t ve = _gps.velocity[1];
    int32_t absolute_velocity = sqrt(sq(vn) + sq(ve));
    if(absolute_velocity > 1000 && fly_cond_cnt > 0 ){
        if(fly_state == 0){
            fly_state = 1;
            return 1;
        }
    }
    return 0;
}


/*
 *  Heading control function
*/
#define MAX_PITCH_    30
#define MAX_YAW_RATE  45  // 45 deg/sec
#define HD_CTR_LOOP   0.01f
#define MIN_ROLL_FOR_PITCH 5
#define YAW_ANGLE_Kp  1.0f
#define YAW_RATE_Kp   0.8f;
#define YAW_RATE_ERR_TO_ROLL 0.5f
void heading_control(const float hd_desired, float *roll, float *pitch){
    const float yaw_rate_measurement = AHRS.yaw_rate;
    const float hd_measurement = AHRS.yaw;   // 0 -> 359
    const float yaw_rate_err_filter;
    static float err = 0;
    // state variable
    static uint8_t err_roll_reset_state = 1;
    static uint8_t cmd_roll_state = 1;
    static uint8_t yaw_rate_err_reset = 1;

    // yaw rate desired
    float yaw_rate_d = hd_desired - hd_measurement;
    if(yaw_rate_d > 180.0f){
       yaw_rate_d = yaw_rate_d - 360.0f;
    }
    // angle P term
    yaw_rate_d *= YAW_ANGLE_Kp;
    // limited yaw rate
    yaw_rate_d = constrainf(yaw_rate_d,-MAX_YAW_RATE,MAX_YAW_RATE);
    //rate P term
    float yaw_rate_error = (yaw_rate_d - yaw_rate_measurement)*YAW_RATE_Kp;
    ------
    if(fabs(yaw_rate_error) > 10 && yaw_rate_err_reset){
        yaw_rate_err_filter = yaw_rate_error;
        yaw_rate_err_reset = 0;
    }else if(fabs(yaw_rate_error) < 10){
        yaw_rate_err_reset = 1;
    }
    yaw_rate_err_filter += pt1FilterGain(1,HD_CTR_LOOP)*(yaw_rate_error - yaw_rate_err_filter);
    ---------

    // cvt yaw rate err to roll angle
    float roll_desired = yaw_rate_error*YAW_RATE_ERR_TO_ROLL;
    roll_desired = constrainf(roll_desired,-50,50);
    *roll = roll_desired;

    float error = roll_desired -  AHRS.roll;

    if(fabs(error) > 5.0f && err_roll_reset_state){
        err = error;
        err_roll_reset_state = 0;
    }else if(fabs(error) < 5.0f){
       err_roll_reset_state = 1;   
    }
    // low pass filter err
    err += pt1FilterGain(1,HD_CTR_LOOP)*(error - err);

    if(fabs(err) < 3.0f && cmd_roll_state){
        float pitch_ =  yaw_rate_error*YAW_RATE_ERR_TO_ROLL;
        *pitch = constrainf(pitch_,0,30);
        cmd_roll_state = 0;
    }
}

/*
 * Rate stabilize
 */
void rate_stabilize(float dt){
    uint16_t servoL;
    uint16_t servoR;

    //v_estimate = dynamic_speed_esitmate(dt);

    if(ibusChannelData[CH5] > CHANNEL_HIGH ){
        float roll_rate_measurement = AHRS.roll_rate;
        float pitch_rate_measurement = AHRS.pitch_rate;

        float roll_rate_desired = ((int)ibusChannelData[0] - 1500)*0.5f;
        float pitch_rate_desired = ((int)ibusChannelData[1] - 1500)*-0.5f;

        // pid scale with velocity
        //float pid_velo_scale = 1.0/(1 + sq(v_estimate)*0.0035f);

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

