#ifndef PID_PROFILE_H
#define PID_PROFILE_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"

typedef struct
{
    // genneral
    uint16_t max_servo;
    uint16_t min_servo;
    
    // roll axis
    int16_t roll_rate_limit;
    float roll_FF_gain;
    
    uint8_t roll_fcut_err;
    float roll_angle_Kp;
    float roll_rate_Kp;
    float roll_rate_Ki;
    float roll_rate_Kd;
    int8_t roll_f_cut_rate_D;
    int16_t roll_max_I;
    int8_t  roll_pid_fcut;
    int16_t roll_max_pid;

    // pitch axis
    int16_t pitch_rate_limit;
    float pitch_FF_gain;

    uint8_t pitch_fcut_err;
    float pitch_angle_Kp;
    float pitch_rate_Kp;
    float pitch_rate_Ki;
    float pitch_rate_Kd;
    int8_t pitch_f_cut_rate_D;
    int16_t pitch_max_I;
    int8_t  pitch_pid_fcut;
    int16_t pitch_max_pid;

}pid_profile_t;

static pid_profile_t pid_profile_1={
    .max_servo = 2000,
    .min_servo = 1000,

    .roll_rate_limit = 180,
    .roll_FF_gain = 1.2f,

    .roll_fcut_err = 10,
    .roll_angle_Kp = 2,
    .roll_rate_Kp  = 1.1,
    .roll_rate_Ki  = 1.5,
    .roll_rate_Kd  = 0,
    .roll_f_cut_rate_D = 0,
    .roll_max_I = 150,
    .roll_pid_fcut = 40,
    .roll_max_pid  = 400,

    // pitch axis
    .pitch_rate_limit = 180,
    .pitch_FF_gain = 1.0f,

    .pitch_fcut_err = 10,
    .pitch_angle_Kp = 2.1,
    .pitch_rate_Kp  = 1.2,
    .pitch_rate_Ki  = 1.5,
    .pitch_rate_Kd  = 0,
    .pitch_f_cut_rate_D = 0,
    .pitch_max_I = 150,
    .pitch_pid_fcut = 40,
    .pitch_max_pid  = 500
};

/*
static pid_profile_t pid_file_2={
     .max_servo = 2000,
    .min_servo = 1000,

    .roll_rate_limit = 50,
    .roll_FF_gain = 1.5f,

    .roll_angle_Kp = 3,
    .roll_rate_Kp  = 1.5, 
    .roll_rate_Ki  = 2.5,
    .roll_rate_Kd  = 0,
    .roll_f_cut_rate_D = 0,
    .roll_max_I = 150,
    .roll_pid_fcut = 5,
    .roll_max_pid  = 400,

    // pitch axis
    .pitch_rate_limit = 50,
    .pitch_FF_gain = 0,

    .pitch_angle_Kp = 3,
    .pitch_rate_Kp  = 1.5,
    .pitch_rate_Ki  = 2.5,
    .pitch_rate_Kd  = 0,
    .pitch_f_cut_rate_D = 0,
    .pitch_max_I = 150,
    .pitch_pid_fcut = 5,
    .pitch_max_pid  = 400
};
*/



#ifdef __cplusplus
}
#endif

#endif
