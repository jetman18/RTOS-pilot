#ifndef _IMU_H_
#define _IMU_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "axis.h"

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
}quaternion_t;

typedef struct{
    float roll;
	float pitch;
    float yaw;

    float p;
    float q;
    float r;

    float roll_rate;
    float pitch_rate;
    float yaw_rate;
}attitude_t;

typedef struct{
    float roll;
    float pitch;
    float yaw;
}euler_t;

typedef struct{
    int16_t accx;
    int16_t accy;
    int16_t accz;

    int16_t gyrox;
    int16_t gyroy;
    int16_t gyroz;
}IMU_raw_t;

typedef struct{
    float gyro_f_cut;
    float acc_f_cut;
    float gyr_lsb;
}imu_config_t;

extern attitude_t AHRS;
void imu_update_ahrs();
void imu_calibrate(int16_t *offsx,int16_t *offsy,int16_t *offsz);
void update_ahrs(int16_t gx_, int16_t gy_, int16_t gz_, int16_t accx_, int16_t accy_, int16_t accz_,int16_t magx,int16_t magy,int16_t magz,const float dt);
void euler_from_dcm(float *roll, float *pitch, float *yaw);
void get_Acc_Angle(euler_t *m);

#ifdef __cplusplus
}
#endif

#endif
