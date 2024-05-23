#include "imu.h"
#include "stm32f4xx_hal.h"
#include "maths.h"
#include "math.h"
#include "filter.h"
#include "timer.h"
#include "axis.h"
#include "compass.h"
#include "i2c.h"
#include "utils.h"
#include "../Driver/ibus.h"
#include "../Driver/mpu6050.h"
#include "../Driver/hmc5883.h"

#define OFFSET_CYCLE  1000
#define USE_MAG 1

float velocity_test = 0;
float position_test = 0;

attitude_t AHRS;
float integralFBx;
float integralFBy;
float integralFBz;
float acc_Eframe[3];
int16_t acc_Bframe[3];

const float Ki_imu = 0;
float Kp_imu = 0.2;
const float Kp_mag = 5;

float q0=1,q1=0,q2=0,q3=0;
static float dcm[3][3];
float cosx,cosy,cosz,sinx,siny, sinz,tany;

uint8_t isGyrocalibrated = FALSE;
uint32_t init_us;

float pitch_trim_imu = 3.14;
float roll_trim_imu = 0;

//IMU configuration parameters
imu_config_t config ={
  .gyro_f_cut =100,
  .acc_f_cut = 100,
  .gyr_lsb = 32.8f
};


static int32_t store_gyro[3];
void imu_calibrate(int16_t *offsx,int16_t *offsy,int16_t *offsz){
	axis3_t gyro_;
	integralFBx = 0;
	integralFBy = 0;
	integralFBz = 0;
	for(int i = 0;i < OFFSET_CYCLE; i++){
		mpu6050_gyro_get_raw(&gyro_);
		store_gyro[X] += gyro_.x;
    	store_gyro[Y] += gyro_.y;
    	store_gyro[Z] += gyro_.z;
		HAL_Delay(1); // delay 1 ms
	}
	*offsx = store_gyro[X] / OFFSET_CYCLE;
	*offsy = store_gyro[Y] / OFFSET_CYCLE;
	*offsz = store_gyro[Z] / OFFSET_CYCLE;
	init_us = millis();
}

/* Calculate euler angles from acceleration
 */
void get_Acc_Angle(euler_t *m)
{
	axis3_t  acce;
	faxis3_t acc;
	uint32_t sum;
	float length;
    mpu6050_acc_get_raw(&acce);
	sum = acce.x*acce.x + acce.y*acce.y + acce.z*acce.z;
	if(sum == 0){
		return;
	}
	length = invSqrt_((float)sum);
    acc.x = acce.x*length;
    acc.y = acce.y*length;
    acc.z = acce.z*length;
	m->pitch  = atan2_approx(acc.y,acc.z)*180/M_PIf;
	m->roll   = atan2_approx(-acc.x, (1/invSqrt_(acc.y * acc.y + acc.z * acc.z)))*180/M_PIf;
}

static int8_t reset_state = 1;
// mahony filter
void update_ahrs(int16_t gx_, int16_t gy_, int16_t gz_, int16_t accx_, int16_t accy_, int16_t accz_,int16_t magx,int16_t magy,int16_t magz,const float dt){
	float norm;
	float ex, ey, ez;
    float gx,gy,gz;
    float acc_x,acc_y,acc_z;
    float vx, vy, vz;
    float emz,wx,wy;
    float mx,my,mz,hx,hy,bx,bz;


	gx = (gx_/config.gyr_lsb) * RAD;
	gy = (gy_/config.gyr_lsb) * RAD;
	gz = (gz_/config.gyr_lsb) * RAD;

	if(!((accx_ == 0) && (accy_ == 0) && ( accz_ == 0))) {
		norm = invSqrt_(accx_ * accx_ + accy_ * accy_ + accz_ * accz_);
		acc_x = (float)accx_ * norm;
		acc_y = (float)accy_ * norm;
		acc_z = (float)accz_ * norm;

		if(reset_state){
			dcm[0][2] = acc_x;
			dcm[1][2] = acc_y;
			dcm[2][2] = acc_z;
			reset_state = 0;
		}

        if(USE_MAG){
			norm = invSqrt_(magx * magx + magy * magy + magz * magz);
			mx = magx * norm;
			my = magy * norm;
			mz = magz * norm;

			hx = mx * dcm[0][0] + my * dcm[1][0] + mz * dcm[2][0];
			hy = mx * dcm[0][1] + my * dcm[1][1] + mz * dcm[2][1];
			bz = mx * dcm[0][2] + my * dcm[1][2] + mz * dcm[2][2];

			bx = sqrtf(hx * hx + hy * hy);

			wx = bx * dcm[0][0] + bz * dcm[0][2];
			wy = bx * dcm[1][0] + bz * dcm[1][2];
			emz = mx * wy - my * wx;
		}
		else{
			emz = 0.0f;
		}

		vx = dcm[0][2];
		vy = dcm[1][2];
		vz = dcm[2][2];

		ex = acc_y * vz - acc_z * vy;
		ey = acc_z * vx - acc_x * vz;
		ez = acc_x * vy - acc_y * vx;

		if( Ki_imu  > 0.0f) {
			integralFBx +=  Ki_imu  * ex * dt;
			integralFBy +=  Ki_imu  * ey * dt;
			integralFBz +=  Ki_imu  * ez * dt;
			gx += integralFBx;
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		gx += Kp_imu * ex;
		gy += Kp_imu * ey;
		gz += Kp_imu * ez + emz * Kp_mag;
	}

	gx *= (0.5f * dt);
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);

	q0 += (-q1 * gx - q2 * gy - q3 * gz);
	q1 += ( q0 * gx + q2 * gz - q3 * gy);
	q2 += ( q0 * gy - q1 * gz + q3 * gx);
	q3 += ( q0 * gz + q1 * gy - q2 * gx);

	norm = invSqrt_(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= norm;
	q1 *= norm;
	q2 *= norm;
	q3 *= norm;
	
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;

	// Quaternion to Rotation matrix
	dcm[0][0] = 2.0f*(0.5f - q2q2  - q3q3);
	dcm[1][0] = 2.0f*(q1q2 - q0q3);
	dcm[2][0] = 2.0f*(q1q3 + q0q2);
	dcm[0][1] = 2.0f*(q1q2 + q0q3);
	dcm[1][1] = 2.0f*(0.5f - q1q1 - q3q3);
	dcm[2][1] = 2.0f*(q2q3 - q0q1);
	dcm[0][2] = 2.0f*(q1q3 - q0q2);
	dcm[1][2] = 2.0f*(q2q3 + q0q1);
	dcm[2][2] = 2.0f*(0.5f - q1q1 - q2q2);
	
    // Rotate acceleration from Body frame to earth frame
	int16_t acc_Eframex = dcm[0][0]*accx_ + dcm[1][0]*accy_ + dcm[2][0]*accz_;
	int16_t acc_Eframey = dcm[0][1]*accx_ + dcm[1][1]*accy_ + dcm[2][1]*accz_;
	int16_t acc_Eframez = dcm[0][2]*accx_ + dcm[1][2]*accy_ + dcm[2][2]*accz_;
	acc_Eframez -= 2000;


	const float accTrueScale = 9.81f/2000.0f; // 2048
	acc_Eframe[X] = acc_Eframex*accTrueScale;
	acc_Eframe[Y] = acc_Eframey*accTrueScale;
	acc_Eframe[Z] = acc_Eframez*accTrueScale;

	acc_Eframe[X] = fapplyDeadband(acc_Eframe[X],0.02);
	acc_Eframe[Y] = fapplyDeadband(acc_Eframe[Y],0.02);
	acc_Eframe[Z] = fapplyDeadband(acc_Eframe[Z],0.02);

    if(millis() - init_us < 5000){
    	acc_Eframe[X] = 0;
    	acc_Eframe[Y] = 0;
    	acc_Eframe[Z] = 0;
    }

	position_test += velocity_test*0.01f + 0.5* acc_Eframe[Z]* 0.01f * 0.01f;
	velocity_test += acc_Eframe[Z]*0.01f;

	AHRS.pitch = -atan2_approx(-dcm[0][2],sqrtf(1 - dcm[0][2]*dcm[0][2]))*DEG;// - pitch_trim_imu;
	AHRS.roll = -atan2_approx(-dcm[1][2],dcm[2][2])*DEG;//  - roll_trim_imu;
	float yaw_ = -atan2_approx(dcm[0][1],dcm[0][0])*DEG;
	if(yaw_ < 0){
		 yaw_ = 360 + yaw_;
	}
	AHRS.yaw =  yaw_;
	AHRS.roll_rate  = gx_/config.gyr_lsb;
	AHRS.pitch_rate = -gy_/config.gyr_lsb;
	AHRS.yaw_rate   = -gz_/config.gyr_lsb;

}

