#include "baro.h"
#include "../Driver/ibus.h"
#include "../Driver/ms5611.h"
#include "../Driver/bmp280.h"
#include "../Lib/maths.h"
#include"../Lib/filter.h"

#define BMP280
//#define MS5611

int8_t baro_calib;
int32_t alt_offset;
int32_t altitude_filted;
int32_t climb_rate;
void baro_init(){
    baro_calib = FALSE;
    alt_offset = 0;
#ifdef BMP280
    bmp280_init(&hi2c2);
    for(int i=0; i< 100; i++){
    	bmp280_read_fixed();
    	HAL_Delay(5);
    }
#elif MS5611
    ms5611_init(&hi2c2);
#endif
}


void baro_zero_calibrate(){
    int32_t altitude = 0;
    climb_rate = 0;
    static int16_t count = 0;
#ifdef BMP280
    altitude = bmp280_read_fixed();
#elif MS5611
    altitude = ms5611_read_fixed();
#endif
    alt_offset += altitude;
    count ++;
    if(count > 100){
        alt_offset /= 100;
        baro_calib = TRUE;
        count = 101;
    }
}

int8_t is_baro_calibration(){
      return baro_calib;
}

void baro_update(float dt){  //100 hz update
    if(baro_calib != 1){
        return;
    }
    static uint16_t count = 0;
    static int32_t pre_alt = 0;
    int32_t alt = bmp280_read_fixed() - alt_offset;  // cm
    altitude_filted += pt1FilterGain(5,dt)*(alt - altitude_filted);
    // calculate climb rate at 5hz
    if(count %20 == 0){
        int climb = (altitude_filted - pre_alt)/(dt*20);
        pre_alt = altitude_filted;
        // apply low-pass filter
        climb_rate += pt1FilterGain(1,dt*20)*(climb - climb_rate);
    }
    count ++;
}

int32_t baro_get_climbCm(){
    return climb_rate;
}


int32_t baro_get_altCm(){
    return altitude_filted;
}
