#include "baro.h"
#include "../Driver/ibus.h"
#include "../Driver/ms5611.h"
#include "../Driver/bmp280.h"
#include "../Lib/maths.h"

#define BMP280
//#define MS5611

int8_t baro_calib;
int32_t alt_offset;

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

int32_t baro_get_altitude(){
     if(baro_calib){
        int32_t alt = bmp280_read_fixed() - alt_offset;
        return alt;
     }
     return 0;
}
