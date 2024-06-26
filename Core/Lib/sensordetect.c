#include "sensordetect.h"
#include "i2c.h"

#include "blackbox.h"

typedef struct{
	uint8_t addr;
	char *name;
    int8_t valid;
}sensor_t;

sensor_t sensor_list[SENSOR_COUNT] ={
   { 0x68,"0x68 mpu6050", -1 },
   { 0x78,"0x78 ssd1306", -1 },
   { 0x0d,"0x0d qmc5883l",-1 },
   { 0x1e,"0x1e hmc5883l",-1 },
   { 0x77,"0x77 ms5611"  ,-1 }
   /* more */
};
uint8_t sensor_[SENSOR_COUNT];
/*
 * Scan i2c address 0 to 127
 * Return number of sensor
 */
void i2cDectect(I2C_HandleTypeDef *i2c){
	uint8_t sensor_count = 0;

	for(int i=0; i<128; i++){
		uint8_t temp = HAL_I2C_IsDeviceReady(i2c,(uint16_t)(i<<1), 3,1000);
		if(temp == HAL_OK){
            for(int j = 0; j < SENSOR_COUNT;j ++){
                if(sensor_list[j].addr == i){
                    sensor_list[j].valid = 0;
                }
            }
			sensor_[sensor_count] = i;
			sensor_count ++;
			if(sensor_count == SENSOR_COUNT){
			     break;
			}
		}
	}
	
}


/*
 * return sensor status 0-> valid -1 invalid 
 */
int8_t get_sensor_status(uint8_t address){
    for( int i = 0; i< SENSOR_COUNT; i ++){
        if(sensor_list[i].addr == address){
            return  sensor_list[i].valid;
        }
    }	
    return -1;
}

