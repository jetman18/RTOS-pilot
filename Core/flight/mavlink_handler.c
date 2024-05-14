#include "plane.h"
#include "usart.h"

#include "../Driver/ibus.h"

#include "../Lib/gps.h"
#include "../Lib/maths.h"
#include "../Lib/timer.h"
#include "../Lib/imu.h"
#include "../Lib/utils.h"
#include "../mavlink/ardupilotmega/mavlink.h"

#define MAX_LENGHT 200


//static mavlink_attitude_t attitude;
static mavlink_message_t msg;
//static mavlink_status_t msg_status;
static uint8_t data;
static uint8_t index_;
uint8_t sys_id,com_id;
static UART_HandleTypeDef *uart;
uint8_t buffer__[MAX_LENGHT];
static int isTxcpl;
uint32_t send_time_us;

extern float v_estimate;

void mavlinkInit(uint8_t syss_id, uint8_t comm_id,UART_HandleTypeDef *uartt,uint32_t baudrate){
    isTxcpl = 1;
    index_ =0;
	sys_id  = syss_id;
    com_id  = comm_id;
	uart = uartt;
    uartt->Init.BaudRate = baudrate;
	HAL_UART_Init(uartt);
	HAL_UART_Receive_IT(uart, &data,1);
}

UART_HandleTypeDef *mavlink_uart_port(){
    return uart;
}

/*
void mavlinkCallback(){
    if (mavlink_parse_char(MAVLINK_COMM_0,data,&msg, &msg_status))
    {
        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_ATTITUDE:
            mavlink_msg_attitude_decode((const mavlink_message_t*)&msg,&attitude);
            break;
        case MAVLINK_MSG_ID_NAMED_VALUE_INT:
        	mavlink_msg_named_value_int_decode((const mavlink_message_t*)&msg,&val_int);
            break;
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
        	mavlink_msg_named_value_float_decode((const mavlink_message_t*)&msg,&val_float);
            break;
        }
    }
    HAL_UART_Receive_IT(uart, &data,1);
}
*/

void mavlink_send_attitude(float roll,float pitch, float yaw){
	static uint8_t count = 0;
	uint16_t len = 0;
	if(isTxcpl)
	{
		if(count < 10){
			mavlink_msg_attitude_pack(sys_id,com_id,&msg,0,roll,pitch,yaw,0,0,0);
		}
		else{ // send heartbeat
			uint8_t type = MAV_TYPE_FIXED_WING;
			uint8_t autopilot = MAV_AUTOPILOT_GENERIC;
			uint8_t base_mode = MAV_MODE_FLAG_TEST_ENABLED;
			mavlink_msg_heartbeat_pack(sys_id,com_id,&msg,type,autopilot,base_mode,base_mode, MAV_STATE_UNINIT);
		}
		len = mavlink_msg_to_send_buffer(buffer__,&msg);
		HAL_UART_Transmit_DMA(uart,buffer__,len);
		count ++;
		if(count > 10){
			count = 0;
		}
		isTxcpl = 0;
	}
}

/* This function send data to miniOSD board at 10Hz
*   Attitude
*   GPS 
*   RC signal
*   heartbeat
*/
void mavlink_osd(){
	static uint8_t count_ = 0;
	static uint8_t count2_ = 0;
	if(isTxcpl)
	{
		switch (count_)
		{
		// fast display
		case 0:  
			mavlink_msg_attitude_pack(sys_id,com_id,&msg,0,AHRS.roll*RAD,AHRS.pitch*RAD,0,0,0,0);
			count_ ++;
			break;
		case 1:  
			float airspeed = 314;    // osd in km/h
			float groundspeed = 57;  // osd in km/h
			uint16_t throttle = (ibusChannelData[CH3] - 1000)*0.1f;
			float alt = 333;
			float climb = 5;
			mavlink_msg_vfr_hud_pack(sys_id,com_id,&msg,
						airspeed, groundspeed,AHRS.yaw, throttle,alt, climb);
			count_ ++;
			break;
		case 2:
			switch (count2_)
			{
			case 0:
				//mavlink_msg_gps_raw_int_pack(sys_id,com_id,&msg,
				//						0, _gps.fix,_gps.position[0],_gps.position[1],1200,
				//						1 , 1 , 2200,0 ,abs(v_estimate));
				count2_ ++;

				break;
			case 1:
			    uint8_t rssi = mapI(ibusChannelData[CH11],1000,2000,0,255);
				mavlink_msg_rc_channels_raw_pack(sys_id,com_id,&msg,
						0,0,ibusChannelData[CH4],ibusChannelData[CH7],ibusChannelData[CH8], 
						0,0,0,
						0,0,rssi);
				count2_ ++;
				break;
			case 2: // send heartbeat
				uint8_t type = MAV_TYPE_FIXED_WING;
				uint8_t autopilot = MAV_AUTOPILOT_GENERIC;
				uint8_t base_mode = MAV_MODE_FLAG_TEST_ENABLED;
				mavlink_msg_heartbeat_pack(sys_id,com_id,&msg,type,autopilot,base_mode,base_mode, MAV_STATE_UNINIT);
				count2_ = 0;
				break;

            /*
			case 3:
			    float wind_direction = 0;
				float wind_speed = 15;  // osd in km/h
				mavlink_msg_wind_pack(sys_id,com_id,&msg,
						       wind_direction,wind_speed,0);
				count2_  = 0;
				break;
            
			case 4:
			    float q[4];
				float thrust = (float)(ibusChannelData[CH3] - 1000)/1000;
		        mavlink_msg_attitude_target_pack(sys_id,com_id,&msg,0,
				                                     0,q,0,0,0,thrust);
		        count2_ = 0;
				break;
			*/
			}
			count_ = 0;
		}
		uint16_t len_data = mavlink_msg_to_send_buffer(buffer__,&msg);
	    HAL_UART_Transmit_DMA(uart,buffer__,len_data);
	    isTxcpl = 0;
	}
}



void mavlink_send_heartbeat(){
  if(isTxcpl){
	uint8_t type = MAV_TYPE_FIXED_WING;
	uint8_t autopilot = MAV_AUTOPILOT_GENERIC;
	uint8_t base_mode = MAV_MODE_FLAG_TEST_ENABLED;
	mavlink_msg_heartbeat_pack(sys_id,com_id,&msg,type,autopilot,base_mode,base_mode, MAV_STATE_UNINIT);
	uint16_t len = mavlink_msg_to_send_buffer(buffer__,&msg);
	HAL_UART_Transmit_DMA(uart,buffer__,len);
  isTxcpl = 0;
 }
}

/*
 * 
 */
uint32_t temp;
void mavlink_tx_cpl_callback()
{   
	//send_time_us = millis() - temp;
	//temp = millis();
	isTxcpl = 1;
}

