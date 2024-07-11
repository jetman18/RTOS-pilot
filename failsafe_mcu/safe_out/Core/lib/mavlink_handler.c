#include "mavlink_handler.h"
#include "../mavlink/common/mavlink.h"
#include "timer.h"
#define MAX_LOST_DATA 5

static UART_HandleTypeDef *uart;
mavlink_rc_channels_raw_t rc_channels_raw;
mavlink_status_t  msg_status;
mavlink_message_t msg;
static uint8_t c;
uint8_t reset_state = 1;
uint32_t ms_boottime = 0;
uint8_t iSfcdead_ = 1;
static uint8_t fc_lost_cnt = 0;


void mavlink_init(UART_HandleTypeDef *huart){
	uart = huart;
	HAL_UART_Receive_IT(uart, &c,1);
}

static uint32_t rc_msg_call_ms;
void mavlink_callback(){
    if (mavlink_parse_char(MAVLINK_COMM_0,c,&msg, &msg_status)){
        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        	rc_msg_call_ms = HAL_GetTick();
        	mavlink_msg_rc_channels_raw_decode((const mavlink_message_t*)&msg,&rc_channels_raw);
        	if(reset_state){
        		ms_boottime = rc_channels_raw.time_boot_ms;
        		reset_state =  0;
        		break;
        	}
       		uint32_t ms_check = rc_channels_raw.time_boot_ms - ms_boottime;
       		ms_boottime = rc_channels_raw.time_boot_ms;
       		if(ms_check == 0 ){
       			 fc_lost_cnt++;
       		}
       		if(fc_lost_cnt > MAX_LOST_DATA){
       			fc_lost_cnt = MAX_LOST_DATA;
       			iSfcdead_ = 0;   // fc is dead hmm
       		}
            break;
        default:
            //mavlink_msg_attitude_decode((const mavlink_message_t*)&msg,&attitude);
            break;
        }
    }
    HAL_UART_Receive_IT(uart, &c,1);
}

uint8_t mavlink_check_fc_healthy(){
	uint32_t wait_time = HAL_GetTick() - rc_msg_call_ms;
	if(wait_time > 500)  // 0.5 s
	    return 0;
     return iSfcdead_;
}

