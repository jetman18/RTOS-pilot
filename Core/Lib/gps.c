#include "gps.h"
#include "math.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "timer.h"
#include "gpsconfig.h"
#include "maths.h"
#include "usart.h"

#define FALSE 0
#define TRUE  1
#define LON   0
#define LAT   1
#define DOWN  2
#define UBLOX_BUFFER_SIZE 100

/* Using Ubx protocol
* 
*/
enum {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_SVINFO = 0x30,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
} ubs_protocol_bytes;



enum {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
	FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
} ubs_nav_fix_type;

enum {
    NAV_STATUS_FIX_VALID = 1
} ubx_nav_status_bits;

enum{
   GPS_UNKNOWN
};

// UBX support
typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} ubx_header;

typedef struct {
    uint32_t time;              // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
} ubx_nav_posllh;

typedef struct {
    uint32_t time;              // GPS msToW
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t differential_status;
    uint8_t res;
    uint32_t time_to_first_fix;
    uint32_t uptime;            // milliseconds
} ubx_nav_status;

/*
typedef struct {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
} ubx_nav_solution;
*/
typedef struct
{
    uint32_t time;
    uint8_t version;
    uint8_t numSvs;
    
    /* data */
}ubx_nav_sat;



typedef struct {
    uint32_t time;              // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
} ubx_nav_velned;

typedef struct {
    uint8_t chn;                // Channel number, 255 for SVx not assigned to channel
    uint8_t svid;               // Satellite ID
    uint8_t flags;              // Bitmask
    uint8_t quality;            // Bitfield
    uint8_t cno;                // Carrier to Noise Ratio (Signal Strength)
    uint8_t elev;               // Elevation in integer degrees
    int16_t azim;               // Azimuth in integer degrees
    int32_t prRes;              // Pseudo range residual in centimetres
} ubx_nav_svinfo_channel;

typedef struct {
    uint32_t time;              // GPS Millisecond time of week
    uint8_t numCh;              // Number of channels
    uint8_t globalFlags;        // Bitmask, Chip hardware generation 0:Antaris, 1:u-blox 5, 2:u-blox 6
    uint16_t reserved2;         // Reserved
    ubx_nav_svinfo_channel channel[16];         // 16 satellites * 12 byte
} ubx_nav_svinfo;

gpsData_t _gps;
static UART_HandleTypeDef *_gpsUartPort;
uint32_t _therad_read_time_ms;
// State machine state
static uint8_t _msg_id;
static int8_t receive_cplt = 0;
//static uint8_t gps_buffer[200];
static uint8_t _char;

int32_t offset_alt;
int8_t gps_alt_zero_calibrate;
static union {
    ubx_nav_posllh posllh;
    ubx_nav_status status;
    //ubx_nav_solution solution;
    ubx_nav_velned velned;
    //ubx_nav_svinfo svinfo;
    uint8_t bytes[UBLOX_BUFFER_SIZE];
} _buffer;


static uint8_t newdata(uint8_t data);
static uint8_t parse_msg();
uint8_t gps_buffer[200];
/*  
 * Init function 
 */
void gps_init(UART_HandleTypeDef *uart,uint32_t baudrate)
{
    offset_alt = 0;
    gps_alt_zero_calibrate = FALSE;
	_gpsUartPort = uart;
    _gps.timer_ = millis();

    _msg_id = 0;

    HAL_Delay(300);
    /* disable NMEA */
    HAL_UART_Transmit(&huart3,disable_NMEA_MSG,sizeof(disable_NMEA_MSG),1000);
    HAL_Delay(100);
    /* enable UBX */
    HAL_UART_Transmit(&huart3,enable_UBX_MSG,sizeof(enable_UBX_MSG),1000);
    HAL_Delay(100);
    /* set max rate */
    HAL_UART_Transmit(&huart3,set_rate_50hz,sizeof(set_rate_50hz),1000);
    HAL_Delay(100);

    /* set gps baudrate */
    //HAL_UART_Transmit(&huart3,uart38400,sizeof(uart38400),1000);
    //HAL_Delay(100);
    HAL_UART_Receive_DMA(&huart3,gps_buffer,200);
    // set baudrate
    //huart3.Init.BaudRate = 38400 ;
	//HAL_UART_Init(&huart3);
    // read gps using interrupt
	//HAL_UART_Receive_IT(&huart3, &_char,1);
}

UART_HandleTypeDef *gps_uart_port(){
    return _gpsUartPort;
}


void gps_readout(){
    uint8_t buffer_index = 0;
    if(receive_cplt){

        while(1){
            newdata(gps_buffer[buffer_index++]);

            if(buffer_index >= 200){
            	 break;
            }
        }

        HAL_UART_Receive_DMA(&huart3,gps_buffer,200);
        receive_cplt = 0;
    }
}

uint32_t ms_gps_thread;
void gps_DMA_callback()
{
    static uint32_t last_call;
    ms_gps_thread = millis() - last_call;
    last_call = millis();
    receive_cplt = 1;
}

/*
void gps_callback()
{
    newdata(_char);
    HAL_UART_Receive_IT(&huart3,&_char,1);
}
*/

/*
static void _update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    while (len--) {
        *ck_a += *data;
        *ck_b += *ck_a;
        data++;
    }
}
*/

uint32_t cnt_vel,cnt_status,cnt_poss;

static uint8_t parse_msg(){
    static uint8_t _new_speed;
    static uint32_t lastPosUpdateTime;
    static uint8_t _new_position;
    static uint8_t gps_cali_count = 0;
    switch (_msg_id) {
        case MSG_POSLLH:
        	cnt_poss ++;
            _gps.position[LON] = _buffer.posllh.longitude;
            _gps.position[LAT] = _buffer.posllh.latitude;
            if(gps_alt_zero_calibrate == FALSE){
                if(gps_cali_count < 20){
                    offset_alt += _buffer.posllh.altitude_msl;
                    gps_cali_count ++;
                }
                else{
                    gps_alt_zero_calibrate = TRUE;
                }
            }
            else{
               _gps.altitude_mgl = _buffer.posllh.altitude_msl - offset_alt/20; 
            }
            _gps.altitude_msl = _buffer.posllh.altitude_msl - offset_alt; 
            _gps.horizontalAccuracy = _buffer.posllh.horizontal_accuracy;
            _gps.VerticalAccuracy = _buffer.posllh.vertical_accuracy;
            /* time update position */
            if(lastPosUpdateTime == 0){
                lastPosUpdateTime = millis();
                break;
            }
            _gps.posUpdateTime = millis() - lastPosUpdateTime;
            lastPosUpdateTime = millis();
            /* flag set */
            _new_position = TRUE;
            break;
        case MSG_STATUS:

        	cnt_status ++;

            //next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
            //if (!next_fix)
            //    _gps.fix = FALSE;
            _gps.fix = _buffer.status.fix_type;
            break;
        /*
        case MSG_SOL:
            //next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
            //if (!next_fix)
            //    _gps.fix = FALSE;
            _gps.fix = _buffer.solution.fix_type; --------------------------------------------------------------------------------------
            _gps.numSat = _buffer.solution.satellites; -------------------------------------------------------------------------------------
            break;
        */
        case MSG_VELNED:
        	cnt_vel ++;
            _gps.velocity[LAT] = _buffer.velned.ned_north;
            _gps.velocity[LON] = _buffer.velned.ned_east;
            _gps.velocity[DOWN] = _buffer.velned.ned_down;
            _gps.Gspeed = _buffer.velned.speed_2d;
            _gps.ground_course = _buffer.velned.heading_2d;
            _gps.speedAccuracy = _buffer.velned.speed_accuracy;
            _gps.headingAccuracy = _buffer.velned.heading_accuracy;
            _new_speed = TRUE;
            break;
        case MSG_SVINFO:
           /*
            numCh = _buffer.svinfo.numCh;
            if (numCh > 32)
                numCh = 32;
            for (i = 0; i < numCh; i++) {
                svinfo_chn[i] = _buffer.svinfo.channel[i].chn;
                svinfo_svid[i] = _buffer.svinfo.channel[i].svid;
                svinfo_quality[i] = _buffer.svinfo.channel[i].quality;
                svinfo_cno[i] = _buffer.svinfo.channel[i].cno;
            }
            // Update GPS SVIFO update rate table.
            svinfo_rate[0] = svinfo_rate[1];
            svinfo_rate[1] = millis();
            */
            break;
        default:
            return FALSE;
    }
    if (_new_position && _new_speed) {
        _new_speed = _new_position = FALSE;
        return TRUE;
    }
    return FALSE;
}

static uint16_t _payload_length = 0;
static uint16_t _payload_counter = 0;
static uint8_t newdata(uint8_t data){
    uint8_t parsed = FALSE;
    static uint8_t _ck_a;
    static uint8_t _ck_b;
    static uint8_t _step = 0;
    switch (_step) {
        case 0: // Sync char 1 (0xB5)
            if (PREAMBLE1 == data)
                _step++;
            break;
        case 1: // Sync char 2 (0x62)
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
        case 2: // Class
            _step++;
            _ck_b = _ck_a = data;  
            break;
        case 3: // ID
            _step++;
            _ck_b += (_ck_a += data);       
            _msg_id = data;
            break;
        case 4: 
            _step++;
            _ck_b += (_ck_a += data);       
            _payload_length = data; 
            break;
        case 5: 
            _step++;
            _ck_b += (_ck_a += data);      
            _payload_length += (uint16_t)(data << 8);
            if (_payload_length > UBLOX_BUFFER_SIZE) {
                _payload_length = 0;
                _step = 0;
            }
            _payload_counter = 0;   
            break;
        case 6:
            _ck_b += (_ck_a += data);     
            if (_payload_counter < UBLOX_BUFFER_SIZE) {
                _buffer.bytes[_payload_counter] = data;
            }
            if (++_payload_counter == _payload_length)
                _step++;
            break;
        case 7:
            _step++;
            if (_ck_a != data)
                _step = 0;         
            break;
        case 8:
            _step = 0;
            if (_ck_b != data)
                break;   
            if(parse_msg())
            {
                parsed = TRUE;
            }
    } 
    return parsed;
}



