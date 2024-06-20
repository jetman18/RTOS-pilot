#include "../Lib/gps.h"
#include "../Lib/maths.h"
#include "../Lib/utils.h"
#include "../Lib/timer.h"
#include "plane.h"

#define  EARTH_RADIUS  6356752 
#define  IS_STICK_TURN_ON(x)  ((x > 1600)?1:0)
#define  CRICLE_RADIUS 100  // m
#define MAX_WAYPOINT 20

typedef enum{
  NULL_WP = 0,
  HOME_WP,
  TAKEOFF_WP,
  NOMAL_WP,
  LAND_WP
}wp_type;

typedef struct{
  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  uint8_t Type;
}waypoint;

static waypoint wp[MAX_WAYPOINT];

static int32_t home_latitude;
static int32_t home_longitude;
static int32_t loiter_latitude;
static int32_t loiter_longitude;
static int32_t latitude_g;
static int32_t longitude_g;
static int wp_init = 1;
static int wp_index = 0;

uint8_t  autopilot_priority = 0;
uint8_t  circle_priority = 1;
uint8_t  rtHome_priority = 2;

uint16_t autopilot_stick;
uint16_t circle_stick;
uint16_t rtHome_stick;

static float course_over_ground(int lat1,int lon1,int lat2, int lon2);
static int distanceBetweenTwoPoint(int lat1,int lon1,int lat2, int lon2);

/*
 * Reset navigation task
 */
void navigation_reset(){
   wp_init = 1;
   wp_index = 0;
}


/**************** Using Vector field method to guidance UAV *********************************/
/* Perform circle loiter
 * Param 1 latitude   10^7
 * Param 2 longitude  10^7
 * Param 3 center latitude   10^7
 * Param 4 center longitude  10^7
 * Param 5 Radius circle
 * Return yaw desired  
 */
static float loiter(uint32_t aircraf_lat,uint32_t aircraf_lon, int32_t center_lat, int32_t center_lon ,uint32_t radius_desired)
{    
     // caculate distance to circle
     int dist = distanceBetweenTwoPoint(aircraf_lat,aircraf_lon,center_lat,center_lon);
     // distance to circle
     dist = dist - CRICLE_RADIUS;
     const float Kc = 500.0f/CRICLE_RADIUS;
     // calc heading to center of circle
     float bearing = course_over_ground(aircraf_lat,aircraf_lon,center_lat,center_lon);
     // calc yaw desired to control UAV
     float yaw_target = bearing + 90.0f - MAX(fabs(crossTrack*Kc),90)*sign(crossTrack);
     // constrain range
     yaw_target = range360(yaw_target);
     return yaw_target;
}

/* Perform waypoint follow
 * Return yaw desired  
 */
static float waypointFollow(){
   static float Kc = 0.22;
   static float Kd = 1;
   static uint8_t wp_count = 0;
   static int32_t nearest_wp = 100000; // 100 km
   if(wp_init){
      // Find the nearest coordinates
      for(int i = 0; i < MAX_WAYPOINT; i++){
         if(wp[i].Type != 0){
            int dis = distanceBetweenTwoPoint(latitude_g,longitude_g,wp[i].lat,wp[i].lon);
            if (dis <= nearest_wp){
               nearest_wp = dis;
               wp_index = i;
            }
         }else{

            break;
         }
      }
      wp_init = 0;
   }
   float path_angle = course_over_ground(wp[wp_index - 1].lat,wp[wp_index - 1].lon,wp[wp_index].lat,wp[wp_index].lon);
   int32_t dis2nextwp = distanceBetweenTwoPoint(latitude_g,longitude_g,wp[wp_index].lat,wp[wp_index].lon);
   float phi = course_over_ground(latitude_g,longitude_g,wp[wp_index].lat,wp[wp_index].lon);
   phi = path_angle - phi;
   if(abs(dis2nextwp) < 100 || fabs(phi) > 80){
      wp_index ++;
      if(wp_index == wp_count)
         wp_index = 0;
   }
   phi = range180(phi)*toRAD;
   int32_t cross_track = dis2nextwp*sin_approx(phi);
   float temp_angle = powf(fabs(cross_track*Kc),Kd);
   float theta = path_angle - MAX(temp_angle,90)*sign(cross_track);
   theta = range360(theta);
   return theta;
}

/* Start autopilot mode
**/
void autoPilot(){
   static uint32_t timer;
   uint32_t dt = millis() - timer;
   timer = millis();
   static int8_t circle_init = 1;
   static float yaw_cmd;
   if(STICK_CHECK(circle_stick)){
       if(circle_init){
         loiter_latitude = latitude_g;
         loiter_longitude = longitude_g;
         circle_init = 0;
       }
      yaw_cmd = circleFly(loiter_latitude,loiter_longitude,CRICLE_RADIUS);

   }else{
      // reset
      circle_init = 1;
   }


}

/* Calculate course over ground
 * Return 0 - 360
 */
static float course_over_ground(int lat1,int lon1,int lat2, int lon2){
     const float scaler = RAD*EARTH_RADIUS/(1e+7f);
     float dx = (lat2 - lat1)*scaler;
     float dy = (lon2 - lon1)*scaler;

     float anlpha  = fabs(atan2(dx,dy)*DEG);
     float belta   = fabs(atan2(dy,dx)*DEG);
     float angle;
     if(dx >= 0 && dy >= 0)
         angle = anlpha;
     else if(dx >= 0 && dy <= 0)
        angle = 90 + belta;
     else if(dx <= 0 && dy <= 0)
        angle = 90 + belta;
     else if(dx <= 0 && dy >= 0)
        angle = 360 - anlpha;
   return angle;
}


/* Calculate distance between two waypoint
 * Return distance in meter 
 */
static int distanceBetweenTwoPoint(int pre_lat,int pre_lon,int lat, int lon){
   const float scaler = RAD*EARTH_RADIUS/(1e+7f);
   float a_temp =  (lat - pre_lat)*scaler;
   float b_temp =  (lon - pre_lon)*scaler;
   int dis = sqrtf(a_temp*a_temp + b_temp*b_temp);
   return dis;
}


/*
 * This function set home position
 */
void navigation_set_home_pos(int lat, int lon, int alt){
   wp[0].Type = HOME_WP;
   wp[0].latitude = lat;
   wp[0].longitude = lon;
   wp[0].altitude = alt;
}

