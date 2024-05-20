#include "../Lib/gps.h"
#include "../Lib/maths.h"
#include "../Lib/utils.h"
#include "../Lib/timer.h"
#include "plane.h"

#define  EARTH_RADIUS  6356752 
#define IS_STICK_TURN_ON(x)  ((x > 1600)?1:0)

#define CRICLE_RADIUS 100  // m

typedef enum{
  NULL_WP = 0,
  TAKE_OFF,
  WAYPOINT,
  LANDING
}wp_type;

typedef struct{
  int32_t lat;
  int32_t lon;
  int32_t alt;
  int8_t Type;
}waypoint;


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


static float waypointBearing(int lat1,int lon1,int lat2, int lon2);
static int distanceBetweenTwoPoint(int lat1,int lon1,int lat2, int lon2);

/*

static float loiter(uint32_t aircraf_lat,uint32_t aircraf_lon,uint32_t radius_desired)
{    
   // caculate distance to circle
     int dist = distanceBetweenTwoPoint(latitude_g,longitude_g,loiter_latitude,loiter_longitude);
     dist = dist - CRICLE_RADIUS;
     float Kc = 500.0f/CRICLE_RADIUS;
     float bearing = waypointBearing(latitude_g,longitude_g,loiter_latitude,loiter_longitude);
     float yaw_target = bearing + 90.0f - MAX(fabs(crossTrack*Kc),90)*sign(crossTrack);
     yaw_target = range360(yaw_target);
     return yaw_target;
}
*/
/*
static float waypointFollow(){
   static float Kc = 0.22;
   static float Kd = 1;
   static uint8_t wp_count;
   static int32_t min_distance = 100000; // 100 km
   if (wp_init){
      wp_count = 0;
      for(int i = 0;i<MAX_WAYPOINT;i++){
         if(wp[i].Type != NULL_WP){
            int dis = distanceBetweenTwoPoint(latitude_g,longitude_g,wp[i].lat,wp[i].lon);
            if (dis <= min_distance)
               {
               min_distance = dis;
               wp_index = i + 1;
               }
            wp_count ++;
         }
         else 
            break;
      }
      wp_init = 0;
   }
   float path_angle = waypointBearing(wp[wp_index - 1].lat,wp[wp_index - 1].lon,wp[wp_index].lat,wp[wp_index].lon);
   int32_t dis2nextwp = distanceBetweenTwoPoint(latitude_g,longitude_g,wp[wp_index].lat,wp[wp_index].lon);
   float phi = waypointBearing(latitude_g,longitude_g,wp[wp_index].lat,wp[wp_index].lon);
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
*/


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

/*
static float waypointBearing(int lat1,int lon1,int lat2, int lon2){
     float tem = toRAD*EARTH_RADIUS/(1e+7f);
     float dx = (lat2 - lat1)/(1e+7f)*tem;
     float dy = (lon2 - lon1)/(1e+7f)*tem;
     float anlpha  = fabs(atan2(dx,dy)*toDEG);
     float belta   = fabs(atan2(dy,dx)*toDEG);
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

static int distanceBetweenTwoPoint(int lat1,int lon1,int lat2, int lon2){
   float scaler = RAD*EARTH_RADIUS/(1e+7f);
   float a_temp =  (lat2 - lat1)*scaler;
   float b_temp =  (lon2 - lon1)*scaler;
   int dis = sqrtf(a_temp*a_temp + b_temp*b_temp);
   return dis;
}
*/
