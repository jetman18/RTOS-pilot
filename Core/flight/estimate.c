#include "../Lib/maths.h"
#include "../Lib/utils.h"
#include "../Lib/imu.h"
#include "../Driver/ibus.h"
#include "../Lib/maths.h"
#include "../Driver/ms5611.h"

/*
#define dt 0.01f

float climb_rate_esitmate;
float altitude_estimate;
float acc_estimate;

float Q_acc = 0.001;
float R_climb_measurement = 0.2f; 
float R_alt_measurement = 0.2f; 

float P[3][3];

void kalman_init(){
    climb_rate_esitmate = 0;
    altitude_estimate = 0;
    acc_estimate = 0;
    ms5611_init(&hi2c2);
}

void kalman_altitude_estimate(){
    // state estimate
    altitude_estimate = altitude_estimate + climb_rate_esitmate*dt + 0.5*sq(dt)*acc_estimate;
    climb_rate_esitmate = climb_rate_esitmate + dt*acc_estimate;
    acc_estimate = acc_Eframe[Z];

    // variance estimate
    P[0][0] = P[0][0] + dt*(P[0][1] + P[1][1]*dt + (P[2][1]*sq(dt))/2) + P[1][0]*dt + (sq(dt)*(P[0][2] + P[1][2]*dt + (P[2][2]*sq(dt))/2))/2 + (P[2][0]*sq(dt))/2 + (Q_acc*sq(dt)^2)/4;
    P[0][1] = P[0][1] + dt*(P[0][2] + P[1][2]*dt + (P[2][2]*sq(dt))/2) + P[1][1]*dt + (P[2][1]*sq(dt))/2 + (Q_acc*dt*sq(dt))/2;
    P[0][2] = P[0][2] + P[1][2]*dt + (P[2][2]*sq(dt))/2 + (Q_acc*sq(dt))/2;

    P[1][0] = P[1][0] + P[2][0]*dt + dt*(P[1][1] + P[2][1]*dt) + (sq(dt)*(P[1][2] + P[2][2]*dt))/2 + (Q_acc*dt*sq(dt))/2;
    P[1][1] = P[1][1] + P[2][1]*dt + dt*(P[1][2] + P[2][2]*dt) + Q_acc*dt^2;
    P[1][2] = P[1][2] + P[2][2]*dt +  Q_acc*dt;

    P[2][0] = P[2][0] + P[2][1]*dt + (P[2][2]*sq(dt))/2 + (Q_acc*sq(dt))/2;
    P[2][1] = P[2][1] + P[2][2]*dt + Q_acc*dt;
    P[2][2] = P[2][2] + Q_acc;

    // correction
    float K1 =  P[0][0];
}

*/

extern float acc_Eframe[];
float vn,ve,vd;
float pn,pe,pd;
int8_t reset_state = 1;

extern float bmp280_altitude_;
extern float climb_rate_baro;
void position_estimate(float dt){
	if(reset_state){
		vn = 0;
		ve = 0;
		vd = 0;
		pn = 0;
		pe = 0;
		pd = 0;
		reset_state = 0;
	}
	//pn += vn*dt + 0.5*sq(dt)*acc_Eframe[X];
	//pe += ve*dt + 0.5*sq(dt)*acc_Eframe[Y];
	pd += vd*dt + 0.5*sq(dt)*acc_Eframe[Z];

	pd = pd*0.95f + 0.05*(bmp280_altitude_/10.0);
    pe = pd*10;

	//vn += dt*acc_Eframe[X];
	//ve += dt*acc_Eframe[Y];
	vd += dt*acc_Eframe[Z];
	vd = vd * 0.9f + 0.1f*(climb_rate_baro/10.0);
}





static float gravity  = -9.81 ;
//static float toDeg = 57.29577;
//static float toRad = 0.01745;
static float Cd = 0.01;
static float weigh = 0.8; // kg

static float velocity = 0;

float dynamic_speed_esitmate(float dt){
    float Thrust = (float)(ibusChannelData[CH3] - 1000) * 0.009;
    float acc = (Thrust - sign(velocity)*sq(velocity)*Cd + weigh * gravity * sin_approx(AHRS.pitch*RAD))/weigh;
    velocity += acc*dt;
    return velocity;
}


/*
#define Dt 0.0125f
// constan variables
static float pi = 3.14159;
static float eart_radius =  6371000;  // m
static float gravity      = -9.81 ;
static float toDeg = 57.29577;
static float toRad = 0.01745;

// dynamic parameters
float air_density     = 1.293;
float mass            = 0.9;         // weight of aircraft
float wing_area       = 0.21;        // m*m  ref area
float wing_ctrl_area  = 0.015;       // control surface area
float rudder_area     = 0.0168;      // rudder surface ares
float Cd_o            = 0.14;        // drag coeffient zero
float Cl_0            = 0.01;
float aileron_Cl      = 0.011;
float aileron_Cd      = 0.0013;
float dis_ruderr2CG   = 0.12;
float dis_aile2center = 0.12;
float dis_ele2center  = 0.2;
float max_aileron_angle = 20;        // max control angle

static float swap180(float val);
static float swap360(float val);

// attitude
extern float cosx,cosy,cosz,sinx,siny, sinz,tany;
extern attitude_t AHRS;
static uint8_t isFlying = 0;
uint8_t pre_run = 1;
float alpha,beta;   // sideslip angle

static float pitch = 0;//10*toRad;
static float yaw = 0;

float vel_estimate_North,vel_estimate_East,vel_estimate_Up;
float pos_estimate_North,pos_estimate_East,pos_estimate_Up;
const float max_thrust = 0.8;   // N
float thrust;
*/

/*
 *   Estimate velocity and postion in earth frame
 **/

/*
void estimates_start(){
    if(pre_run){
        vel_estimate_North = 0, vel_estimate_East = 0,vel_estimate_Up = 0;
        pos_estimate_North = 0, pos_estimate_East = 0,pos_estimate_Up = 0;
        alpha = 0, beta = 0;
        thrust    = 0;
        pre_run = 0;
        return;
    }
    // update thrust
    thrust = ((float)ibusChannelDataCH3 - 1000)/1000*max_thrust;

    // alpha
    float v_horizon = sqrt(vel_estimate_North*vel_estimate_North + vel_estimate_East*vel_estimate_East);
    float temp_a = atan2_approx(vel_estimate_Up,v_horizon)*toDeg;
    temp_a =  AHRS.pitch - temp_a;

     // beta
    float temp_beta  = abs(atan2(vel_estimate_East,vel_estimate_North)*toDeg);
    float beta_t = 0;
    if  (vel_estimate_East >= 0)
        beta_t = temp_beta;
    else if  (vel_estimate_East <= 0)
        beta_t = 360 - temp_beta;
    beta_t = AHRS.yaw - beta_t;  

    if (beta_t < -180)
        beta_t = beta_t + 360;
    else if (beta_t > 180)
        beta_t = beta_t - 360;

    alpha =  temp_a*cosx + beta_t*sinx;
    beta  = -temp_a*sinx + beta_t*cosx;

    float Cd = (pow(abs(alpha),3.7)/125 + alpha)*3/3625 + Cd_o;
    float Cl = 0.01*alpha + Cl_0;
    float Cl_rudder = 0.01*beta;
    //Cl = constrainf(Cl,-1.3,1.3);

    // absolute velocity
    float Vsqr = vel_estimate_North*vel_estimate_North + vel_estimate_East*vel_estimate_East + vel_estimate_Up*vel_estimate_Up;
    float dynamic_p = 0.5*air_density*Vsqr;
    float L =  dynamic_p*wing_area*Cl;
    float D = -dynamic_p*wing_area*Cd *0.5;
    float Side_force = dynamic_p*rudder_area*Cl_rudder;

    float sinA = sin_approx(alpha*toRad);
    float cosA = cos_approx(alpha*toRad);
    float cosB = cos_approx(beta*toRad);
    float sinB = sin_approx(beta*toRad);

    // rotate aero(or wind frame) to body frame
    float Fbx =  L*sinA + D*cosA*cosB + Side_force*cosA*sinB;
    float Fby =  Side_force*cosB - D*sinB;
    float Fbz =  L*cosA - D*cosB*sinA - Side_force*sinA*sinB;

    // rotate gravity to body frame
    float g_bx =  gravity*siny;
    float g_by = -gravity*cosy*sinx;
    float g_bz =  gravity*cosx*cosy;

    Fbx += mass*g_bx + thrust;
    Fby += mass*g_by;
    Fbz += mass*g_bz;

    float acc_bx = Fbx/mass;
    float acc_by = Fby/mass;
    float acc_bz = Fbz/mass;

    // rotate acc body  to inertial frame
    float accEx = acc_bx*cosy*cosz - acc_bz*(sinx*sinz + cosx*cosz*siny) - acc_by*(cosx*sinz - cosz*sinx*siny);
    float accEy = acc_by*(cosx*cosz + sinx*siny*sinz) + acc_bz*(cosz*sinx - cosx*siny*sinz) + acc_bx*cosy*sinz;
    float accEz = acc_bx*siny + acc_bz*cosx*cosy - acc_by*cosy*sinx;

    // zero acce z on ground
    if (accEz > 0 && isFlying == 0)
        isFlying = 1;
    else if (accEz < 0 && isFlying == 0)
        accEz = 0;

    pos_estimate_North += vel_estimate_North*Dt + 0.5*accEx*Dt*Dt;
    pos_estimate_East += vel_estimate_East*Dt + 0.5*accEy*Dt*Dt;
    pos_estimate_Up += vel_estimate_Up*Dt + 0.5*accEz*Dt*Dt;

    //arrow.lat= pos_estimate_North/eart_radius*toDeg + init_latitude;
    //arrow.lon = pos_estimate_East/eart_radius*toDeg + init_longitude;
    //arrow.alt = pos_estimate_Up + init_altitude;

    vel_estimate_North +=  accEx*Dt;
    vel_estimate_East +=  accEy*Dt;
    vel_estimate_Up +=  accEz*Dt;


}
*/
