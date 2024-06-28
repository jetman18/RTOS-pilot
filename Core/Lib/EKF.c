#include "EKF.h"

#define toRAD 0.0174f

// using Extended kalman filter to estimate 4 states q0 q1 q2 q3
static float q0 = 1.0f, q1, q2, q3;
const float Dt = 0.01f;
float gyro_LBS = 32.8f;

// Gyro variance 
float Q_gyro = 0.01;

// Covarian
const float P_init = 1.0f;
float P[4][4];

// Esitmate vector
float acc[3] = {0,0,1};
float mag[3] = {0,0,1};


/*
 *  Update measurement variance from sensor
 */
void EKF_measurementVarianceUpdate(){

}



void EKF_reset(){
    // reset P 
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            if(i == j){
                P[i][j] = P_init;
            }else
                P[i][j] = 0.0f;
        }
    }
    // reset Q
    q0 = 1.0f;
    q1 = 0;
    q2 = 0;
    q3 = 0;
}

/*
 * Extended kalman filter update
 */
void EKF_update(int16_t gx, int16_t gy, int16_t gz, 
        int16_t accx, int16_t accy, int16_t accz, 
        int16_t mx, int16_t my, int16_t mz)
{
/**********  intput calibrate ************/
    // gyro to rad/s
    float wx = gx / gyro_LBS  * toRAD;
    float wy = gy / gyro_LBS  * toRAD;
    float wz = gz / gyro_LBS  * toRAD;

    /***************** Estimate step ******************************************/
    float q0_ = 2*q0/Dt - wx*q1 - wy*q2 - wz*q3;
    float q1_ = 2*q1/Dt + wx*q0 - wy*q3 - wz*q2;
    float q2_ = 2*q2/Dt + wx*q3 + wy*q0 - wz*q1;
    float q3_ = 2*q3/Dt - wx*q2 + wy*q1 - wz*q0;

    q0 = q0_ * Dt / 2.0f;
    q1 = q1_ * Dt / 2.0f;
    q2 = q2_ * Dt / 2.0f;
    q3 = q3_ * Dt / 2.0f;


    float DtDt = Dt*Dt;
    float q0q0 = q0*q0;
    float q1q1 = q1*q1;
    float q2q2 = q2*q2;
    float q3q3 = q3*q3;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q2q3 = q2*q3;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;

    // Proccess variance
    float Q00 =  (Q_gyro*DtDt*q1q1 + Q_gyro*DtDt*q2q2 + Q_gyro*DtDt*q3q3)/4;
    float Q01 = -DtDt*Q_gyro*q0q1/4;
    float Q02 = -(DtDt*Q_gyro*q0q2 - DtDt*Q_gyro*q1*q3 - DtDt*Q_gyro*q2q3)/4;
    float Q03 = -(DtDt*Q_gyro*q0q3)/4;

    float Q10 = -(DtDt*Q_gyro*q0q1)/4;
    float Q11 =  (Q_gyro*DtDt*q0q0 + Q_gyro*DtDt*q2q2 + Q_gyro*DtDt*q3q3)/4;
    float Q12 =  (DtDt*Q_gyro*q2q2)/4;
    float Q13 = -(DtDt*Q_gyro*q1*q3)/4;

    float Q20 = -(DtDt*Q_gyro*q0q2 - DtDt*Q_gyro*q1*q3 - DtDt*Q_gyro*q2q3)/4;
    float Q21 =  (DtDt*Q_gyro*q2q2)/4;
    float Q22 =  (Q_gyro*DtDt*q0q0 + Q_gyro*DtDt*q2q2 + Q_gyro*DtDt*q3q3)/4;
    float Q23 =  (DtDt*Q_gyro*q0q1 + DtDt*Q_gyro*q0q2 - DtDt*Q_gyro*q2q3)/4;

    float Q30 = -(DtDt*Q_gyro*q0q3)/4;
    float Q31 = -(DtDt*Q_gyro*q1*q3)/4;
    float Q32 =  (DtDt*Q_gyro*q0q1 + DtDt*Q_gyro*q0q2 - DtDt*Q_gyro*q2q3)/4;
    float Q33 =  (Q_gyro*DtDt*q0q0 + Q_gyro*DtDt*q1q1 + Q_gyro*DtDt*q2q2)/4;
    
    // proccess covarian
    P[0][0] = P[0][0] - (Dt*P[1][0]*wx)/2 - (Dt*P[2][0]*wy)/2 - (Dt*P[3][0]*wz)/2 + (Dt*wx*((Dt*P[1][1]*wx)/2 - P[0][1] + (Dt*P[2][1]*wy)/2 + (Dt*P[3][1]*wz)/2))/2 + (Dt*wy*((Dt*P[1][2]*wx)/2 - P[0][2] + (Dt*P[2][2]*wy)/2 + (Dt*P[3][2]*wz)/2))/2 + (Dt*wz*((Dt*P[1][3]*wx)/2 - P[0][3] + (Dt*P[2][3]*wy)/2 + (Dt*P[3][3]*wz)/2))/2 + Q00;
    P[0][1] = P[0][1] - (Dt*P[1][1]*wx)/2 - (Dt*P[2][1]*wy)/2 - (Dt*P[3][1]*wz)/2 - (Dt*wx*((Dt*P[1][0]*wx)/2 - P[0][0] + (Dt*P[2][0]*wy)/2 + (Dt*P[3][0]*wz)/2))/2 + (Dt*wy*((Dt*P[1][3]*wx)/2 - P[0][3] + (Dt*P[2][3]*wy)/2 + (Dt*P[3][3]*wz)/2))/2 - (Dt*wz*((Dt*P[1][2]*wx)/2 - P[0][2] + (Dt*P[2][2]*wy)/2 + (Dt*P[3][2]*wz)/2))/2 + Q01;
    P[0][2] = P[0][2] - (Dt*P[1][2]*wx)/2 - (Dt*P[2][2]*wy)/2 - (Dt*P[3][2]*wz)/2 - (Dt*wx*((Dt*P[1][3]*wx)/2 - P[0][3] + (Dt*P[2][3]*wy)/2 + (Dt*P[3][3]*wz)/2))/2 - (Dt*wy*((Dt*P[1][0]*wx)/2 - P[0][0] + (Dt*P[2][0]*wy)/2 + (Dt*P[3][0]*wz)/2))/2 + (Dt*wz*((Dt*P[1][1]*wx)/2 - P[0][1] + (Dt*P[2][1]*wy)/2 + (Dt*P[3][1]*wz)/2))/2 + Q02;
    P[0][3] = P[0][3] - (Dt*P[1][3]*wx)/2 - (Dt*P[2][3]*wy)/2 - (Dt*P[3][3]*wz)/2 + (Dt*wx*((Dt*P[1][2]*wx)/2 - P[0][2] + (Dt*P[2][2]*wy)/2 + (Dt*P[3][2]*wz)/2))/2 - (Dt*wy*((Dt*P[1][1]*wx)/2 - P[0][1] + (Dt*P[2][1]*wy)/2 + (Dt*P[3][1]*wz)/2))/2 - (Dt*wz*((Dt*P[1][0]*wx)/2 - P[0][0] + (Dt*P[2][0]*wy)/2 + (Dt*P[3][0]*wz)/2))/2 + Q03;

    P[1][0] = P[1][0] + (Dt*P[0][0]*wx)/2 - (Dt*P[3][0]*wy)/2 + (Dt*P[2][0]*wz)/2 - (Dt*wx*(P[1][1] + (Dt*P[0][1]*wx)/2 - (Dt*P[3][1]*wy)/2 + (Dt*P[2][1]*wz)/2))/2 - (Dt*wy*(P[1][2] + (Dt*P[0][2]*wx)/2 - (Dt*P[3][2]*wy)/2 + (Dt*P[2][2]*wz)/2))/2 - (Dt*wz*(P[1][3] + (Dt*P[0][3]*wx)/2 - (Dt*P[3][3]*wy)/2 + (Dt*P[2][3]*wz)/2))/2 + Q10;
    P[1][1] = P[1][1] + (Dt*P[0][1]*wx)/2 - (Dt*P[3][1]*wy)/2 + (Dt*P[2][1]*wz)/2 + (Dt*wx*(P[1][0] + (Dt*P[0][0]*wx)/2 - (Dt*P[3][0]*wy)/2 + (Dt*P[2][0]*wz)/2))/2 - (Dt*wy*(P[1][3] + (Dt*P[0][3]*wx)/2 - (Dt*P[3][3]*wy)/2 + (Dt*P[2][3]*wz)/2))/2 + (Dt*wz*(P[1][2] + (Dt*P[0][2]*wx)/2 - (Dt*P[3][2]*wy)/2 + (Dt*P[2][2]*wz)/2))/2 + Q11;
    P[1][2] = P[1][2] + (Dt*P[0][2]*wx)/2 - (Dt*P[3][2]*wy)/2 + (Dt*P[2][2]*wz)/2 + (Dt*wx*(P[1][3] + (Dt*P[0][3]*wx)/2 - (Dt*P[3][3]*wy)/2 + (Dt*P[2][3]*wz)/2))/2 + (Dt*wy*(P[1][0] + (Dt*P[0][0]*wx)/2 - (Dt*P[3][0]*wy)/2 + (Dt*P[2][0]*wz)/2))/2 - (Dt*wz*(P[1][1] + (Dt*P[0][1]*wx)/2 - (Dt*P[3][1]*wy)/2 + (Dt*P[2][1]*wz)/2))/2 + Q12;
    P[1][3] = P[1][3] + (Dt*P[0][3]*wx)/2 - (Dt*P[3][3]*wy)/2 + (Dt*P[2][3]*wz)/2 - (Dt*wx*(P[1][2] + (Dt*P[0][2]*wx)/2 - (Dt*P[3][2]*wy)/2 + (Dt*P[2][2]*wz)/2))/2 + (Dt*wy*(P[1][1] + (Dt*P[0][1]*wx)/2 - (Dt*P[3][1]*wy)/2 + (Dt*P[2][1]*wz)/2))/2 + (Dt*wz*(P[1][0] + (Dt*P[0][0]*wx)/2 - (Dt*P[3][0]*wy)/2 + (Dt*P[2][0]*wz)/2))/2 + Q13;

    P[2][0] = P[2][0] + (Dt*P[3][0]*wx)/2 + (Dt*P[0][0]*wy)/2 - (Dt*P[1][0]*wz)/2 - (Dt*wx*(P[2][1] + (Dt*P[3][1]*wx)/2 + (Dt*P[0][1]*wy)/2 - (Dt*P[1][1]*wz)/2))/2 - (Dt*wy*(P[2][2] + (Dt*P[3][2]*wx)/2 + (Dt*P[0][2]*wy)/2 - (Dt*P[1][2]*wz)/2))/2 - (Dt*wz*(P[2][3] + (Dt*P[3][3]*wx)/2 + (Dt*P[0][3]*wy)/2 - (Dt*P[1][3]*wz)/2))/2 + Q20;
    P[2][1] = P[2][1] + (Dt*P[3][1]*wx)/2 + (Dt*P[0][1]*wy)/2 - (Dt*P[1][1]*wz)/2 + (Dt*wx*(P[2][0] + (Dt*P[3][0]*wx)/2 + (Dt*P[0][0]*wy)/2 - (Dt*P[1][0]*wz)/2))/2 - (Dt*wy*(P[2][3] + (Dt*P[3][3]*wx)/2 + (Dt*P[0][3]*wy)/2 - (Dt*P[1][3]*wz)/2))/2 + (Dt*wz*(P[2][2] + (Dt*P[3][2]*wx)/2 + (Dt*P[0][2]*wy)/2 - (Dt*P[1][2]*wz)/2))/2 + Q21;
    P[2][2] = P[2][2] + (Dt*P[3][2]*wx)/2 + (Dt*P[0][2]*wy)/2 - (Dt*P[1][2]*wz)/2 + (Dt*wx*(P[2][3] + (Dt*P[3][3]*wx)/2 + (Dt*P[0][3]*wy)/2 - (Dt*P[1][3]*wz)/2))/2 + (Dt*wy*(P[2][0] + (Dt*P[3][0]*wx)/2 + (Dt*P[0][0]*wy)/2 - (Dt*P[1][0]*wz)/2))/2 - (Dt*wz*(P[2][1] + (Dt*P[3][1]*wx)/2 + (Dt*P[0][1]*wy)/2 - (Dt*P[1][1]*wz)/2))/2 + Q22;
    P[2][3] = P[2][3] + (Dt*P[3][3]*wx)/2 + (Dt*P[0][3]*wy)/2 - (Dt*P[1][3]*wz)/2 - (Dt*wx*(P[2][2] + (Dt*P[3][2]*wx)/2 + (Dt*P[0][2]*wy)/2 - (Dt*P[1][2]*wz)/2))/2 + (Dt*wy*(P[2][1] + (Dt*P[3][1]*wx)/2 + (Dt*P[0][1]*wy)/2 - (Dt*P[1][1]*wz)/2))/2 + (Dt*wz*(P[2][0] + (Dt*P[3][0]*wx)/2 + (Dt*P[0][0]*wy)/2 - (Dt*P[1][0]*wz)/2))/2 + Q23;

    P[3][0] = P[3][0] - (Dt*P[2][0]*wx)/2 + (Dt*P[1][0]*wy)/2 + (Dt*P[0][0]*wz)/2 - (Dt*wx*(P[3][1] - (Dt*P[2][1]*wx)/2 + (Dt*P[1][1]*wy)/2 + (Dt*P[0][1]*wz)/2))/2 - (Dt*wy*(P[3][2] - (Dt*P[2][2]*wx)/2 + (Dt*P[1][2]*wy)/2 + (Dt*P[0][2]*wz)/2))/2 - (Dt*wz*(P[3][3] - (Dt*P[2][3]*wx)/2 + (Dt*P[1][3]*wy)/2 + (Dt*P[0][3]*wz)/2))/2 + Q30;
    P[3][1] = P[3][1] - (Dt*P[2][1]*wx)/2 + (Dt*P[1][1]*wy)/2 + (Dt*P[0][1]*wz)/2 + (Dt*wx*(P[3][0] - (Dt*P[2][0]*wx)/2 + (Dt*P[1][0]*wy)/2 + (Dt*P[0][0]*wz)/2))/2 - (Dt*wy*(P[3][3] - (Dt*P[2][3]*wx)/2 + (Dt*P[1][3]*wy)/2 + (Dt*P[0][3]*wz)/2))/2 + (Dt*wz*(P[3][2] - (Dt*P[2][2]*wx)/2 + (Dt*P[1][2]*wy)/2 + (Dt*P[0][2]*wz)/2))/2 + Q31;
    P[3][2] = P[3][2] - (Dt*P[2][2]*wx)/2 + (Dt*P[1][2]*wy)/2 + (Dt*P[0][2]*wz)/2 + (Dt*wx*(P[3][3] - (Dt*P[2][3]*wx)/2 + (Dt*P[1][3]*wy)/2 + (Dt*P[0][3]*wz)/2))/2 + (Dt*wy*(P[3][0] - (Dt*P[2][0]*wx)/2 + (Dt*P[1][0]*wy)/2 + (Dt*P[0][0]*wz)/2))/2 - (Dt*wz*(P[3][1] - (Dt*P[2][1]*wx)/2 + (Dt*P[1][1]*wy)/2 + (Dt*P[0][1]*wz)/2))/2 + Q32;
    P[3][3] = P[3][3] - (Dt*P[2][3]*wx)/2 + (Dt*P[1][3]*wy)/2 + (Dt*P[0][3]*wz)/2 - (Dt*wx*(P[3][2] - (Dt*P[2][2]*wx)/2 + (Dt*P[1][2]*wy)/2 + (Dt*P[0][2]*wz)/2))/2 + (Dt*wy*(P[3][1] - (Dt*P[2][1]*wx)/2 + (Dt*P[1][1]*wy)/2 + (Dt*P[0][1]*wz)/2))/2 + (Dt*wz*(P[3][0] - (Dt*P[2][0]*wx)/2 + (Dt*P[1][0]*wy)/2 + (Dt*P[0][0]*wz)/2))/2 + Q33;

    /***************** Correction step ******************************************/
    //measurement variable accx, accy, accz mx,  my, mz
    // normalize acceleration
    uint32_t sum = accx*accx + accy*accy + accz*accz;
    float norm = sqrtf(sum);
    float acc_x = accx/norm;
    float acc_y = accy/norm;
    float acc_z = accz/norm;

    // normalize magnetic
    sum = mx*mx + my*my + mz*mz;
    norm = sqrtf(sum);
    float m_x = mz/norm;
    float m_y = my/norm;
    float m_z = mz/norm;
    
    // estimate acceleration and magnetic from quaternion
    acc[0] = 2*(acc[0]*(0.5 - q2q2 - q3q3) + acc[1]*(q0q3 + q1q2) + acc[2]*(q1q3 - q1q2));
    acc[1] = 2*(acc[0]*(q1q2 - q0q3) + acc[1]*(0.5 - q1q1 -  q3q3) + acc[2]*(q0q1 + q2q3)); 
    acc[2] = 2*(acc[0]*(q1q2 + q1q3) + acc[1]*(q2q3 - q0q1) + acc[2]*(0.5 - q1q1 -  q2q2)); 
    mag[0] = 2*(mag[0]*(0.5 - q2q2 - q3q3) + mag[1]*(q0q3 + q1q2) + mag[2]*(q1q3 - q1q2)); 
    mag[1] = 2*(mag[0]*(q1q2 - q0q3) + mag[1]*(0.5 - q1q1 -  q3q3) + mag[2]*(q0q1 + q2q3)); 
    mag[2] = 2*(mag[0]*(q1q2 + q1q3) + mag[1]*(q2q3 - q0q1) + mag[2]*(0.5 - q1q1 -  q2q2));
    
    float v1 = acc_x -  acc[0];
    float v2 = acc_y -  acc[1];
    float v3 = acc_z - acc[2];
    float v4 = m_x   - mag[0];
    float v5 = m_y   - mag[1];
    float v6 = m_z   - mag[2];




}

