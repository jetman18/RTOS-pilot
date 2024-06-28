#ifndef _EKF_H_
#define _EKF_H_

#ifdef __cplusplus
extern "C" {
#endif

void EKF_reset();
void EKF_update(int16_t gx, int16_t gy, int16_t gz, 
        int16_t accx, int16_t accy, int16_t accz, 
        int16_t mx, int16_t my, int16_t mz);

#ifdef __cplusplus
}
#endif
#endif