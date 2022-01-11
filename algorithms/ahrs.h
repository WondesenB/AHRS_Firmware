#ifndef _AHRSALOGRITHMS_H_
#define _AHRSALOGRITHMS_H_
#include <math.h>
//===========================EKF========================
//#include "ekfq.h"
/* states */
#define Nsta 8
/* observables */
#define Mobs 9

#define  f0    ekf->fx[0]
#define  f1    ekf->fx[1]
#define  f2    ekf->fx[2]
#define  f3    ekf->fx[3]
#define  f4    ekf->fx[4]
#define  f5    ekf->fx[5]
#define  f6    ekf->fx[6]
#define  f7    ekf->fx[7]

static int cov_updated = 0;
 struct EKF_Q{

    int n;           /* number of state values */
    int m;           /* number of observables */

    float x[Nsta];     /* state vector */

    float P[Nsta][Nsta];  /* prediction error covariance */
    float Q[Nsta][Nsta];  /* process noise covariance */
    float R[Mobs][Mobs];  /* measurement error covariance */

    float G[Nsta][Mobs];  /* Kalman gain; a.k.a. K */

    float F[Nsta][Nsta];  /* Jacobian of process model */
    float H[Mobs][Nsta];  /* Jacobian of measurement model */

    float Ht[Nsta][Mobs]; /* transpose of measurement Jacobian */
    float Ft[Nsta][Nsta]; /* transpose of process Jacobian */
    float Pp[Nsta][Nsta]; /* P, post-prediction, pre-update */

    float fx[Nsta];   /* output of user defined f() state-transition function */
    float hx[Mobs];   /* output of user defined h() measurement function */

    /* temporary storage */
    float tmp0[Nsta][Nsta];
    float tmp1[Nsta][Mobs];
    float tmp2[Mobs][Nsta];
    float tmp3[Mobs][Mobs];
    float tmp4[Mobs][Mobs];
    float tmp5[Mobs];
};
static float q[8] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
extern struct EKF_Q ekf_t;
extern void ekf_init(void*, int, int);
extern int ekf_step(void*, float*);
void EKFQ(float ax, float ay, float az, float gx, float gy, float gz, float mx,
          float my, float mz, float deltat,float time);
void Covariance_update(struct EKF_Q * ekf,float R);
const float* getQ();
#endif // _AHRSALOGRITHMS_H_
