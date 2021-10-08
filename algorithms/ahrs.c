
#include "ahrs.h"

// Vector to hold quaternion
//static float q[8] = {1.0f, 0.0f, 0.0f, 0.0f,1.0f, 0.0f, 0.0f, 0.0f};
//
//================EKF==============
// gravity vector normalized
float g  =  1; //-9.81;
//  Magnetic Vector at location (35.235118 N,129.082762 E, 30m)
// North comp (30,700.0nT), East comp (-4,403.8nT), Vertical Comp ,+D | -U, (38,500.3nT)
const float HX = 0.6210; //307.0;
const float HY = -0.089; //-44.03;
const float HZ = 0.7788; // 385.0; // Magnetic reading in z-direction is constant with value of 385.
// Process noise standard deviation
float sigma_g=0.0007;
// Measurement noise standard deviation for the accelerometer
float sigma_a=0.04;
// Measurement noise standard deviation for the magnetometer
float sigma_m=0.004;
int firstime =1;
//
static void init(struct EKF_Q * ekf)
{
    int i, j;
    int n = Nsta;
    int m = Mobs;
    // Initial states
    // quaternion
    ekf->x[0] = 1.0;
    ekf->x[1] = 0.0;
    ekf->x[2] = 0.0;
    ekf->x[3] = 0.0;
    // quaternion 2
    ekf->x[4] = 1.0;
    ekf->x[5] = 0.0;
    ekf->x[6] = 0.0;
    ekf->x[7] = 0.0;
    // initial covariances of state noise, measurement noise
    // Initial error Covariance P
    for (i =0;i<n;i++)
    {
        for(j=0;j<n;j++)
        {
            ekf->P[i][j] =0.0;
        }
    }
    float R0 = 50;
    for (i=0; i<m-3; ++i)
    {
        ekf->R[i][i] = R0;
    }
    ekf->R[6][6]=500;
    ekf->R[7][7]=500;
    ekf->R[8][8]=500;
}
//system model
static void model(struct EKF_Q * ekf, float omega[3],float T)
{
     int k,  w;
     int n =Nsta;
// determine State transition matrix    Fk
     ekf->F[0][0] = 1.0;
     ekf->F[0][1] = (omega[0] * -1)*0.5*T;
     ekf->F[0][2] = (omega[1]*-1)*0.5*T;
     ekf->F[0][3] = (omega[2]* -1)*0.5*T;
     ekf->F[0][4] = 0.0;
     ekf->F[0][5] = 0.0;
     ekf->F[0][6] = 0.0;
     ekf->F[0][7] = 0.0;

     ekf->F[1][0] = omega[0]*0.5*T;
     ekf->F[1][1] = 1.0;
     ekf->F[1][2] = omega[2]*0.5*T;
     ekf->F[1][3] = omega[1]*-1*0.5*T;
     ekf->F[1][4] = 0.0;
     ekf->F[1][5] = 0.0;
     ekf->F[1][6] = 0.0;
     ekf->F[1][7] = 0.0;

     ekf->F[2][0] = omega[1]*0.5*T;
     ekf->F[2][1] = (omega[2]* -1)*0.5*T;
     ekf->F[2][2] = 1.0;
     ekf->F[2][3] = omega[0]*0.5*T;
     ekf->F[2][4] = 0.0;
     ekf->F[2][5] = 0.0;
     ekf->F[2][6] = 0.0;
     ekf->F[2][7] = 0.0;

     ekf->F[3][0] = omega[2]*0.5*T;
     ekf->F[3][1] = omega[1]*0.5*T;
     ekf->F[3][2] = (omega[0] * -1)*0.5*T;
     ekf->F[3][3] = 1.0;
     ekf->F[3][4] = 0.0;
     ekf->F[3][5] = 0.0;
     ekf->F[3][6] = 0.0;
     ekf->F[3][7] = 0.0;

     ekf->F[4][0] = 0.0;
     ekf->F[4][1] = 0.0;
     ekf->F[4][2] = 0.0;
     ekf->F[4][3] = 0.0;
     ekf->F[4][4] = 1.0;
     ekf->F[4][5] = (omega[0] * -1)*0.5*T;
     ekf->F[4][6] = (omega[1]*-1)*0.5*T;
     ekf->F[4][7] = (omega[2]* -1)*0.5*T;

     ekf->F[5][0] = 0.0;
     ekf->F[5][1] = 0.0;
     ekf->F[5][2] = 0.0;
     ekf->F[5][3] = 0.0;
     ekf->F[5][4] = omega[0]*0.5*T;
     ekf->F[5][5] = 1.0;
     ekf->F[5][6] = omega[2]*0.5*T;
     ekf->F[5][7] = (omega[1]*-1)*0.5*T;

     ekf->F[6][0] = 0.0;
     ekf->F[6][1] = 0.0;
     ekf->F[6][2] = 0.0;
     ekf->F[6][3] = 0.0;
     ekf->F[6][4] = omega[1]*0.5*T;
     ekf->F[6][5] = (omega[2]* -1)*0.5*T;
     ekf->F[6][6] = 1.0;
     ekf->F[6][7] = omega[0]*0.5*T;

     ekf->F[7][0] = 0.0;
     ekf->F[7][1] = 0.0;
     ekf->F[7][2] = 0.0;
     ekf->F[7][3] = 0.0;
     ekf->F[7][4] = omega[2]*0.5*T;
     ekf->F[7][5] = omega[1]*0.5*T;
     ekf->F[7][6] = (omega[0] * -1)*0.5*T;
     ekf->F[7][7] = 1.0;

     // predict current state  fk(x)
     ekf->fx[0] = ekf->F[0][0] * ekf->x[0] + ekf->F[0][1] * ekf->x[1]
             + ekf->F[0][2] * ekf->x[2] + ekf->F[0][3] * ekf->x[3];
     ekf->fx[1] = ekf->F[1][0] * ekf->x[0] + ekf->F[1][1] * ekf->x[1]
             + ekf->F[1][2] * ekf->x[2] + ekf->F[1][3] * ekf->x[3];
     ekf->fx[2] = ekf->F[2][0] * ekf->x[0] + ekf->F[2][1] * ekf->x[1]
             + ekf->F[2][2] * ekf->x[2] + ekf->F[2][3] * ekf->x[3];
     ekf->fx[3] = ekf->F[3][0] * ekf->x[0] + ekf->F[3][1] * ekf->x[1]
             + ekf->F[3][2] * ekf->x[2] + ekf->F[3][3] * ekf->x[3];

     ekf->fx[4] = ekf->F[4][4] * ekf->x[4] + ekf->F[4][5] * ekf->x[5]
             + ekf->F[4][6] * ekf->x[6] + ekf->F[4][7] * ekf->x[7];
     ekf->fx[5] = ekf->F[5][4] * ekf->x[4] + ekf->F[5][5] * ekf->x[5]
             + ekf->F[5][6] * ekf->x[6] + ekf->F[5][7] * ekf->x[7];
     ekf->fx[6] = ekf->F[6][4] * ekf->x[4] + ekf->F[6][5] * ekf->x[5]
             + ekf->F[6][6] * ekf->x[6] + ekf->F[6][7] * ekf->x[7];
     ekf->fx[7] = ekf->F[7][4] * ekf->x[4] + ekf->F[7][5] * ekf->x[5]
             + ekf->F[7][6] * ekf->x[6] + ekf->F[7][7] * ekf->x[7];
    // observation prediction hk(x)
       ekf->hx[0]= 2*(f1*f3-f0*f2)*g;
       ekf->hx[1]= 2*(f2*f3+f0*f1)*g;
       ekf->hx[2]= (f0*f0-f1*f1-f2*f2+f3*f3)*g;
       //
       ekf->hx[3]= 2*(f5*f7-f4*f6)*g;
       ekf->hx[4]= 2*(f6*f7+f4*f5)*g;
       ekf->hx[5]= (f4*f4-f5*f5-f6*f6+f7*f7)*g;
       //
       ekf->hx[6]= (f4*f4+f5*f5-f6*f6-f7*f7)*HX + 2*(f5*f6 +f4*f7)*HY +2*(f5*f7-f4*f6)*HZ ;
       ekf->hx[7]= 2*(f5*f6 -f4*f7)*HX +(f4*f4 -f5*f5+f6*f6 -f7*f7)*HY +2*(f6*f7 +f4*f5)*HZ ;
       ekf->hx[8]= 2*(f4*f6 +f5*f7)*HX + 2*(f6*f7-f4*f5)*HY + (f4*f4 -f5*f5 -f6*f6 +f7*f7)*HZ ;
     // Observation matrix  Hk
       // column 1~4
       ekf->H[0][0] =-2.0*g*f2;    ekf->H[0][1] = 2.0*g*f3 ;    ekf->H[0][2] =-2.0*g*f0;   ekf->H[0][3] = 2.0*g*f1;
       ekf->H[1][0] = 2.0*g*f1;    ekf->H[1][1] = 2.0*g*f0 ;    ekf->H[1][2] = 2.0*g*f3;   ekf->H[1][3] = 2.0*g*f2;
       ekf->H[2][0] = 2.0*g*f0;    ekf->H[2][1] =-2.0*g*f1 ;    ekf->H[2][2] =-2.0*g*f2;   ekf->H[2][3] = 2.0*g*f3;
       ekf->H[3][0] = 0.0;         ekf->H[3][1] = 0.0 ;         ekf->H[3][2] =0.0;         ekf->H[3][3] = 0.0;
       ekf->H[4][0] = 0.0;         ekf->H[4][1] = 0.0 ;         ekf->H[4][2] =0.0;         ekf->H[4][3] = 0.0;
       ekf->H[5][0] = 0.0;         ekf->H[5][1] = 0.0 ;         ekf->H[5][2] =0.0;         ekf->H[5][3] = 0.0;
       ekf->H[6][0] = 0.0;         ekf->H[6][1] = 0.0 ;         ekf->H[6][2] =0.0;         ekf->H[6][3] = 0.0;
       ekf->H[7][0] = 0.0;         ekf->H[7][1] = 0.0 ;         ekf->H[7][2] =0.0;         ekf->H[7][3] = 0.0;
       ekf->H[8][0] = 0.0;         ekf->H[8][1] = 0.0 ;         ekf->H[8][2] =0.0;         ekf->H[8][3] = 0.0;
       // column 5 ~ 8
       ekf->H[0][4] = 0.0;                        ekf->H[0][5] = 0.0 ;                    ekf->H[0][6] =0.0;                      ekf->H[0][7] = 0.0;
       ekf->H[1][4] = 0.0;                        ekf->H[1][5] = 0.0 ;                    ekf->H[1][6] =0.0;                      ekf->H[1][7] = 0.0;
       ekf->H[2][4] = 0.0;                        ekf->H[2][5] = 0.0 ;                    ekf->H[2][6] =0.0;                      ekf->H[2][7] = 0.0;
       ekf->H[3][4] =-2.0*g*f6;                   ekf->H[3][5] = 2.0*g*f7 ;               ekf->H[3][6] =-2.0*g*f4;                ekf->H[3][7] = 2.0*g*f5;
       ekf->H[4][4] = 2.0*g*f5;                   ekf->H[4][5] = 2.0*g*f4 ;               ekf->H[4][6] = 2.0*g*f7;                ekf->H[4][7] = 2.0*g*f6;
       ekf->H[5][4] = 2.0*g*f4;                   ekf->H[5][5] =-2.0*g*f5 ;               ekf->H[5][6] =-2.0*g*f6;                ekf->H[5][7] = 2.0*g*f7;
       ekf->H[6][4] = 2.0*(HX*f4 +HY*f7 -HZ*f6) ; ekf->H[6][5] = 2.0*(HX*f5+HY*f6+HZ*f7); ekf->H[6][6] = 2.0*(HY*f5-HX*f6-HZ*f4); ekf->H[6][7] = 2.0*(-HX*f7+HY*f4+HZ*f5);
       ekf->H[7][4] = 2.0*(HY*f4-HX*f7 + HZ*f5) ; ekf->H[7][5] = 2.0*(HX*f6-HY*f5+HZ*f4); ekf->H[7][6] = 2.0*(HX*f5+HY*f6+HZ*f7); ekf->H[7][7] = 2.0*(HZ*f6-HY*f7-HX*f4);
       ekf->H[8][4] = 2.0*(HX*f6-HY*f5 + HZ*f4) ; ekf->H[8][5] = 2.0*(HX*f7-HY*f4-HZ*f5); ekf->H[8][6] = 2.0*(HX*f4+HY*f7-HZ*f6); ekf->H[8][7] = 2.0*(HX*f5+HY*f6+HZ*f7);
       // Process noise covariance Qk
       for (k=0;k<n;k++)
       {
           for(w=0;w<n;w++)
           {
               if(k==w)
               {
                   ekf->Q[k][w]=100;
               }
               else
               {
                   ekf->Q[k][w]=  0.0;
               }
           }
       }
}
void EKFQ(float ax, float ay, float az, float gx, float gy,float gz, float mx, float my, float mz, float deltat)
{
    int N =Nsta;
    int M = Mobs;
    float norm, norm_;
    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;
    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;
    // Do generic EKF initialization
    //ekf_t ekf;
    if(firstime)
    {
    ekf_init(&ekf_t, N, M);

    // Do local initialization
    init(&ekf_t);
    firstime =0;
    }
    // Make a place to store the data from the file and the output of the EKF
    float omega[3] = {gx,gy,gz}; //{0.0,0.0,0.0};//
    float acclmag[9] = {ax,ay,az,ax,ay,az,mx,my,mz};  //{0.0,0.0,0.0,0.0,0.0,0.0}; //
    float q1, q2, q3, q4, q5, q6, q7, q8 ;
    // compute Fk, fk(x), hk(x), Hk & Qk
    model(&ekf_t,omega,deltat);
    // EKF update
    ekf_step(&ekf_t,acclmag);
    // normalize quaternion
    q1 =ekf_t.x[0];
    q2 =ekf_t.x[1];
    q3 =ekf_t.x[2];
    q4 =ekf_t.x[3];
    q5 =ekf_t.x[4];
    q6 =ekf_t.x[5];
    q7 =ekf_t.x[6];
    q8 =ekf_t.x[7];
    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm_ = sqrt(q5 * q5 + q6 * q6 + q7 * q7 + q8 * q8);
    if (norm == 0.0f || norm_ == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;
    norm_ = 1.0f / norm_;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
    q[4] = q5 * norm_;
    q[5] = q6 * norm_;
    q[6] = q7 * norm_;
    q[7] = q8 * norm_;
    //
    ekf_t.x[0] =q[0];
    ekf_t.x[1] =q[1];
    ekf_t.x[2] =q[2];
    ekf_t.x[3] =q[3];
    ekf_t.x[4] =q[4];
    ekf_t.x[5] =q[5];
    ekf_t.x[6] =q[6];
    ekf_t.x[7] =q[7];
}

const float * getQ () { return q; }
