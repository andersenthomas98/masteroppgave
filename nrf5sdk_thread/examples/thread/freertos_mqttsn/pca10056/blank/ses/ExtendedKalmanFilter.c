/************************************************************************/
// File:            kalmanFilter.h                                      //
// Author:          Leithe                                              //
// Purpose:         Kalman filter                                       //
//                                                                      //
/************************************************************************/

#include "ExtendedKalmanFilter.h"

#include "positionEstimate.h"

#include "matrix_operations.h"

#include "FreeRTOS.h"

#include "nrf_log.h"

#define N_constant 5
#define M_constant 4
#define p 0.04 //Period of estimator

static float X[N_constant]; //states in order x_pos, x_speed, x_acc, y_pos, y_speed, y_acc, theta, theta_hat
static float P[N_constant * N_constant];
static float Q[N_constant * N_constant];
static float R[M_constant * M_constant];
static float H[M_constant * N_constant];
static float Phi[N_constant * N_constant];

static float TEMP_HPH[M_constant * M_constant];
static float TEMP_INVERSE[M_constant * M_constant];
static float temp0[N_constant * N_constant];
static float temp1[N_constant * N_constant];
static float temp2[N_constant * M_constant];
static float temp3[N_constant * M_constant];
static float temp4[M_constant];
static float temp5[N_constant];
static int N;
static int M;
/*Set variance of IMU depending on which robot is used, this is NRF2*/

static float var_gyr_z = 0.0034;
//static float var_acc_x = 0.0000476; // Not used
//static float var_acc_y = 0.000044; // Not used

//functions to set value of row m column n, 0 indexed
void ekf_setH(int16_t m, int16_t n, float value) {
  H[N * m + n] = value;
}

void ekf_setPhi(int16_t m, int16_t n, float value) {
  Phi[N * m + n] = value;
}

void ekf_setQ(int16_t m, int16_t n, float value) {
  Q[N * m + n] = value;
}

void ekf_setR(int16_t m, int16_t n, float value) {
  R[M * m + n] = value;
}

void ekf_setEncoderVar(float value) {
  ekf_setR(2, 2, value);
}
void ekf_setGyroVar(float value) {
  ekf_setR(3, 3, value);
}

//Not usable
void ekf_setPeriod(float value) {
  if (value > 1 || value < 0) {
    return;
  } //dont allow periods over 1 or negative
  ekf_setPhi(0, 1, value);
  ekf_setPhi(1, 2, value);
  ekf_setPhi(3, 4, value);
  ekf_setPhi(4, 5, value);
}

void ekf_init(int16_t m, int16_t n) {
  //allocate data structures
  N = n;
  M = m;
  //0 out data structures
  zeros(X, N, 1);
  zeros(P, N, N);
  zeros(Q, N, N);
  zeros(R, M, M);
  zeros(H, M, N);
  zeros(Phi, N, N);
  //set initial P
  mat_addeye(P, N);

  //set Q
  //Temporary test values (Process noise)
  ekf_setQ(0, 0, 0.0001);
  ekf_setQ(1, 1, 0.0001);
  ekf_setQ(2, 2, 0.0001); //0.05
  ekf_setQ(3, 3, 0.01);
  ekf_setQ(4, 4, 0.01);
  //set R

  ekf_setR(0, 0, 0.02); //var encoder fwd
  ekf_setR(1, 1, 0.11); //var accel (was 0.03)
  ekf_setR(2, 2, 0.1); //var encoderTurn
  ekf_setR(3, 3, var_gyr_z); //var gyro	(was 0.0134)

  //set H

  ekf_setH(0, 3, 1);
  ekf_setH(1, 3, 1);
  ekf_setH(2, 4, 1);
  ekf_setH(3, 4, 1);

  //set Phi

  mat_addeye(Phi, N);
}

int ekf_step(float * Z, float cosTheta, float sinTheta) {

  //X_prior= Phi*x_prior_-1
  /*This updates the a priori estimate as if the motion model is linear (Normal KF filter), it is not. 
  The motion model is given as:
  x_t = x_t_-1 + delta(d)(distance traveled between t-1 and t)*cos(theta_t_-1)
  y_t = y_t_-1 + delta(d)*sin(theta_t_-1)
  theta_t = theta_t_-1 + delta(theta)(change in heading between t-1 and t) 
  Can be linearized in each step using a First order Taylor Expansion
  */
  X[0] += X[3] * p * cosTheta;
  X[1] += X[3] * p * sinTheta;
  X[2] += X[4] * p;
  //NRF_LOG_INFO("X_apri: " NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(X[0]));
  //NRF_LOG_INFO("Y_apri: " NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(X[1]));
  //NRF_LOG_INFO("Theta_apri: " NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(X[2]));
  /*
  printf("\nX[0] = %d",(int)X[0]);
  printf("\nX[1] = %d",(int)X[1]);
  printf("\nX[2] = %d",(int)X[2]);
  printf("\nX[3] = %d",(int)X[3]);
  printf("\nX[4] = %d",(int)X[4]);
  
  printf("\n---------------");
  printf("\nZ[0] = %d",(int)Z[0]);
  printf("\nZ[1] = %d",(int)Z[1]);
  printf("\nZ[2] = %d",(int)Z[2]);
  printf("\nZ[3] = %d",(int)Z[3]*1000);
  */

  /*
  mulmat(Phi, X, temp5, N, N, 1); //result N*1
  zeros(X,N,1);
  accum(X,temp5,N,1);  //move result to X
  */
  ekf_setPhi(0, 2, -X[3] * p * sinTheta);
  ekf_setPhi(0, 3, p * cosTheta);
  ekf_setPhi(1, 2, X[3] * p * cosTheta);
  ekf_setPhi(1, 3, p * sinTheta);
  ekf_setPhi(2, 4, p);

  //P_P = Phi*P_P_-1*transpose(Phi)+Q
  mulmat(Phi, P, temp1, N, N, N); //result N*N
  transpose(Phi, temp0, N, N); //Result N*N
  mulmat(temp1, temp0, P, N, N, N); //result N*N
  accum(P, Q, N, N);

  //K =P_prior*transpose(H)*inv(H*p_prior*transpose(H)+R)
  transpose(H, temp2, M, N); //result N*M

  mulmat(H, P, temp3, M, N, N); //result M*N
  mulmat(temp3, temp2, TEMP_HPH, M, N, M); //result M*M
  accum(TEMP_HPH, R, M, M);
  mulmat(P, temp2, temp3, N, N, M); //result N*M
  //TAKE INVERSE
  if (cholsl(TEMP_HPH, TEMP_INVERSE, temp5, M)) {
    return 0;
  }
  mulmat(temp3, TEMP_INVERSE, temp2, N, M, M); //result N*M

  //X_estimate = X_p + K(Z-H*X_p)
  mulmat(H, X, temp4, M, N, 1); //result m*1
  sub(Z, temp4, temp4, M); //result M*1
  mulmat(temp2, temp4, temp5, N, M, 1); //result N*1
  accum(X, temp5, N, 1);

  //P= (I-K*H)*P_p
  mulmat(temp2, H, temp1, N, M, N); //result N*N
  negate(temp1, N, N);
  mat_addeye(temp1, N);
  mulmat(temp1, P, temp0, N, N, N); //result N*N
  zeros(P, N, N);
  accum(P, temp0, N, N); //move result to P

  return 1;
}

position_estimate_t ekf_GetState() {
  position_estimate_t result;
  result.x = X[0];
  result.y = X[1];
  result.heading = X[2];
  result.vel = X[3];
  return result;
}