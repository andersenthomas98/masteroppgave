/************************************************************************/
// File:            kalmanFilter.h                                      //
// Author:          Leithe                                              //
// Purpose:         kalman filter                             //
//                                                                      //
/************************************************************************/

#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <stdint.h>
#include "positionEstimate.h"
/*
typedef struct{
float x;
float y;
float heading;
float vel;
}state;
*/
void ekf_init(int16_t m, int16_t n);

int ekf_step(float* Z,float cosTheta,float sinTheta);

position_estimate_t ekf_GetState();

void ekf_setGyroVar(float value);

void ekf_setEncoderVar(float value);




#endif /*EXTENDED_KALMAN_FILTER_H*/