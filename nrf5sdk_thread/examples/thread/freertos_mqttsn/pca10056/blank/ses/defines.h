/************************************************************************/
// File:			defines.h
// Author:			Erlend Ese, NTNU Spring 2016
//					Remeasured by Stenset, spring 2020
/************************************************************************/

#ifndef DEFINES_H_
#define DEFINES_H_

#include "robot_config.h"



/************************************************************************/
/* Program settings                                                     */
#define PERIOD_MOTOR_MS         20
#define PERIOD_ESTIMATOR_MS     40
#define PERIOD_SENSORS_MS       200
#define moveStop                0
#define moveForward             1
#define moveBackward            2
#define moveClockwise           3
#define moveCounterClockwise    4
#define moveLeftForward         5
#define moveRightForward        6
#define moveLeftBackward        7
#define moveRightBackward       8


/****** Motor ******/
#define MOTOR_FORWARDS			1
#define MOTOR_BACKWARDS			-1


/************************************************************************/
/* Macros
*************************************************************************/
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
                                                               
#define DEG2RAD M_PI / 180.0
#define RAD2DEG 180.0 / M_PI


#endif /* DEFINES_H_ */