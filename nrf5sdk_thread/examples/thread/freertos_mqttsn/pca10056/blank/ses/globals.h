/************************************************************************/
// File:            globals.h                                      //
// Author:                                                        //
// Purpose:         Organize all global stash and shared stuff          //
//                                                                      //
/************************************************************************/

#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Semaphore handles */
extern SemaphoreHandle_t xScanLock;
extern SemaphoreHandle_t xPoseMutex;
extern SemaphoreHandle_t xTickBSem;
extern SemaphoreHandle_t xControllerBSem;
extern SemaphoreHandle_t xCommandReadyBSem;
extern SemaphoreHandle_t mutex_spi;
extern SemaphoreHandle_t mutex_i2c;
//extern SemaphoreHandle_t xCollisionMutex;

/* Queues */
//QueueHandle_t movementQ = 0;
extern QueueHandle_t poseControllerQ;
extern QueueHandle_t scanStatusQ;
extern QueueHandle_t queue_microsd;
extern QueueHandle_t queue_display;

extern QueueHandle_t targetReachedQ;

extern QueueHandle_t encoderTicksToMotorSpeedControllerQ;
extern QueueHandle_t encoderTicksToMotorPositionControllerQ;
extern QueueHandle_t encoderTicksToEstimatorTaskQ;

// Flag to indicate connection status. Interrupt can change handshook status
//extern uint8_t gHandshook;
//extern uint8_t gPaused;


//Globals for direction
extern int LeftMotorDirection;
extern int RightMotorDirection;


/* STRUCTURE */
typedef struct sPolar {
    float heading;
    int16_t distance;
} polar;

typedef struct sCartesian {
    float x;
    float y;
} cartesian;










#endif  /* GLOBALS_H */