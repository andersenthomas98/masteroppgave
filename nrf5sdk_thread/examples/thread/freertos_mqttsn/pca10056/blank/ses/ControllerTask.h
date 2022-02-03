/************************************************************************/
// File:            ControllerTask.h                                    //
// Author:                                                              //
// Purpose:         ControllerTask                                      //
//                                                                      //
/************************************************************************/

#ifndef CONTROLLER_TASK_H
#define CONTROLLER_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

void vMainPoseControllerTask(void *pvParameters);
float getThetaTarget(void);
void runDistanceController(float distanceErr, float thetaErr, float thetaDer);
void runThetaController(float thetaDiff);







#endif /* CONTROLLER_TASK_H */