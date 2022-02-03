/************************************************************************/
// File:            MotorSpeedControllerTask.h                          //
// Author:          Eivind Jølsgard                                     //
// Purpose:         Controlling the motor speed based on encoder reading//
//                                                                      //
/************************************************************************/

#ifndef MOTOR_SPEED_CONTROLLER_TASK_H
#define MOTOR_SPEED_CONTROLLER_TASK_H

#include <stdint.h>

#include "globals.h"

/**
 * @brief Motor speed controller task
 *
 * @param[in] pvParameters
 */
void vMotorSpeedControllerTask(void *pvParameters);

/**
 * @brief Set motor speed reference
 *
 * @param[in] left_motor Left motor speed reference
 *
 * @param[in] right_motor Right motor speed reference
 * */
void setMotorSpeedReference(int32_t left_motor, int32_t right_motor);








#endif /* MOTOR_SPEED_CONTROLLER_TASK_H */