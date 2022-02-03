#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "app_pwm.h"

/**
 * @brief Function for initializing motor driver
 */
void motor_init();

/**
 * @brief Function for running left motor forward with a given speed.
 *
 * @param[in] duty	  The duty cycle, which translates to speed.
 */
void motor_left_forward(int duty);

/**
 * @brief Function for running right motor forward with a given speed.
 *
 * @param[in] duty	  The duty cycle, which translates to speed.
 */
void motor_right_forward(int duty);

/**
 * @brief Function for running both motors forward with a given speed.
 *
 * @param[in] duty	  The duty cycle, which translates to speed.
 */
void motor_forward(int duty);

/**
 * @brief Function to stop supplying motors with power.
 * @details This will result in the motors coasting, which with high speeds may
 * mean that it takes a little while for the motors to come to a full stop.
 */
void motor_stop(void);

void motor_backward(int duty);

/**
 * @brief Function to stop motors by braking them.
 * @details The motor controller board will short the motors, which will create
 * a braking force in the motor. (Insignificant at low speeds?)
 */
void motor_brake(void);

void vMotorMovementSwitch(int leftSpeed, int rightSpeed);

#endif /* _MOTOR_H_ */