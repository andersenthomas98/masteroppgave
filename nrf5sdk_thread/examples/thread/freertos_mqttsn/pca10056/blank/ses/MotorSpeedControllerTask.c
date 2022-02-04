/************************************************************************/
// File:            ControllerTask.c                                    //
// Author:			Eivind Jølsgard 									//
// Purpose:         Controlling motor speed                             //
//                                                                      //
/************************************************************************/

#include <stdlib.h>

#include <math.h>

#include "defines.h"

#include "freeRTOS.h"

#include "functions.h"

#include "math.h"

#include "motor.h"

#include "nrf_log.h"

#include "queue.h"

#include "semphr.h"

#include "timers.h"

#include "globals.h"

#include "time.h"

#include "MotorSpeedControllerTask.h"

#include "encoder_with_counter.h"

#include "PID_controller.h"

#include "robot_config.h"

double left_motor_reference, right_motor_reference;
double left_motor_speed, right_motor_speed; //For digital low pass filter 
double raw_speed_left, raw_speed_right;

extern TaskHandle_t pose_controller_task_handle;

/*  Motor controller task*/
void vMotorSpeedControllerTask(void * pvParameters) {

  NRF_LOG_INFO("MotorSpeedControllerTask: initializing");

  motor_init();

  left_motor_reference = right_motor_reference = 0;
  left_motor_speed = right_motor_speed = 0;

  TickType_t ticks_since_startup = xTaskGetTickCount();
  TickType_t xLastWakeTime = xTaskGetTickCount();

  /************************************************
   * PID controller parameters
   *************************************************/

  PID_parameters_t pid_parameters_left_motor, pid_parameters_right_motor;
  pid_parameters_left_motor.K_P = MOTOR_SPEED_CTRL_LEFT_K_P;
  pid_parameters_left_motor.K_I = MOTOR_SPEED_CTRL_LEFT_K_I;
  pid_parameters_left_motor.K_D = MOTOR_SPEED_CTRL_LEFT_K_D;
  pid_parameters_left_motor.integral_boundary = MOTOR_SPEED_CTRL_ERROR_INTEGRAL_BOUNDARY;
  pid_parameters_left_motor.min_output = MOTOR_SPEED_CTRL_MIN_OUTPUT;
  pid_parameters_left_motor.max_output = MOTOR_SPEED_CTRL_MAX_OUTPUT;
  pid_parameters_left_motor.measurement_previous = 0;
  pid_parameters_left_motor.error_previous = 0;
  pid_parameters_left_motor.error_integral = 0;
  pid_parameters_left_motor.output_previous = 0;
  pid_parameters_left_motor.derivative_select = ERROR;

  pid_parameters_right_motor.K_P = MOTOR_SPEED_CTRL_RIGHT_K_P;
  pid_parameters_right_motor.K_I = MOTOR_SPEED_CTRL_RIGHT_K_I;
  pid_parameters_right_motor.K_D = MOTOR_SPEED_CTRL_RIGHT_K_D;
  pid_parameters_right_motor.integral_boundary = MOTOR_SPEED_CTRL_ERROR_INTEGRAL_BOUNDARY;
  pid_parameters_right_motor.max_output = MOTOR_SPEED_CTRL_MAX_OUTPUT;
  pid_parameters_right_motor.min_output = MOTOR_SPEED_CTRL_MIN_OUTPUT;
  pid_parameters_right_motor.measurement_previous = 0;
  pid_parameters_right_motor.error_previous = 0;
  pid_parameters_right_motor.error_integral = 0;
  pid_parameters_right_motor.output_previous = 0;
  pid_parameters_right_motor.derivative_select = ERROR;

  NRF_LOG_INFO("MotorSpeedControllerTask: init complete");
  if (LOG_MOTOR_SPEED_CONTROLLER) printf("Time;Reference L;Reference R;Speed L;Speed R;Left u; Right u\n\r");

  if (USE_POSE_CONTROLLER) {
    UNUSED_RETURN_VALUE(xTaskNotifyGive(pose_controller_task_handle));
   }

  //int change_in_reference = 1;
  while (1) {
    TickType_t ticks_since_startup_prev = ticks_since_startup;

    vTaskDelayUntil( & xLastWakeTime, MOTOR_SPEED_CTRL_TASK_DELAY_TIME);

    /************************************************
     * Get elapsed time
     *************************************************/
    ticks_since_startup = xTaskGetTickCount();
    float delta_t = (ticks_since_startup - ticks_since_startup_prev) * 1.0 / configTICK_RATE_HZ;

    /************************************************
     * Get velocity reading
     *************************************************/
    encoder_with_counter_get_ticks_since_last_time();

    encoderTicks encoder_ticks, encoder_ticks_temp;
    encoder_ticks.left = 0;
    encoder_ticks.right = 0;

    while (xQueueReceive(encoderTicksToMotorSpeedControllerQ, & encoder_ticks_temp, 1) == pdTRUE) {
      encoder_ticks.left += encoder_ticks_temp.left;
      encoder_ticks.right += encoder_ticks_temp.right;
    }

    if (PRINT_DEBUG) NRF_LOG_INFO("Encoder ticks: %ld\t%ld\n\r", encoder_ticks.left, encoder_ticks.right);

    raw_speed_left = ((encoder_ticks.left * WHEEL_FACTOR_MM) / 1000.0) / delta_t;
    raw_speed_right = ((encoder_ticks.right * WHEEL_FACTOR_MM) / 1000.0) / delta_t;
    //left_motor_speed  = left_motor_speed *0.3 + 0.7 * raw_speed_right;    //Low pass filter
    //right_motor_speed = right_motor_speed*0.3 + 0.7 * raw_speed_right;	    //Low pass filter

    // At low speeds the raw speed calculated from encoder ticks should be smoothed out more.
    left_motor_speed = left_motor_speed * LOW_PASS_FILTER_SPEED_WEIGHT + (1 - LOW_PASS_FILTER_SPEED_WEIGHT) * raw_speed_left;
    right_motor_speed = right_motor_speed * LOW_PASS_FILTER_SPEED_WEIGHT + (1 - LOW_PASS_FILTER_SPEED_WEIGHT) * raw_speed_right;

    /************************************************
     * Reset PID integral on change in reference sgn
     *************************************************/

    if ((sgn_1(pid_parameters_left_motor.reference_previous) != sgn_1(left_motor_reference)) ||
      (sgn_1(pid_parameters_right_motor.reference_previous) != sgn_1(right_motor_reference))
    ) {
      pid_parameters_left_motor.reference_previous = 0;
      pid_parameters_right_motor.reference_previous = 0;
      pid_parameters_left_motor.error_integral = 0;
      pid_parameters_right_motor.error_integral = 0;

    }

    /************************************************
     * PID
     *************************************************/

    double left_u;
    if (left_motor_reference != 0) {

      left_u = PID_controller( & pid_parameters_left_motor, (double) left_motor_reference, (double) left_motor_speed, (double) delta_t);

    } else {
      left_u = 0;
    }

    double right_u;
    if (right_motor_reference != 0) {
      right_u = PID_controller( & pid_parameters_right_motor, (double) right_motor_reference, (double) right_motor_speed, (double) delta_t);
    } else {
      right_u = 0;
    }

    /************************************************
     * Feed Forward
     *************************************************/

    left_u += MOTOR_SPEED_CTRL_LEFT_FEED_FORWARD * sgn_2(left_motor_reference, 0.1); //sgn_2 approaches zero when reference approaches zero

    right_u += MOTOR_SPEED_CTRL_RIGHT_FEED_FORWARD * sgn_2(right_motor_reference, 0.1);

    /************************************************
     * Logging
     *************************************************/
    if (LOG_MOTOR_SPEED_CONTROLLER) {
      double time_since_startup = ticks_since_startup * 1.0 / configTICK_RATE_HZ;
      NRF_LOG_INFO("Speed controller");
      //NRF_LOG_INFO(NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER, time_since_startup, left_motor_reference, right_motor_reference, left_motor_speed, right_motor_speed, raw_speed_left, raw_speed_right, left_u, right_u);
    }

    /************************************************
     * Set output
     *************************************************/

    //NRF_LOG_INFO("left_u: %ld\t right_u: %ld\r\n", (int)left_u, (int)right_u);
    vMotorMovementSwitch((int) left_u, (int) right_u);

  } //while(1)
}

void setMotorSpeedReference(int32_t left_motor, int32_t right_motor) {
  //NRF_LOG_INFO("New Reference: L: %ld \tR:%ld\t", left_motor, right_motor);
  if (left_motor > 100) left_motor_reference = 100;
  else if (left_motor < -100) left_motor_reference = -100;
  else left_motor_reference = left_motor;

  if (right_motor > 100) right_motor_reference = 100;
  else if (right_motor < -100) right_motor_reference = -100;
  else right_motor_reference = right_motor;

  //Cast reference to m/s
  //left_motor_reference = left_motor_reference  / 250.0;  //Robot max velocity is approx 0.5m/s, allow up to 0.4m/s to have margin for motor differences 
  //right_motor_reference = right_motor_reference  / 250.0;

  left_motor_reference = left_motor_reference * (MOTOR_MAX_SPEED_M_PER_S / 100);
  right_motor_reference = right_motor_reference * (MOTOR_MAX_SPEED_M_PER_S / 100);
}