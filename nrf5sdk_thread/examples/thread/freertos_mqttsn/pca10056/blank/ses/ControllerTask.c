/************************************************************************/
// File:            ControllerTask.c                                    //
// Author:																//
// Purpose:                                                             //
//                                                                      //
/************************************************************************/

#include "defines.h"

#include "encoder.h"

#include "freeRTOS.h"

#include "functions.h"

#include "math.h"

#include "motor.h"

#include "nrf_log.h"

#include "queue.h"

#include "semphr.h"

#include "timers.h"

#include "ControllerTask.h"

#include "positionEstimate.h"

// TODO: #include "server_communication.h"
#include "thread_mqttsn.h"

#include "MotorSpeedControllerTask.h"

#include "globals.h"

#include "PID_controller.h"

#include "robot_config.h"


TickType_t ticks_since_startup;

/* Added 12.05.2020 */
float previousThetahat = 0;
//int controllerPrint = 0;
bool integrateTheta = true;
uint8_t thetaDoneCounter = 0;
float maxU = 30.0;
float minU = 20.0; //was 14.0
float stopU = 1.0;

float thetaErrorInt = 0;
float thetaErrIntegral;

float distanceDriven = 0;
float distanceToTarget = 0;

// Logging test
bool controllerLogDone = false;
int controllerLogCounter = 0;
int controllerTime = 0;
char controllerdata[20];

float thetaTargt = 0;

bool executingOrder = false;

/*  Calculates new settings for the movement task */
void vMainPoseControllerTask(void * pvParameters) {

  NRF_LOG_INFO("mainPoseControllerTask: initializing");
  /* Task init */
  struct sCartesian Setpoint = {
    0,
    0
  }; // Updates from server
  float radiusEpsilon = 0.015; //[m]The acceptable radius from goal for completion
  uint8_t lastMovement = 0;

  mqttsn_target_msg_t target_msg;
  mqttsn_controller_msg_t controller_msg;

  //uint8_t maxDriveActuation = 90; //The max speed the motors will run at during drive max is 100, check also MAX_DUTY in motor.c.

  /* Controller variables for tuning */
  //float rotateThreshold = 10.0 * DEG2RAD; // [rad] The threshold at which the robot will go from driving to rotation.
  float driveThreshold = 3.0 * DEG2RAD; // [rad ]The threshold at which the robot will go from rotation to driving.

  /* Current position variables */
  float thetahat = 0;
  float xhat = 0;
  float yhat = 0;

  // Errors
  float distanceError = 0;
  float thetaError = 0;
  float xError = 0;
  float yError = 0;

  // Targets
  float xTargt = 0;
  float yTargt = 0;

  float theta_correction = 0;

  uint8_t doneTurning = false;
  //uint8_t doneDriving = false;
  uint8_t idleSendt = false;

  /* TESTING VARIABLES */
  uint8_t controllerStop = false;
  //float distanceStart = 0;
  //float prevDistError = 0;
  //float prevThetaError = 0;
  //float thetahatStart = 0;
  float xhatStart = 0;
  float yhatStart = 0;

  //uint8_t newOrder = false;
  bool collisionDetected = false;

  ticks_since_startup = 0;

  //PID
  float left_u, right_u;
  left_u = right_u = 0.0;

  PID_parameters_t pid_parameters_distance, pid_parameters_heading;
  pid_parameters_distance.K_P = ROBOT_DISTANCE_CTRL_K_P;
  pid_parameters_distance.K_I = ROBOT_DISTANCE_CTRL_K_I;
  pid_parameters_distance.K_D = ROBOT_DISTANCE_CTRL_K_D;
  pid_parameters_distance.integral_boundary = ROBOT_DISTANCE_CTRL_ERROR_INTEGRAL_BOUNDARY;
  pid_parameters_distance.max_output = ROBOT_DISTANCE_CTRL_MAX_OUTPUT;
  pid_parameters_distance.min_output = ROBOT_DISTANCE_CTRL_MIN_OUTPUT;
  pid_parameters_distance.measurement_previous = 0;
  pid_parameters_distance.error_previous = 0;
  pid_parameters_distance.error_integral = 0;
  pid_parameters_distance.output_previous = 0;
  pid_parameters_distance.derivative_select = ERROR;

  pid_parameters_heading.K_P = ROBOT_HEADING_CTRL_K_P;
  pid_parameters_heading.K_I = ROBOT_HEADING_CTRL_K_I;
  pid_parameters_heading.K_D = ROBOT_HEADING_CTRL_K_D;
  pid_parameters_heading.integral_boundary = ROBOT_HEADING_CTRL_ERROR_INTEGRAL_BOUNDARY;
  pid_parameters_heading.max_output = ROBOT_HEADING_CTRL_MAX_OUTPUT;
  pid_parameters_heading.min_output = ROBOT_HEADING_CTRL_MIN_OUTPUT;
  pid_parameters_heading.measurement_previous = 0;
  pid_parameters_heading.error_previous = 0;
  pid_parameters_heading.error_integral = 0;
  pid_parameters_heading.output_previous = 0;
  pid_parameters_heading.derivative_select = MEASUREMENT;

  PID_parameters_t pid_parameters_heading_while_driving;
  pid_parameters_heading_while_driving.K_P = ROBOT_HEADING_WHILE_DRIVING_CTRL_K_P;
  pid_parameters_heading_while_driving.K_I = ROBOT_HEADING_WHILE_DRIVING_CTRL_K_I;
  pid_parameters_heading_while_driving.K_D = ROBOT_HEADING_WHILE_DRIVING_CTRL_K_D;
  pid_parameters_heading_while_driving.integral_boundary = ROBOT_HEADING_WHILE_DRIVING_CTRL_ERROR_INTEGRAL_BOUNDARY;
  pid_parameters_heading_while_driving.max_output = ROBOT_HEADING_WHILE_DRIVING_CTRL_MAX_OUTPUT;
  pid_parameters_heading_while_driving.min_output = ROBOT_HEADING_WHILE_DRIVING_CTRL_MIN_OUTPUT;
  pid_parameters_heading_while_driving.measurement_previous = 0;
  pid_parameters_heading_while_driving.error_previous = 0;
  pid_parameters_heading_while_driving.error_integral = 0;
  pid_parameters_heading_while_driving.output_previous = 0;
  pid_parameters_heading_while_driving.derivative_select = ERROR;

  if (USE_SPEED_CONTROLLER) {
    // Wait for initialization of speed controller
    UNUSED_RETURN_VALUE(ulTaskNotifyTake(pdTRUE, portMAX_DELAY));
  }

  setMotorSpeedReference(0, 0);
  motor_brake();

  TickType_t xLastWakeTime = xTaskGetTickCount();

  NRF_LOG_INFO("mainPoseControllerTask: init complete");
  if (LOG_ROBOT_POSITION_CONTROLLER) NRF_LOG_INFO("Time;Reference X;Reference Y;Reference heading;Estimated X;Estimated Y; Estimated heading;Left u; Right u\n\r");

  while (1) {
    /*  Used to check timing
    int previousTime = xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    int timeDifference = xLastWakeTime-previousTime;
    count++;
    if(count > 500){
    	NRF_LOG_INFO("TimeDiff: %i", (int)timeDifference);
    	count = 0;
    }
    */

   // if (gHandshook || !CONNECT_TO_SERVER) {

      if (xSemaphoreTake(xControllerBSem, portMAX_DELAY) == pdTRUE) // Wait for synchronization from estimator
      {
        if (USE_SPEED_CONTROLLER) {
          // TODO: Pose controller period should be larger than speed controller period in order for successive loop closure to work
          int POSE_CTRL_TASK_DELAY_TIME = MOTOR_SPEED_CTRL_TASK_DELAY_TIME * 4;
          vTaskDelayUntil( & xLastWakeTime, POSE_CTRL_TASK_DELAY_TIME);
        }

        if (LOG_ROBOT_POSITION_CONTROLLER) NRF_LOG_INFO("Running controller task");

        TickType_t ticks_since_startup_prev = ticks_since_startup;
        ticks_since_startup = xTaskGetTickCount();
        float delta_t = (ticks_since_startup - ticks_since_startup_prev) * 1.0 / configTICK_RATE_HZ;

        previousThetahat = thetahat;
        
        // Get robot pose
        xSemaphoreTake(xPoseMutex, portMAX_DELAY);
        thetahat = get_position_estimate_heading();
        xhat = (get_position_estimate_x()); //m to mm
        yhat = (get_position_estimate_y());
        xSemaphoreGive(xPoseMutex);

        /************************************************
         * Update waypoint if new waypoint is given
         *************************************************/
        if (xQueueReceive(get_queue_handle("v2/server/NRF_5/cmd"), &target_msg, (TickType_t) 0) == pdTRUE) {
          xTargt = target_msg.target_x / 1000.0; //Distance is received in mm, convert to m for continuity
          yTargt = target_msg.target_y / 1000.0; //Distance is received in mm, convert to m for continuity
          xhatStart = xhat;
          yhatStart = yhat;
          controllerStop = false;
          doneTurning = false;
          collisionDetected = false;
          NRF_LOG_INFO("Received target from server: (%d, %d)", target_msg.target_x, target_msg.target_y);
        }
        
        /************************************************
         * Find error
         *************************************************/
        //prevDistError = distanceError;
        distanceToTarget = (float) sqrt((xTargt - xhatStart) * (xTargt - xhatStart) + (yTargt - yhatStart) * (yTargt - yhatStart));

        distanceError = (float) sqrt((xTargt - xhat) * (xTargt - xhat) + (yTargt - yhat) * (yTargt - yhat));

        distanceDriven = (float) sqrt((xhat - xhatStart) * (xhat - xhatStart) + (yhat - yhatStart) * (yhat - yhatStart));

        xError = xTargt - xhat;
        yError = yTargt - yhat;
        thetaTargt = atan2(yError, xError); //atan() returns radians
        vFunc_Inf2pi( & thetaTargt);
        //NRF_LOG_INFO("ThetaTarget : " NRF_LOG_FLOAT_MARKER "\t\r\n", NRF_LOG_FLOAT(thetaTargt));
        //prevThetaError = thetaError;
        thetaError = thetaTargt - thetahat; //Might be outside pi to -pi degrees

        vFunc_Inf2pi( & thetaError);
        //NRF_LOG_INFO("ThetaError : " NRF_LOG_FLOAT_MARKER "\t\r\n", NRF_LOG_FLOAT(thetaError));				

        /************************************************
         * Collision Avoidance
         *************************************************/
        // Check for collision only when turning is complete 
        if (doneTurning) {
          //collisionDetected = checkForCollision();
          collisionDetected = false; //Set as false only for test purpose
        } else {
          collisionDetected = false;
        }

        /************************************************
         * Stop if distance driven is longer than to target
         *************************************************/

        if ((distanceError < radiusEpsilon) || (distanceDriven > distanceToTarget) || (collisionDetected)) {
          if (PRINT_DEBUG) NRF_LOG_INFO("Controller STOP");
          controllerStop = true;
          if (USE_SPEED_CONTROLLER) {
            setMotorSpeedReference(0, 0);
          } else {
            motor_brake();
          }

          left_u = right_u = 0.0;

          thetaErrIntegral = 0.0;
          thetaErrorInt = 0.0;
          lastMovement = moveStop;
          xQueueSend(scanStatusQ, & lastMovement, 0); // Send the current movement to the scan task
        }

        /************************************************
         * Select Heading or distance controller 
         *************************************************/

        if (distanceError > radiusEpsilon && !controllerStop) //Not close enough to target and controllerStop == false
        {
          idleSendt = false;

          //newOrder = false;

          if (doneTurning) //Start forward movement
          {
            if (LOG_ROBOT_POSITION_CONTROLLER) NRF_LOG_INFO("Running distance controller");
            //float distanceTraveled = distanceStart - distanceError;
            float u_distance;
            float u_heading_while_driving;
            if (delta_t > 0.0) {
              u_distance = PID_controller_with_error_as_input( & pid_parameters_distance, distanceError, 0, 0, delta_t);
              u_heading_while_driving = PID_controller_with_error_as_input( & pid_parameters_heading_while_driving, thetaError, 0, 0, delta_t);
              (void) u_heading_while_driving;
              //Add theta offset
              //Sgn_2 reduces exessive turning upon reaching waypoint
              theta_correction = fmin(fabs(u_heading_while_driving) * distanceError, MAX_THETA_ERROR_CORRECTION);
              if (thetaError < 0) {
                theta_correction = -theta_correction;
              }
              left_u = (u_distance - theta_correction); // - driveKd*thetaDer - driveKi*rightIntError);  
              right_u = (u_distance + theta_correction); // + driveKd*thetaDer + driveKi*rightIntError); 

              if (left_u > ROBOT_DISTANCE_CTRL_MAX_OUTPUT) left_u = ROBOT_DISTANCE_CTRL_MAX_OUTPUT;
              else if (left_u < -ROBOT_DISTANCE_CTRL_MAX_OUTPUT) left_u = -ROBOT_DISTANCE_CTRL_MAX_OUTPUT;

              if (right_u > ROBOT_DISTANCE_CTRL_MAX_OUTPUT) right_u = ROBOT_DISTANCE_CTRL_MAX_OUTPUT;
              else if (right_u < -ROBOT_DISTANCE_CTRL_MAX_OUTPUT) right_u = -ROBOT_DISTANCE_CTRL_MAX_OUTPUT;

            } else {
              NRF_LOG_INFO("Delta_t IS ZERO");
            }

            //collisionDetected = checkForCollision();
            collisionDetected = false;
            lastMovement = moveForward;
            xQueueSend(scanStatusQ, & lastMovement, 0); // Send the current movement to the scan task

          } else {
            if (delta_t > 0) {
              if (LOG_ROBOT_POSITION_CONTROLLER) NRF_LOG_INFO("Running theta controller");
              left_u = right_u = PID_controller_with_error_as_input( & pid_parameters_heading, thetaError, thetaTargt, thetahat, delta_t);
              left_u = -left_u;
            }

            lastMovement = (thetaError < 0) ? moveClockwise : moveCounterClockwise; // let EstimatorTask and SensorTask know robot motion.
            xQueueSend(scanStatusQ, & lastMovement, 0); // Send the current movement to the scan task

            if (fabs(thetaError) < driveThreshold) {
              thetaDoneCounter++;
              if (thetaDoneCounter > 20) {
                thetaErrorInt = 0; // Reset integral
                doneTurning = true; // Allow distance controller to run
              }
            }
          }

        } else { // Close enough to target

          if (idleSendt == false) {
            NRF_LOG_INFO("controller sending idle");
            //TODO: send_idle();
            idleSendt = true;
          }

          if (USE_SPEED_CONTROLLER) {
            setMotorSpeedReference(0, 0);
          } else {
            motor_brake();
          }
          lastMovement = moveStop;
          xQueueSend(scanStatusQ, & lastMovement, 0); // Send the current movement to the scan task
          //display_text_on_line(4,"Reached target");
        }

        /************************************************
         * Set output
         *************************************************/
        if (PUBLISH_POSITION_CONTROLLER) {
          double time_since_startup = ticks_since_startup * 1.0 / configTICK_RATE_HZ;
          //NRF_LOG_INFO("Pose controller");
          //NRF_LOG_INFO(""NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER ";"NRF_LOG_FLOAT_MARKER, time_since_startup, xTargt, yTargt, thetaTargt, (double) xhat, (double) yhat, thetahat, left_u, right_u, theta_correction, thetaError);
          controller_msg.time = time_since_startup;
          controller_msg.x = xhat;
          controller_msg.y = yhat;
          controller_msg.theta = thetahat;
          controller_msg.left_u = left_u;
          controller_msg.right_u = right_u;
          NRF_LOG_INFO("Publish size %d", sizeof(controller_msg))
          publish("v2/robot/NRF_5/controller", &controller_msg, sizeof(controller_msg), 0, 0);
        }

        if (USE_SPEED_CONTROLLER) {
          setMotorSpeedReference(left_u, right_u);
        } else {
          vMotorMovementSwitch(left_u, right_u);
        }
        //NRF_LOG_INFO("left u: %d\t right u: %d", left_u, right_u);

      } else {
        // No semaphore available, task is blocking
        // printf("no semaphore available\n\r");
      }

    //} //if(gHandshook) end
    //else {
    //  if (USE_SPEED_CONTROLLER) {
    //    setMotorSpeedReference(0, 0);
    //  } else {
    //    motor_brake();
    //  }
    //}

  }
}

float getThetaTarget() {
  return thetaTargt;
}