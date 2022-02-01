/************************************************************************/
// File:            EstimatorTask.c                                     //
// Author:																//
// Purpose:                                                             //
//                                                                      //
/************************************************************************/

#include "ICM_20948.h"

#include "defines.h"

#include "freeRTOS.h"

#include "functions.h"

//#include "mag3110.h"

#include "math.h"

#include "nrf_log.h"

#include "queue.h"

#include "semphr.h"

#include "timers.h"

#include "ExtendedKalmanFilter.h"

#include "NewEstimatorTask.h"

#include "globals.h"

// TODO #include "ControllerTask.h"

#include <string.h>

#include <stdio.h>

#include <stdarg.h>

#include "encoder_with_counter.h"
 //#include "encoder.h"
#include "positionEstimate.h"

int8_t gyroCalib = 1;
int8_t finished_calibration = 0;

static IMU_reading_t gyro;
static IMU_reading_t accel;
position_estimate_t ekf_state;

/* Pose estimator task */
void vNewMainPoseEstimatorTask(void * pvParameters) {
  NRF_LOG_INFO("NewMainPoseEstimatorTask: initializing");
  
  init_position_estimate();
  encoder_with_counter_init();
  IMU_init();

  int count = 0;
  float period_in_S = PERIOD_ESTIMATOR_MS / 1000.0;
  float accelXoffset = 0;
  float accelYoffset = 0;
  float gyroOffset = 0.0;
  float gyrZ = 0;

  // Initialise the xLastWakeTime variable with the current time.
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t ticks_since_startup = xTaskGetTickCount();

  float Z[4];
  ekf_init(4, 5);
  ekf_state.vel = 0;
  
  float gyroSum = 0;
  float gyroLimit = 0.1;

  encoderTicks ticks_sum = {
    0,
    0
  };

  NRF_LOG_INFO("NewMainPoseEstimatorTask: init complete");

  //float accel_x_temp = 0;
  //encoderTicks encoder_ticks_total;
  //encoder_ticks_total.left = 0;
  //encoder_ticks_total.right = 0;

  //if (LOG_MEASUREMENT_DATA) printf("delta_t accel.x gyrZ encoder_ticks.left encoder_ticks.right ekf_state.heading ekf_state.x ekf_state.y\n\r");
  while (true) {

    TickType_t ticks_since_startup_prev = ticks_since_startup;
    vTaskDelayUntil( & xLastWakeTime, 40); //This delays for 40 tics, not ms. 
    count += 1;
    //The constant portTICK_PERIOD_MS can be used to calculate real time from the tick rate – with the resolution of one tick period
    ticks_since_startup = xTaskGetTickCount();
    float delta_t = (ticks_since_startup - ticks_since_startup_prev) * 1.0 / configTICK_RATE_HZ;

    if (finished_calibration) { // finished_calibration set to true in IMU calibration further down
      //int16_t leftWheelTicks = 0;
      //int16_t rightWheelTicks = 0;
      float dRobot = 0;
      float dTheta = 0;

      /* DATA COLLECTION*/
      // Get encoder data, protect the global tick variables
      encoder_with_counter_get_ticks_since_last_time();
      encoderTicks encoder_ticks, encoder_ticks_temp;
      encoder_ticks.left = 0;
      encoder_ticks.right = 0;

      while (xQueueReceive(encoderTicksToEstimatorTaskQ, & encoder_ticks_temp, 1) == pdTRUE) {
        //NRF_LOG_INFO("Read ticks");
        encoder_ticks.left += encoder_ticks_temp.left;
        encoder_ticks.right += encoder_ticks_temp.right;
      }
      //encoder_ticks_total.left += encoder_ticks.left;
      //encoder_ticks_total.right += encoder_ticks.right;

      float dLeft = (float)(encoder_ticks.left * 1.0 * WHEEL_FACTOR_MM); // Distance left wheel has traveled since last sample
      float dRight = (float)(encoder_ticks.right * 1.0 * WHEEL_FACTOR_MM); // Distance right wheel has traveled since last sample

      NRF_LOG_INFO("dLeft: " NRF_LOG_FLOAT_MARKER "\n\r", NRF_LOG_FLOAT(dLeft));
      NRF_LOG_INFO("dRight: " NRF_LOG_FLOAT_MARKER "\n\r", NRF_LOG_FLOAT(dRight));

      //float dLeft_total = (float)(encoder_ticks_total.left * 1.0*WHEEL_FACTOR_MM);	// Distance left wheel has traveled since last sample
      //float dRight_total = (float)(encoder_ticks_total.right * 1.0*WHEEL_FACTOR_MM);	// Distance right wheel has traveled since last sample

      //printf("left: %f\tright: %f\n\r", dLeft_total, dRight_total);
      dRobot = (dLeft + dRight) / 2;
      dTheta = (dRight - dLeft) / WHEELBASE_MM; // Get angle from encoders, dervied from arch of circles formula NB! When turning in place this will only give 0
      //NRF_LOG_INFO("dTheta: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(dTheta));

      // Get IMU data:
      if (IMU_new_data()) {
        IMU_read();
        gyro = IMU_getGyro();
        accel = IMU_getAccel();
        accel.x -= accelXoffset;
        accel.y -= accelYoffset;
        gyrZ = gyro.z - gyroOffset; // [deg/sec]

        if (fabs(gyrZ) < gyroLimit) { // Compensate for noise from the gyro
          gyrZ = 0.0;
        }
      } else {
        //NRF_LOG_INFO("No new data from IMU");
        gyrZ = 0.0;
      }

      /* Alternative to Kalman Filter for heading. */
      gyroSum += gyrZ * DEG2RAD * (float)(40.0 / 1000.0);

      //NRF_LOG_INFO("gyro_Z: " NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(gyrZ));

      /* Used to check timing issues */
      //TickType_t tickdiff =xTaskGetTickCount()- xLastWakeTime2;
      //xLastWakeTime2 = xTaskGetTickCount(); 
      //NRF_LOG_INFO("ET:%u",(uint32_t) tickdiff);

      //get MAGNETOMETER data:
      // MAG_reading_t mag = mag_read();

      /* SENSOR DATA collected*/

      /*step kalman filter*/
      float cosTheta = cos(get_position_estimate_heading());
      float sinTheta = sin(get_position_estimate_heading());
      // inputs in meters and radians
      Z[0] = dRobot * 0.025; // encoder speed // (distance /(40/1000))/1000
      Z[1] = ekf_state.vel + accel.x * period_in_S * 9.81; //linear velocity [m/s] //ADD PREVIOUS VELOCITY AS WELL? V = V0 + A*T
      //Z[1] = period_in_S*(accel_x_temp+accel.x)/2;
      Z[2] = dTheta * 25.0; // encoder rotation speed (theta_hat)
      Z[3] = gyrZ * DEG2RAD;
      //NRF_LOG_INFO("Z[3]: " NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(Z[3]));

      if ((dRobot == 0) | (fabs(accel.x) < 0.05)) { //if (dRobot ==0)
        //kf_setGyroVar(1);//if wheels arent turning have less trust in gyro
        Z[1] = 0.0; //if wheels arent turning dont trust accelerometer  
        accelXoffset += (accel.x / 1000); //try to correct sensor offset drifting

      } else {
        //kf_setGyroVar(0.0134);
        if (fabs(Z[3] - Z[2]) > 0.2 * Z[3]) {
          ekf_setEncoderVar(1);
        } // if gyro and encoders mismatch (stuck?) trust gyro
        else {
          ekf_setEncoderVar(0.03);
        }
      }

      ekf_step(Z, cosTheta, sinTheta); //step the filter  
      ekf_state = ekf_GetState(); //extract state

      // Update global pose
      vFunc_Inf2pi( & (ekf_state.heading)); // Places angles inside -pi to pi
      vFunc_Inf2pi( & (gyroSum));
      /*
			if(count > 40){
				
				headingTime = (xTaskGetTickCount());
				sprintf(str2, "%d, %d, %d", (int)(kf_state.heading*RAD2DEG), (int)(gyroIntegral*RAD2DEG), (int)(headingTime));
				NRF_LOG_INFO("%s", str2);
		
				count = 0;
			}
			*/
      //accel_x_temp = accel.x;
      xSemaphoreTake(xPoseMutex, 15);
      set_position_estimate_heading(ekf_state.heading); // previously: gTheta_hat = kf_state.heading;  replaced with: gTheta_hat = gyroSum;
      set_position_estimate_x((ekf_state.x));
      set_position_estimate_y((ekf_state.y));
      xSemaphoreGive(xPoseMutex);
      if (PRINT_DEBUG) {
        // BUG: Values explode if the robot reverses
        NRF_LOG_INFO("EKF_x: " NRF_LOG_FLOAT_MARKER "\t\r\n", NRF_LOG_FLOAT(1000 * ekf_state.x));
        NRF_LOG_INFO("EKF_y: " NRF_LOG_FLOAT_MARKER "\t\r\n", NRF_LOG_FLOAT(1000 * ekf_state.y));
        NRF_LOG_INFO("EKF_heading: " NRF_LOG_FLOAT_MARKER "\t\r\n", NRF_LOG_FLOAT(ekf_state.heading));
      }
      /*if (LOG_MEASUREMENT_DATA) {
        //Accel.x and Gyrz is treated with offset
        double time_since_startup = ticks_since_startup * 1.0 / configTICK_RATE_HZ;
        ticks_sum.left += encoder_ticks.left;
        ticks_sum.right += encoder_ticks.right;
        //printf("%f;%f;%f;%ld;%ld;%ld;%ld;%f;%f;%f\n\r", time_since_startup, accel.x, gyrZ, encoder_ticks.left, encoder_ticks.right, ticks_sum.left, ticks_sum.right, ekf_state.heading, ekf_state.x, ekf_state.y);
      }*/
      
      // Send semaphore to controller
      // TODO xSemaphoreGive(xControllerBSem);

    } else if (gyroCalib && !finished_calibration) {
      // IMU calibration
      //neccesary to complete this part only once
      // Not connected, getting heading and gyro bias
      //char str4[20];
      uint16_t i;
      uint16_t samples = 300;
      float gyroF = 0;
      float accelFX = 0;
      float accelFY = 0;
      int fails = 0;
      int sucsess = 0;
      NRF_LOG_INFO("IMU calib init done");
      vTaskDelay(150); //use delay so we dont write before i2c is initialized
      NRF_LOG_INFO("Enter IMU calibration");
      for (i = 0; i < samples; i++) {
        IMU_read(); //needs to be called to get new gyro data
        gyro = IMU_getGyro();
        accel = IMU_getAccel();
        gyroF += gyro.z;
        accelFX += accel.x;
        accelFY += accel.y;

        vTaskDelay(40);
        /*
        sprintf(str4,"cal F:%i S:%i",fails,sucsess);
        display_text_on_line(4,str4);
        */
        sucsess++;

        while (!IMU_new_data()) {
          vTaskDelay(20); // wait for new data
          fails++;
          NRF_LOG_INFO("Waiting for new IMU data");
          /*
          sprintf(str4,"cal F:%i S:%i",fails,sucsess);
          display_text_on_line(4,str4); 
          */
        }
      }
      //NRF_LOG_INFO("aFX: %i aFY: %i gF: %i", gyroF, accelFX, gyroOffset);
      NRF_LOG_INFO("Calib.i: %i", i);
      gyroOffset = gyroF / (float) samples;
      accelXoffset = accelFX / (float) samples;
      accelYoffset = accelFY / (float) samples;
      gyroCalib = 0;
      NRF_LOG_INFO("gyroOffset: "
        NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(gyroOffset));
      NRF_LOG_INFO("accelXOffset: "
        NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(accelXoffset));
      NRF_LOG_INFO("accelYOffset: "
        NRF_LOG_FLOAT_MARKER "\t\t\r\n", NRF_LOG_FLOAT(accelYoffset));
      //NRF_LOG_INFO("aX: %i aY: %i g: %i", accelXoffset, accelYoffset, gyroOffset);

      finished_calibration = 1;

    }

  } // While(1) end

}