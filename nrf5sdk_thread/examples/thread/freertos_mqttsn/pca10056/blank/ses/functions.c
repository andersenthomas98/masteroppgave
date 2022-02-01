/************************************************************************/
// File:            functions.h                                         //
// Author:          Arild Stenset, NTNU Spring 2020                     //
// Purpose:         Arrange useful functions                            //
//                                                                      //
/************************************************************************/

#include "functions.h"

#include <math.h>

#include "stdint.h"

#include "defines.h"

#include "i2c.h"

#include "globals.h"

#include "nrf_log.h"

#include "ir.h"

#include "robot_config.h"

uint8_t collisionCounter;

//find sign of double
int sgn_1(float v) {
  if (v == 0) return 0;
  return (v < 0) ? -1 : 1;
}

float sgn_2(float v, float zero_threshold) {
  return v / (fabs(v) + zero_threshold);
}

/* Take any angle and put it inside -PI, PI */
void vFunc_Inf2pi(float * angle_in_radians) {
  float result = fmod( * angle_in_radians, 2 * M_PI); //any angle is on form x +2*pi*n 
  if (result > M_PI) { //limit angle to +_ pi instead of +_2 pi 
    result -= 2 * M_PI;
  } else if (result < -M_PI) {
    result += 2 * M_PI;
  }
  * angle_in_radians = result;
}

/* Calculates the distance in x direction to a measured object in global coordinate system */
int16_t distObjectX(int16_t x, float theta, int8_t servoAngle, int16_t * sensorData, uint8_t sensorNumber) {
  int16_t xDist = x - cos(theta) * SENSOR_TOWER_OFFSET_X_MM + cos(theta + (servoAngle + sensorNumber * 90) * DEG2RAD) * (sensorData[sensorNumber]);

  return xDist;
}

/* Returns the distance to object in local robot coordinate system */
int16_t distObjectXlocal(float theta, int8_t servoAngle, int16_t * sensorData, uint8_t sensorNumber) {
  int16_t xDist = cos((servoAngle + sensorNumber * 90) * DEG2RAD) * (sensorData[sensorNumber]);

  return xDist;
}

/* Calculates the distance in y direction to a measured object in global coordinate system */
int16_t distObjectY(int16_t y, float theta, int8_t servoAngle, int16_t * sensorData, uint8_t sensorNumber) {
  int16_t yDist = y - cos(theta) * SENSOR_TOWER_OFFSET_Y_MM + sin(theta + (servoAngle + sensorNumber * 90) * DEG2RAD) * (sensorData[sensorNumber]);

  return yDist;
}

/* Returns the distance to object in local robot coordinate system */
int16_t distObjectYlocal(float theta, int8_t servoAngle, int16_t * sensorData, uint8_t sensorNumber) {
  int16_t yDist = sin((servoAngle + sensorNumber * 90) * DEG2RAD) * (sensorData[sensorNumber]);

  return yDist;
}

int16_t lastX = 0;
int16_t lastY = 0;
float lastTheta = 0;

/* Arranges the message with robot positons, object positions and sends it. Used from February 2020*/
void sendNewPoseMessage(int16_t x, int16_t y, float theta, int8_t servoAngle, int16_t * sensorData) {
  uint8_t scanMessageID = 2;
  uint8_t msgLength = 24;
  uint8_t data[msgLength];
  int16_t xObject;
  int16_t yObject;
  int16_t xDiff = x - lastX;
  int16_t yDiff = y - lastY;
  int16_t thetaDiff = (theta - lastTheta) * RAD2DEG;
  lastX = x;
  lastY = y;
  lastTheta = theta;

  data[23] = 0;

  data[0] = scanMessageID;
  data[1] = (xDiff & 0xFF); //xLowByte                
  data[2] = (xDiff >> 8); //xHighByte
  data[3] = (yDiff & 0xFF);
  data[4] = (yDiff >> 8);
  data[5] = (thetaDiff & 0xFF);
  data[6] = (thetaDiff >> 8);

  for (int i = 0; i < NUM_DIST_SENSORS; i++) {
    xObject = distObjectXlocal(theta, servoAngle, sensorData, i);
    yObject = distObjectYlocal(theta, servoAngle, sensorData, i);
    data[i * 4 + 7] = (xObject & 0xFF);
    data[i * 4 + 8] = (xObject >> 8);
    data[i * 4 + 9] = (yObject & 0xFF);
    data[i * 4 + 10] = (yObject >> 8);

    if (sensorData[i] < IR_MAX_DETECT_DISTANCE_MM) {
      data[23] |= (1 << ((NUM_DIST_SENSORS - i) - 1));
    } else {
      data[23] &= ~(1 << ((NUM_DIST_SENSORS - i) - 1));
    }
  }
  i2c_sendNOADDR(I2C_DEVICE_DONGLE, data, msgLength);
}

/* Arranges the message with robot positons, object positions and returns an array */
/* If using Grindviks server version, this function has to be used to format the messages */

void sendOldPoseMessage(int16_t x, int16_t y, float theta, int8_t servoAngle, int16_t * sensorData) {
  uint8_t msgLength = 8;
  uint8_t data[msgLength];
  int16_t xObject;
  int16_t yObject;

  data[0] = (x & 0xFF); // lowbyte               
  data[1] = (x >> 8); // highByte
  data[2] = (y & 0xFF);
  data[3] = (y >> 8);

  for (int i = 0; i < NUM_DIST_SENSORS; i++) {
    if (sensorData[i] < IR_MAX_DETECT_DISTANCE_MM) {
      xObject = distObjectX(x, theta, servoAngle, sensorData, i);
      yObject = distObjectY(y, theta, servoAngle, sensorData, i);
      data[4] = (xObject & 0xFF);
      data[5] = (xObject >> 8);
      data[6] = (yObject & 0xFF);
      data[7] = (yObject >> 8);
      i2c_sendNOADDR(I2C_DEVICE_DONGLE, data, msgLength);
    }
  }
}

void sendScanBorder() {
  uint8_t msgLength = 1;
  uint8_t scanBorderID = 1;
  i2c_sendNOADDR(I2C_DEVICE_DONGLE, & scanBorderID, msgLength);
}

int16_t collisionSectors[2 * NUM_DIST_SENSORS] = {
  360,
  360,
  360,
  360,
  360,
  360,
  360,
  360
}; // 360 is chosen randomly, any number above 180 will probably work
uint8_t collisionPrinter = 0;

void increaseCollisionSector(int16_t angle, uint8_t sensor) {

  int16_t lowerLimit = collisionSectors[sensor * 2];
  int16_t upperLimit = collisionSectors[sensor * 2 + 1];

  if (lowerLimit == 360 && upperLimit == 360) {
    collisionSectors[sensor * 2] = angle - 1;
    collisionSectors[sensor * 2 + 1] = angle + 1;
  } else {
    if (angle < lowerLimit) {
      collisionSectors[sensor * 2] = angle;
    } else if (angle > upperLimit) {
      collisionSectors[sensor * 2 + 1] = angle;
    }
  }
  //printCollisionSectors();
}

/* Returns detection angle inside -179->180 based on servoAngle and which sensor */
int16_t getDetectionAngle(uint8_t servoAngle, uint8_t sensor) {
  int16_t sensorAngle = servoAngle + sensor * 90;
  if (sensorAngle > 180) {
    return (sensorAngle - 360);
  } else {
    return sensorAngle;
  }
}

void decreaseCollisionSector(int16_t angle, uint8_t sensor) {

  int16_t lowerLimit = collisionSectors[sensor * 2];
  int16_t upperLimit = collisionSectors[sensor * 2 + 1];

  if ((angle >= lowerLimit) && (angle <= upperLimit)) {
    if ((angle - lowerLimit) < (upperLimit - angle)) {
      collisionSectors[sensor * 2] = angle;
    } else if ((angle - lowerLimit) > (upperLimit - angle)) {
      collisionSectors[sensor * 2 + 1] = angle;
    } else {
      collisionSectors[sensor * 2] = 360;
      collisionSectors[sensor * 2 + 1] = 360;
    }
  }
  //printCollisionSectors();
}

void printCollisionSectors(void) {
  collisionPrinter++;
  if (collisionPrinter > 60) {
    for (int i = 0; i < NUM_DIST_SENSORS; i++) {
      NRF_LOG_INFO("CollSector: %i. From: %i", (int) i, (int) collisionSectors[i * 2]);
      NRF_LOG_INFO("CollSector: %i.   To: %i", (int) i, (int) collisionSectors[i * 2 + 1]);
    }
    collisionPrinter = 0;
  }
}

/*  */
bool validWaypoint(int16_t waypointAngle) {
  uint8_t noCollision = 0;

  for (int i = 0; i < NUM_DIST_SENSORS; i++) {
    if ((collisionSectors[i * 2] != 360) || (collisionSectors[i * 2 + 1] != 360)) {

      int16_t upperLimit = collisionSectors[i * 2 + 1] + COLLISION_SECTOR_OFFSET;
      int16_t lowerLimit = collisionSectors[i * 2] - COLLISION_SECTOR_OFFSET;

      if (upperLimit > 179) {
        if ((waypointAngle > lowerLimit) && (waypointAngle + 360 < upperLimit)) {
          return false;
        }
      } else if (lowerLimit < -179) {
        if ((waypointAngle - 360 > lowerLimit) && (waypointAngle < upperLimit)) {
          return false;
        }
      } else if ((waypointAngle > lowerLimit) && (waypointAngle < upperLimit)) {
        return false;
      }
    } else {
      noCollision++;
    }
  }

  if (noCollision == 4) {
    return true;
  }
  return false;
}

/* Returns true if a collision at sensor 1 is detected */
bool checkForCollision() {
  if (IrAnalogToMM(ir_read_blocking(0), 0) < COLLISION_THRESHOLD_MM) {
    return true;
  }
  return false;
}

/* Same purpose as above, but checks all ir-sensors */
/*
bool checkForCollision(){
	int sensorReading = 1000;
	for(int i = 0; i < NUM_DIST_SENSORS; i++){
		sensorReading = IrAnalogToMM(ir_read_blocking(i), i);
		if(sensorReading < COLLISION_THRESHOLD_MM){
			return true;
		}
	}
	return false;
}*/