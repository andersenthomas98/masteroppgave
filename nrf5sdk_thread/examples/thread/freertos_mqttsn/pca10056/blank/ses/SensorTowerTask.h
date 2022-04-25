/************************************************************************/
// File:            SensorTowerTask.h                                   //
// Author:                                                              //
// Purpose:         SensorTowerTask                                     //
//                                                                      //
/************************************************************************/

#ifndef SENSOR_TOWER_TASK_H
#define SENSOR_TOWER_TASK_H

typedef struct ir_measurement {
  uint8_t servo_angle;
  int8_t servo_dir;
  uint16_t measurements[4];
  float x;
  float y;
  float theta

} ir_measurement_t;

void vMainSensorTowerTask(void *pvParameters);


#endif /* SENSOR_TOWER_TASK_H */