
#include "mqttsn_client.h"
#include "mapping.h"
#include "thread_mqttsn.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "nrf_log.h"
#include "SensorTowerTask.h"
#include "robot_config.h"

#include "math.h"
#include "defines.h"

#define PB_SIZE 100

extern QueueHandle_t ir_measurement_queue;
extern TaskHandle_t mapping_task_handle;

static point_buffer_t point_buffers[NUM_DIST_SENSORS];

/* Wrap any angle in radians into the interval [0,2pi) */
void wrap_to_2pi(float *angle_in_radians) {
    do {
        if (*angle_in_radians >= 2*M_PI) *angle_in_radians -= 2*M_PI;
        else if (*angle_in_radians < 0) *angle_in_radians += 2*M_PI;
    } while (fabs(*angle_in_radians) >= 2*M_PI);
}

/* Convert from polar to cartesian coordinates */
point_t polar2cartesian(float theta, float r) {
    float x = r * cos(theta);
    float y = r * sin(theta);
    return (point_t) { x, y };
}

void update_point_buffers(point_buffer_t* point_buffers, ir_measurement_t measurement) {

  /*Convert range-bearing measurement to obstacle position in global reference frame and update point buffers with obstacle positions */
  float theta = measurement.servo_angle*DEG2RAD + measurement.theta;
  NRF_LOG_INFO("Servo angle: %d", measurement.servo_angle);
  for (int i=0; i<NUM_DIST_SENSORS; i++) {

    NRF_LOG_INFO("Sensor %d range: %d", i, measurement.measurements[i]);

    if (i > 0) {
      theta += 0.5 * M_PI; // 90 degrees between each ir sensor
    }
    wrap_to_2pi(&theta);
    uint16_t range = measurement.measurements[i];

    // Prune away measurements outside valid range
    if (range <= 0 || range > IR_MAX_DETECT_DISTANCE_MM) {
      continue;
    }

    point_t point = polar2cartesian(theta, (float) range);
    point.x += measurement.x;
    point.y += measurement.y;
    uint8_t pb_len = point_buffers[i].len;
    NRF_LOG_INFO("%d", pb_len);
    point_buffers[i].len++;
    point_buffers[i].buffer[pb_len] = point;

    if (pb_len >= PB_MAX_SIZE) {
      // Start line extraction
      xTaskNotifyGive(mapping_task_handle);

    }
  }
}

void mapping_task(void *arg) {

  TickType_t lastWakeTime;
  const TickType_t delay = 0.1;


  mqttsn_init_msg_t rx_msg;

  ir_measurement_t new_measurement;

  // Block until sensor tower task has initialized ir measurement queue
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  NRF_LOG_INFO("mapping task initialized");
  

  while(1) {
      /* Receive ir sensor measurement + robot pose from sensor tower task */
      if (xQueueReceive(ir_measurement_queue, &(new_measurement), (TickType_t) 10) == pdPASS) {
          update_point_buffers(&point_buffers, new_measurement);

      }
      
      /* Start line extraction */
      if (ulTaskNotifyTake(pdTRUE, (TickType_t) 0) == pdPASS) {
        NRF_LOG_INFO("Start line extraction");

        // Gather point buffers into one buffer
        for (int i=0; i<NUM_DIST_SENSORS; i++) {
          

        }
      
      }

      lastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}