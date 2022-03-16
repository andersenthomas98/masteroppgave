
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

static point_buffer_t ir_sensor_point_buffers[NUM_DIST_SENSORS];
static common_point_buffer_t common_point_buffer;

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

polar_t cartesian2polar(float x, float y) {
  float r = sqrt(x*x+y*y);
  float theta = atan2(y, x);
  return (polar_t) {r, theta};
}

void update_point_buffers(point_buffer_t* point_buffers, ir_measurement_t measurement) {

  /*Convert range-bearing measurement to obstacle position in global reference frame and update point buffers with obstacle positions */
  float theta = measurement.servo_angle*DEG2RAD + measurement.theta;
  //NRF_LOG_INFO("Servo angle: %d", measurement.servo_angle);
  for (int i=0; i<NUM_DIST_SENSORS; i++) {

    //NRF_LOG_INFO("Sensor %d range: %d", i, measurement.measurements[i]);

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
    //NRF_LOG_INFO("%d", pb_len);
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

  for (int i=0; i<NUM_DIST_SENSORS; i++) {
    ir_sensor_point_buffers[i].len = 0;
  }

  common_point_buffer.len = 0;


  mqttsn_init_msg_t rx_msg;

  ir_measurement_t new_measurement;

  // Block until sensor tower task has initialized ir measurement queue
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  NRF_LOG_INFO("mapping task initialized");
  

  while(1) {
      /* Receive ir sensor measurement + robot pose from sensor tower task */
      if (xQueueReceive(ir_measurement_queue, &(new_measurement), (TickType_t) 10) == pdPASS) {
          update_point_buffers(&ir_sensor_point_buffers, new_measurement);

      }
      
      /* Start line extraction */
      if (ulTaskNotifyTake(pdTRUE, (TickType_t) 0) == pdPASS) {
        NRF_LOG_INFO("Start line extraction");
        
        
        // Gather point buffers into one buffer
        for (int i=0; i<NUM_DIST_SENSORS; i++) {
          uint8_t pb_len = ir_sensor_point_buffers[i].len;
          ir_sensor_point_buffers[i].len = 0; // Clear ir sensor's point buffer
          for (int j=0; j<pb_len; j++) {
            //NRF_LOG_INFO("j: %d", j);
            uint8_t curr_len = common_point_buffer.len;
            common_point_buffer.buffer[curr_len] = ir_sensor_point_buffers[i].buffer[j];
            common_point_buffer.len += 1;
          }
        }

        // Verify the points are stored sorted by bearing
        uint8_t len = common_point_buffer.len;
        for (int i=0; i<len; i++) {
          polar_t polar = cartesian2polar(common_point_buffer.buffer[i].x, common_point_buffer.buffer[i].y);
          NRF_LOG_INFO("polar theta: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(polar.theta));
        }
      
      }

      //lastWakeTime = xTaskGetTickCount();
      //vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}