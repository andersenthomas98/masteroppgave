
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
#include "mapping_types.h"
#include "mapping_utils.h"
#include "DBSCAN.h"
#include "IEPF.h"

extern QueueHandle_t ir_measurement_queue;
extern TaskHandle_t mapping_task_handle;


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
    point_buffers[i].buffer[pb_len].x = point.x;
    point_buffers[i].buffer[pb_len].y = point.y;
    point_buffers[i].buffer[pb_len].label = LABEL_UNDEFINED;

    if (pb_len >= PB_MAX_SIZE) {
      // Start line extraction
      xTaskNotifyGive(mapping_task_handle);

    }
  }
}

void mapping_task(void *arg) {

  TickType_t lastWakeTime;
  const TickType_t delay = 0.1;
  
  point_buffer_t point_buffers[NUM_DIST_SENSORS];
  cluster_buffer_t cluster_buffers[NUM_DIST_SENSORS];

  for (int i=0; i<NUM_DIST_SENSORS; i++) {
    point_buffers[i].len = 0;
    cluster_buffers[i].len = 0;
  }

  mqttsn_init_msg_t rx_msg;

  ir_measurement_t new_measurement;

  // Block until sensor tower task has initialized ir measurement queue
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  NRF_LOG_INFO("mapping task initialized");

  // Testing
  int freeHeap = xPortGetFreeHeapSize();
  NRF_LOG_INFO("Free heap before: %d", freeHeap);
  point_buffer_dynamic_t points;
  points.len = 6;
  points.buffer = pvPortMalloc(sizeof(point_t)*points.len);

  point_t test_points[6] = {
    {
      .x = 2,
      .y = -2
    },
    {
      .x = 2,
      .y = 0
    },
    {
      .x = 2,
      .y = 2
    },
    {
      .x = 0,
      .y = 2
    },
    {
      .x = -2,
      .y = 2
    }, 
    {
      .x = -3,
      .y = 2
    }
  
  };

  for (int i=0; i<points.len; i++) {
    points.buffer[i] = test_points[i];
  
  }

  NRF_LOG_INFO("Size of float: %d", sizeof(float));
  NRF_LOG_INFO("Size of int16_t: %d", sizeof(int16_t));
  NRF_LOG_INFO("Size of point_t: %d", sizeof(point_t));

  cluster_buffer_t output;
  output.len = 0;
  output.buffer = NULL;

  IEPF(points, &output, 0.1);
  freeHeap = xPortGetFreeHeapSize();
  NRF_LOG_INFO("Free heap after IEPF: %d", freeHeap);

  NRF_LOG_INFO("output len %d", output.len);

  for (int i=0; i<output.len; i++) {
    NRF_LOG_INFO("Line cluster %d", i);
    for (int j=0; j<output.buffer[i].len; j++) {
      volatile point_t point = output.buffer[i].buffer[j];
      NRF_LOG_INFO("("NRF_LOG_FLOAT_MARKER","NRF_LOG_FLOAT_MARKER")", NRF_LOG_FLOAT(point.x), NRF_LOG_FLOAT(point.y));
    }
  
  }
  deallocate_cluster_buffer(output);

  freeHeap = xPortGetFreeHeapSize();
  NRF_LOG_INFO("Free heap after deallocation in mapping: %d", freeHeap);


  while(1) {
    NRF_LOG_INFO("testing");
  }

  

  while(1) {
      
      
      /* Receive ir sensor measurement + robot pose from sensor tower task */
      if (xQueueReceive(ir_measurement_queue, &(new_measurement), (TickType_t) 10) == pdPASS) {
          update_point_buffers(&point_buffers, new_measurement);


      }
      
      /* Start line extraction */
      if (ulTaskNotifyTake(pdTRUE, (TickType_t) 0) == pdPASS) {
        
        NRF_LOG_INFO("Start line extraction");

        // Verify the points are stored sorted by bearing
        /*uint8_t len = common_point_buffer.len;
        for (int i=0; i<len; i++) {
          polar_t polar = cartesian2polar(common_point_buffer.buffer[i].x, common_point_buffer.buffer[i].y);
          wrap_to_2pi(&polar.theta);
          NRF_LOG_INFO("polar theta: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(polar.theta));
        }*/
        
        //uint8_t max_len = 3;
        int freeHeap = xPortGetFreeHeapSize();
        NRF_LOG_INFO("Free heap before: %d", freeHeap);
        for (int i=0; i<NUM_DIST_SENSORS; i++) {
          //NRF_LOG_INFO("DBSCAN #%d", i);
          
          // First filtering - Find points near each other
          cluster_buffer_t clusters = DBSCAN(&point_buffers[i], euclidean_distance, 10, 5); // Allocates memory on heap
          point_buffers[i].len = 0;
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap before IEPF: %d", freeHeap);
          for (int j=0; j<clusters.len; j++) {
            cluster_buffer_t line_clusters;
            line_clusters.len = 0;
            line_clusters.buffer = NULL;
            // Second filtering - Find clusters of points which make up line segments
            IEPF(clusters.buffer[j], &line_clusters, 1.0);
            //for (int k=0; k<line_clusters.len; k++) {
              //NRF_LOG_INFO("Points in line segment %d: %d", k, line_clusters.buffer[k].len);
            //}
            
            
            // TODO: Third filtering - Least-square line fitting using point clusters found in previous step

            // TODO: Fourth filtering - Constrained Hough Transform

            // TODO: Append lines extracted from one ir sensor to common buffer

            // TODO: Merge lines together weighted by their covariances.

            deallocate_cluster_buffer(line_clusters);
          
          }
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after IEPF: %d", freeHeap);

          if (clusters.len == 0) {
            continue;
          } 
          else {
            deallocate_cluster_buffer(clusters);
          }

  

          // Verify cluster
          /*for (int j=0; j<clusters.len; j++) {
            NRF_LOG_INFO("--------- Cluster %d --------", j);
            for (int k=0; k<clusters.buffer[j].len; k++) {
              point_t point = clusters.buffer[j].buffer[k];
              NRF_LOG_INFO("label: %d", point.label);
            }
          }*/


        }

        freeHeap = xPortGetFreeHeapSize();
        NRF_LOG_INFO("Free heap after: %d", freeHeap);

        /*for (int i=0; i<NUM_DIST_SENSORS; i++) {
          IEPF(point_buffers[i], &cluster_buffers[i], 0.5);
          point_buffers[i].len = 0;
        
        }*/
        
      } // ulTaskNotify [end]

      //lastWakeTime = xTaskGetTickCount();
      //vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}