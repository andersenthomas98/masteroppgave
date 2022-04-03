
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
#include "LS_line_fit.h"
#include "MSE_line_fit.h"

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

  volatile line_buffer_t line_buffer;
  line_buffer.len = 0;

  for (int i=0; i<NUM_DIST_SENSORS; i++) {
    point_buffers[i].len = 0;
    cluster_buffers[i].len = 0;
  }

  mqttsn_line_msg_t msg;
  msg.identifier = LINE_IDENTIFIER;

  mqttsn_update_msg_t update_msg;
  update_msg.identifier = UPDATE_IDENTIFIER;

  float lastPublishedX = 0;
  float lastPublishedY = 0;
  float lastPublishedTheta = 0;

  
  mqttsn_cluster_msg_t dbscan_msg;

  ir_measurement_t new_measurement;

  // Block until sensor tower task has initialized ir measurement queue
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  NRF_LOG_INFO("mapping task initialized");

  // Testing
  /*int freeHeap = xPortGetFreeHeapSize();
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
  }*/

  while(!mqttsn_client_is_connected()) {
    // wait
  
  }

  

  while(1) {
      
      
      /* Receive ir sensor measurement + robot pose from sensor tower task */
      if (xQueueReceive(ir_measurement_queue, &(new_measurement), (TickType_t) 10) == pdPASS) {
          update_point_buffers(&point_buffers, new_measurement);
          
          update_msg.xdelta = new_measurement.x;
          update_msg.ydelta = new_measurement.y;
          update_msg.thetadelta = new_measurement.theta;
          update_msg.ir1 = (coordinate_t) {
            .x = point_buffers[0].buffer[point_buffers[0].len-1].x,
            .y = point_buffers[0].buffer[point_buffers[0].len-1].y
          };
          update_msg.ir2 = (coordinate_t) {
            .x = point_buffers[1].buffer[point_buffers[1].len-1].x,
            .y = point_buffers[1].buffer[point_buffers[1].len-1].y
          };
          update_msg.ir3 = (coordinate_t) {
            .x = point_buffers[2].buffer[point_buffers[2].len-1].x,
            .y = point_buffers[2].buffer[point_buffers[2].len-1].y
          };
          update_msg.ir4 = (coordinate_t) {
            .x = point_buffers[3].buffer[point_buffers[3].len-1].x,
            .y = point_buffers[3].buffer[point_buffers[3].len-1].y
          };
          update_msg.valid = 0x0f;
          publish_update("v2/robot/NRF_5/adv", update_msg, sizeof(update_msg), 0, 0);


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
        NRF_LOG_INFO("Free heap before line extraction: %d", freeHeap);
        for (int i=0; i<NUM_DIST_SENSORS; i++) {
          //NRF_LOG_INFO("DBSCAN #%d", i);
          
          // First filtering - Find points near each other
          cluster_buffer_t clusters = DBSCAN(&point_buffers[i], euclidean_distance, 5, 2); // Allocates memory on heap
          for (int j = 0; j < clusters.len; j++) {
            for (int k=0; k < clusters.buffer[j].len; k++) {
              dbscan_msg.cluster_id = j;
              dbscan_msg.point = (coordinate_t) {.x = clusters.buffer[j].buffer[k].x, .y = clusters.buffer[j].buffer[k].y};
              publish_cluster_point("v2/robot/NRF_5/cluster", dbscan_msg, sizeof(dbscan_msg), 0, 0);
              TickType_t lastWakeTime = xTaskGetTickCount();
              vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
            }
          
          }
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after DBSCAN: %d", freeHeap);

          if (clusters.len <= 0) {
            continue; // Go to next ir sensor readings
          }

          point_buffers[i].len = 0;
          
          point_buffer_dynamic_t cluster_means;
          cluster_means.len = clusters.len;
          cluster_means.buffer = pvPortMalloc(sizeof(point_t)*cluster_means.len);
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after cluster means: %d", freeHeap);

          for (int j=0; j<clusters.len; j++) {
            
            // Find cluster means
            float mean_x = 0;
            float mean_y = 0;
            for (int k=0; k<clusters.buffer[j].len; k++) {
              mean_x += clusters.buffer[j].buffer[k].x / clusters.buffer[j].len;
              mean_y += clusters.buffer[j].buffer[k].y / clusters.buffer[j].len;
            }

            cluster_means.buffer[j] = (point_t){.x = mean_x, .y = mean_y, .label = clusters.buffer[j].len};
          }

          deallocate_cluster_buffer(clusters); // Free allocated heap
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after deallocating clusters: %d", freeHeap);

        
          // Second filtering - Find clusters of points which make up line segments

          cluster_buffer_t line_clusters;
          line_clusters.len = 0;
          line_clusters.buffer = NULL;

          if (cluster_means.len < 2) {
            vPortFree(cluster_means.buffer);
            cluster_means.len = 0;
            continue;
          }
          
          IEPF(cluster_means, &line_clusters, 0.2); // Allocates memory on heap
            
          // Third filtering - Least-square line fitting using line clusters found in previous step
          // Line model: ax + by + c = 0
          for (int k=0; k<line_clusters.len; k++) {
            if (line_clusters.buffer[k].len >= 2) {
              line_t ls_fit_line = MSE_line_fit(line_clusters.buffer[k]);
              line_buffer.buffer[line_buffer.len] = ls_fit_line;
              line_buffer.len++;

              if (line_buffer.len == LB_MAX_SIZE) {
                line_buffer.len = 0;
              }
            
              // Publish line
              if (mqttsn_client_is_connected()) {
                msg.startPoint = (coordinate_t) {.x = ls_fit_line.P.x, .y = ls_fit_line.P.y};
                msg.endPoint = (coordinate_t) {.x = ls_fit_line.Q.x, .y = ls_fit_line.Q.y };
                msg.xdelta = 0;
                msg.ydelta = 0;
                msg.thetadelta = 0;
                publish_line("v2/robot/NRF_5/line", msg, sizeof(mqttsn_line_msg_t), 0, 0);
                TickType_t lastWakeTime = xTaskGetTickCount();
                vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
              }

            }
        
          }



          deallocate_cluster_buffer(line_clusters); // Free heap memory
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after deallocating line clusters: %d", freeHeap);
          vPortFree(cluster_means.buffer);
          cluster_means.len = 0;
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after deallocating cluster_means: %d", freeHeap);


          // TODO: Fourth filtering - Constrained Hough Transform

          // TODO: Append lines extracted from one ir sensor to common buffer

          // TODO: Merge lines together weighted by their covariances.
          
        }

        freeHeap = xPortGetFreeHeapSize();
        NRF_LOG_INFO("Free heap after: %d", freeHeap);

        vTaskSuspend(NULL);
        
      } // ulTaskNotify [end]

      //lastWakeTime = xTaskGetTickCount();
      //vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}