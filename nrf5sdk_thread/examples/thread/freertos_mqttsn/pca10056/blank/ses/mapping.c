
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
#include "float.h"

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
  int16_t test_x = 0;
  int16_t test_y = 0;

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
          for (int j=0; j<clusters.len; j++) {
            cluster_buffer_t line_clusters;
            line_clusters.len = 0;
            line_clusters.buffer = NULL;
            // Second filtering - Find clusters of points which make up line segments
            IEPF(clusters.buffer[j], &line_clusters, 1.0);
            //for (int k=0; k<line_clusters.len; k++) {
              //NRF_LOG_INFO("Points in line segment %d: %d", k, line_clusters.buffer[k].len);
            //}
            
            
            // Third filtering - Least-square line fitting using point clusters found in previous step
            // Line model: ax + by + c = 0
            for (int k=0; k<line_clusters.len; k++) {
              if (line_clusters.buffer[k].len >= 2) {
                float sum_x = 0;
                float sum_yy = 0;
                float sum_y = 0;
                float sum_xy = 0;
                float sum_xx = 0;
                for (int l=0; l<line_clusters.buffer[k].len; l++) {
                  point_t point = line_clusters.buffer[k].buffer[l];
                  sum_x += point.x;
                  sum_y += point.y;
                  sum_xx += point.x * point.x;
                  sum_yy += point.y*point.y;
                  sum_xy += point.x * point.y;
                }
                float a_hat = sum_x*sum_yy - sum_y*sum_xy;
                float b_hat = sum_y*sum_xx - sum_x*sum_xy;
                float c_hat = sum_xy*sum_xy - sum_xx*sum_yy;
                if (b_hat == 0) {
                  a_hat = FLT_MAX;
                  b_hat = FLT_MAX;
                  b_hat = 1.0;
                }
                point_t p = {
                  .x = line_clusters.buffer[k].buffer[0].x,
                  .y = -(a_hat / b_hat)*(line_clusters.buffer[k].buffer[0].x) - (c_hat / b_hat)
                };
                point_t q = {
                  .x = line_clusters.buffer[k].buffer[line_clusters.buffer[k].len-1].x,
                  .y = -(a_hat / b_hat)*(line_clusters.buffer[k].buffer[line_clusters.buffer[k].len-1].x) - (c_hat / b_hat)
                };
                line_t line = {.P = p, .Q = q};
                point_t proj_point_start = get_projected_point_on_line(line, line_clusters.buffer[k].buffer[0]);
                point_t proj_point_end = get_projected_point_on_line(line, line_clusters.buffer[k].buffer[line_clusters.buffer[k].len-1]);
                line_t ls_fit_line = {.P = proj_point_start, .Q = proj_point_end};
                line_buffer.buffer[line_buffer.len] = ls_fit_line;
                line_buffer.len++;

                if (line_buffer.len == LB_MAX_SIZE) {
                  line_buffer.len = 0;
                }
                
                // Publish line
                if (mqttsn_client_is_connected()) {
                  //msg.startPoint = (coordinate_t) {.x = ls_fit_line.P.x, .y = ls_fit_line.P.y};
                  //msg.endPoint = (coordinate_t) {.x = ls_fit_line.Q.x, .y = ls_fit_line.Q.y };
                  test_x++;
                  test_y++;
                  msg.startPoint = (coordinate_t) {.x = test_x, .y = test_y};
                  msg.endPoint = (coordinate_t) {.x = test_x, .y = test_y};
                  msg.xdelta = 0;
                  msg.ydelta = 0;
                  msg.thetadelta = 0;
                  publish("v2/robot/NRF_5/line", &msg, sizeof(mqttsn_line_msg_t), 0, 0);
                }

              }
            
            }


            // TODO: Fourth filtering - Constrained Hough Transform

            // TODO: Append lines extracted from one ir sensor to common buffer

            // TODO: Merge lines together weighted by their covariances.

            deallocate_cluster_buffer(line_clusters);
          
          }

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