
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
#include "line_merging.h"
#include "float.h"

#define UNDEFINED -1

extern QueueHandle_t ir_measurement_queue;
extern TaskHandle_t mapping_task_handle;

int sgn(float val) {
  if (val > 0) {
    return 1;
  }
  if (val < 0) {
    return -1;
  }
  return 0;
}

void update_point_buffers(point_buffer_t* point_buffers, ir_measurement_t measurement) {

  /*Convert range-bearing measurement to obstacle position in global reference frame and update point buffers with obstacle positions */
  float theta = measurement.servo_angle*DEG2RAD + M_PI + measurement.theta;
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

/*void update_map(map_t* map, line_segment_t line) {
  if (map->len == 0) {
    map_line_segment_t map_line = (map_line_segment_t) {
      .r = line.r,
      .theta = line.theta, 
      .start = line.start, 
      .end = line.end, 
      .sigma_r2 = line.sigma_r2, 
      .sigma_rtheta = line.sigma_rtheta, 
      .sigma_theta2 = line.sigma_theta2,
      .id = 0;
    };
    map->buffer[0] = map_line;
    map->len++;
    return;
  }

  for (int i=0; i<map->len; i++) {
    map_line_segment_t candidate_line_segment = (map_line_segment_t) {
      .r = line.r, 
      .theta = line.theta, 
      .start = line.start, 
      .end = line.end, 
      .sigma_r2 = line.sigma_r2, 
      .sigma_rtheta = line.sigma_rtheta, 
      .sigma_theta2 = line.sigma_theta2
    };
    if (is_map_line_mergeable(map->buffer[i], candidate_line_segment)) {
      
    
    }
  
  }

}*/


void mapping_task(void *arg) {

  TickType_t lastWakeTime;
  const TickType_t delay = 0.1;
  
  point_buffer_t point_buffers[NUM_DIST_SENSORS];
  point_buffer_common_t point_buffer_common;
  point_buffer_common.len = 0;

  static volatile line_segment_buffer_t line_buffer;
  line_buffer.len = 0;

  static volatile map_t map;
  map.len = 0;

  for (int i=0; i<NUM_DIST_SENSORS; i++) {
    point_buffers[i].len = 0;
  }

  mqttsn_line_msg_t line_msg;
  line_msg.identifier = LINE_IDENTIFIER;

  mqttsn_update_msg_t update_msg;
  update_msg.identifier = UPDATE_IDENTIFIER;

  float lastPublishedX = 0;
  float lastPublishedY = 0;
  float lastPublishedTheta = 0;
  
  mqttsn_cluster_msg_t dbscan_msg;
  mqttsn_cluster_msg_t iepf_msg;

  ir_measurement_t new_measurement;

  while(!mqttsn_client_is_connected()) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*5);
  }


  // Block until sensor tower task has initialized ir measurement queue
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  NRF_LOG_INFO("mapping task initialized");

  

  while(1) {
      
      
      /* Receive ir sensor measurement + robot pose from sensor tower task */
      if (xQueueReceive(ir_measurement_queue, &(new_measurement), (TickType_t) 10) == pdPASS) {
          update_point_buffers(&point_buffers, new_measurement);

          line_msg.xdelta = 100*(new_measurement.x - lastPublishedX); // cm
          line_msg.ydelta = 100*(new_measurement.y - lastPublishedY); // cm
          line_msg.thetadelta = (new_measurement.theta - lastPublishedTheta)*RAD2DEG;
          lastPublishedX = new_measurement.x;
          lastPublishedY = new_measurement.y;
          lastPublishedTheta = new_measurement.theta;

          /*update_msg.xdelta = new_measurement.x;
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
          publish_update("v2/robot/NRF_5/adv", update_msg, sizeof(update_msg), 0, 0);*/


      }
      
      /* Start line extraction */
      if (ulTaskNotifyTake(pdTRUE, (TickType_t) 0) == pdPASS) {
        
        NRF_LOG_INFO("Start line extraction");

        /*for (int i=0; i<NUM_DIST_SENSORS; i++) {
          for (int j=0; j<point_buffers[i].len; j++) {
            mqttsn_cluster_msg_t msg;
            msg.cluster_id = i;
            msg.point.x = point_buffers[i].buffer[j].x;
            msg.point.y = point_buffers[i].buffer[j].y;
            publish_cluster_point("v2/robot/NRF_5/point", msg, sizeof(mqttsn_cluster_msg_t), 0, 0);
            TickType_t lastWakeTime = xTaskGetTickCount();
            vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
          }
          
        
        }*/

        int freeHeap = xPortGetFreeHeapSize();
        NRF_LOG_INFO("Free heap before line extraction: %d", freeHeap);

        //int dbscan_msg_cluster_id = 0;
        //int iepf_msg_cluster_id = 0;
        //int mse_cluster_id = 0;
        for (int i=0; i<NUM_DIST_SENSORS; i++) {
          //NRF_LOG_INFO("DBSCAN #%d", i);
          
          // First filtering - Find points near each other
          cluster_buffer_t clusters = DBSCAN(&point_buffers[i], euclidean_distance, 20, 5); // Allocates memory on heap // 20, 5
          /*for (int j = 0; j < clusters.len; j++) {
            for (int k=0; k < clusters.buffer[j].len; k++) {
              dbscan_msg.cluster_id = dbscan_msg_cluster_id;
              dbscan_msg.point = (coordinate_t) {.x = clusters.buffer[j].buffer[k].x, .y = clusters.buffer[j].buffer[k].y};
              publish_cluster_point("v2/robot/NRF_5/DBSCAN", dbscan_msg, sizeof(dbscan_msg), 0, 0);
              TickType_t lastWakeTime = xTaskGetTickCount();
              vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
            }
            dbscan_msg_cluster_id++;
          
          }*/
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after DBSCAN: %d", freeHeap);

          if (clusters.len <= 0) {
            continue; // Go to next ir sensor readings
          }

          point_buffers[i].len = 0;
        
          // Second filtering - Find clusters of points which make up line segments
          for (int j=0; j<clusters.len; j++) {
            cluster_buffer_t line_clusters;
            line_clusters.len = 0;
            line_clusters.buffer = NULL;
            if (clusters.buffer[j].len < 2) {
              continue;
            }
          
            // No need for cluster means, use the DBSCAN clusters directly
            IEPF(clusters.buffer[j], &line_clusters, 10); // Allocates memory on heap

            /*for (int k=0; k<line_clusters.len; k++) {
              for (int l=0; l<line_clusters.buffer[k].len; l++) {
                // Publish IEPF-based clustered points
                iepf_msg.cluster_id = iepf_msg_cluster_id;
                iepf_msg.point = (coordinate_t) {.x = line_clusters.buffer[k].buffer[l].x, .y = line_clusters.buffer[k].buffer[l].y};
                publish_cluster_point("v2/robot/NRF_5/IEPF", iepf_msg, sizeof(iepf_msg), 0, 0);
                TickType_t lastWakeTime = xTaskGetTickCount();
                vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
              }
              iepf_msg_cluster_id++;
            
            }*/
            
            // Third filtering - Least-square line fitting using line clusters found in previous step
            for (int k=0; k<line_clusters.len; k++) {
              if (line_clusters.buffer[k].len >= 2 && line_buffer.len < LB_MAX_SIZE) {
                line_segment_t fitted_line = MSE_line_fit(line_clusters.buffer[k]);
                line_buffer.buffer[line_buffer.len].points = line_clusters.buffer[k];
                
                /*for (int l=0; l<line_buffer.buffer[line_buffer.len].points.len; l++) {
                  mqttsn_cluster_msg_t mse_point;
                  mse_point.cluster_id = mse_cluster_id;
                  mse_point.point.x = line_buffer.buffer[line_buffer.len].points.buffer[l].x;
                  mse_point.point.y = line_buffer.buffer[line_buffer.len].points.buffer[l].y;
                  publish_cluster_point("v2/robot/NRF_5/mse_point", mse_point, sizeof(mqttsn_cluster_msg_t), 0, 0);
                  TickType_t lastWakeTime = xTaskGetTickCount();
                  vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
                }*/

                line_buffer.buffer[line_buffer.len].start = fitted_line.start;
                line_buffer.buffer[line_buffer.len].end = fitted_line.end;
                line_buffer.buffer[line_buffer.len].r = fitted_line.r;
                line_buffer.buffer[line_buffer.len].theta = fitted_line.theta;
                line_buffer.len++;

                /*msg.startPoint = (coordinate_t) {.x = fitted_line.start.x, .y = fitted_line.start.y};
                msg.endPoint = (coordinate_t) {.x = fitted_line.end.x, .y = fitted_line.end.y};
                msg.sigma_r2 = 0;
                msg.sigma_theta2 = 0;
                msg.sigma_rtheta = 0;
                publish_line("v2/robot/NRF_5/MSE", msg, sizeof(mqttsn_line_msg_t), 0, 0);
                TickType_t lastWakeTime = xTaskGetTickCount();
                vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);*/


              }
              //mse_cluster_id++;
        
            }
            vPortFree(line_clusters.buffer);
            line_clusters.len = 0;
            freeHeap = xPortGetFreeHeapSize();
            NRF_LOG_INFO("Free heap after deallocating line clusters: %d", freeHeap);

          }

          vPortFree(clusters.buffer);
          clusters.len = 0;
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after deallocating DBSCAN clusters: %d", freeHeap);
          
        }

        // Recursively merge lines of line_buffer
        merge_linebuffer(&line_buffer, 45*DEG2RAD, 60);

        for (int i=0; i<line_buffer.len; i++) {
          if (line_buffer.buffer[i].points.len < 10 || get_length((line_t){.P = line_buffer.buffer[i].start, .Q = line_buffer.buffer[i].end}) < 50) {
            // Remove line segment from buffer
            vPortFree(line_buffer.buffer[i].points.buffer);
            line_buffer.buffer[i].points.len = 0;

            for (int j=i; j<line_buffer.len-1; j++) {
              line_buffer.buffer[j] = line_buffer.buffer[j+1];
            
            }
            line_buffer.len -= 1;
          
          }
        }

        for (int i=0; i<line_buffer.len; i++) {
          // Estimate line uncertainty
          float sigma_rho2 = 0;
          uint16_t N = line_buffer.buffer[i].points.len;
          for (int j=0; j<N; j++) {
            float x = line_buffer.buffer[i].points.buffer[j].x;
            float y = line_buffer.buffer[i].points.buffer[j].y;
            float s = line_buffer.buffer[i].points.buffer[j].label;
            float rho = x*cos(line_buffer.buffer[i].theta) + y*sin(line_buffer.buffer[i].theta) - line_buffer.buffer[i].r;
            sigma_rho2 += s * rho*rho;
          }

          float L = get_length((line_t){.P = line_buffer.buffer[i].start, .Q = line_buffer.buffer[i].end});
          float theta_N = atan2(line_buffer.buffer[i].end.y, line_buffer.buffer[i].end.x);
          float theta_1 = atan2(line_buffer.buffer[i].start.y, line_buffer.buffer[i].start.x);
          float x_off = (line_buffer.buffer[i].r / 2.0) * (tan(line_buffer.buffer[i].theta - theta_N) + tan(line_buffer.buffer[i].theta - theta_1)); 
          float sigma_r2 = 12*sigma_rho2*x_off*x_off / ((float)(L*L * N)) + 1.0 / ((float)N);
          float sigma_theta2 = (12*sigma_rho2 / ((float)(L*L * N))) * ((float)N - 1.0) / ((float)N + 1.0);
          float sigma_rtheta = -12*sigma_rho2*x_off / ((float)(L*L * N));
          line_buffer.buffer[i].sigma_r2 = sigma_r2;
          line_buffer.buffer[i].sigma_theta2 = sigma_theta2;
          line_buffer.buffer[i].sigma_rtheta = sigma_rtheta;

          // Add line segment to map
          //update_map(&map, line_buffer.buffer[i])
        }

        NRF_LOG_INFO("# lines extracted: %d", line_buffer.len);
        for (int i=0; i<line_buffer.len; i++) {
          line_msg.startPoint = (coordinate_t) {.x = line_buffer.buffer[i].start.x, .y = line_buffer.buffer[i].start.y};
          line_msg.endPoint = (coordinate_t) {.x = line_buffer.buffer[i].end.x, .y = line_buffer.buffer[i].end.y };
          //msg.sigma_r2 = line_buffer.buffer[i].sigma_r2;
          //msg.sigma_theta2 = line_buffer.buffer[i].sigma_theta2;
          //msg.sigma_rtheta = line_buffer.buffer[i].sigma_rtheta;
          publish_line("v2/robot/NRF_5/line", line_msg, sizeof(mqttsn_line_msg_t), 0, 0);
          TickType_t lastWakeTime = xTaskGetTickCount();
          vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);

          vPortFree(line_buffer.buffer[i].points.buffer);
          line_buffer.buffer[i].points.len = 0;
        
        }
        

        // Clear line buffer
        line_buffer.len = 0;



        
        freeHeap = xPortGetFreeHeapSize();
        NRF_LOG_INFO("Free heap after: %d", freeHeap);

        
      } // ulTaskNotify [end]

      //lastWakeTime = xTaskGetTickCount();
      //vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}