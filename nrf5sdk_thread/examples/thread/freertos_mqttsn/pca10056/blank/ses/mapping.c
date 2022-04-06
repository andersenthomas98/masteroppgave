
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

void copy_points_to_line_segment(line_segment_t* line, point_buffer_dynamic_t points) {
  line->points.buffer = pvPortMalloc(sizeof(point_t)*points.len);
  line->points.len = points.len;
  
  for (int i=0; i<points.len; i++) {
    line->points.buffer[i] = points.buffer[i];
  }
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

  line_segment_buffer_t line_buffer;
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

  /*while(1) {
    // Testing
    
    int freeHeap = xPortGetFreeHeapSize();
    NRF_LOG_INFO("Free heap before: %d", freeHeap);

    point_buffer_dynamic_t pb1;
    pb1.len = 2;
    pb1.buffer = pvPortMalloc(sizeof(point_t)*pb1.len);
    pb1.buffer[0] = (point_t){.x = -1, .y = 0, .label = 1};
    pb1.buffer[1] = (point_t){.x = 0, .y = 0.5, .label = 1};

    point_buffer_dynamic_t pb2;
    pb2.len = 2;
    pb2.buffer = pvPortMalloc(sizeof(point_t)*pb1.len);
    pb2.buffer[0] = (point_t){.x = 0, .y = 0.5, .label = 1};
    pb2.buffer[1] = (point_t){.x = 1, .y = 0, .label = 1};

    point_buffer_dynamic_t pb3;
    pb3.len = 2;
    pb3.buffer = pvPortMalloc(sizeof(point_t)*pb1.len);
    pb3.buffer[0] = (point_t){.x = 1, .y = 0.25, .label = 1};
    pb3.buffer[1] = (point_t){.x = 2, .y = 0.25, .label = 1};

    point_buffer_dynamic_t pb4;
    pb4.len = 2;
    pb4.buffer = pvPortMalloc(sizeof(point_t)*pb1.len);
    pb4.buffer[0] = (point_t){.x = 2, .y = 0.25, .label = 1};
    pb4.buffer[1] = (point_t){.x = 2, .y = -1, .label = 1};

    line_segment_t l1 = MSE_line_fit(pb1);
    l1.points = pb1;
    line_segment_t l2 = MSE_line_fit(pb2);
    l2.points = pb2;
    line_segment_t l3 = MSE_line_fit(pb3);
    l3.points = pb3;
    line_segment_t l4 = MSE_line_fit(pb4);
    l4.points = pb4;

    line_segment_buffer_t lb;
    lb.len = 4;
    lb.buffer[0] = l1;
    lb.buffer[1] = l2;
    lb.buffer[2] = l3;
    lb.buffer[3] = l4;

    merge_linebuffer(&lb, 55*DEG2RAD, 10);

    
    for (int i=0; i<lb.len; i++) {
      vPortFree(lb.buffer[i].points.buffer);
      lb.buffer[i].points.len = 0;
    }

    freeHeap = xPortGetFreeHeapSize();
    NRF_LOG_INFO("Free heap after: %d", freeHeap);

    vTaskSuspend(NULL);
  } */


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

            cluster_means.buffer[j] = (point_t){.x = mean_x, .y = mean_y, .label = 1/*clusters.buffer[j].len*/};
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
          
          IEPF(cluster_means, &line_clusters, 10); // Allocates memory on heap
            
          // Third filtering - Least-square line fitting using line clusters found in previous step
          // Line model: ax + by + c = 0
          for (int k=0; k<line_clusters.len; k++) {
            if (line_clusters.buffer[k].len >= 2 && line_buffer.len < LB_MAX_SIZE) {
              line_segment_t fitted_line = MSE_line_fit(line_clusters.buffer[k]);
              copy_points_to_line_segment(&fitted_line, line_clusters.buffer[k]);
              line_buffer.buffer[line_buffer.len] = fitted_line;
              line_buffer.len++;

            }
        
          }



          deallocate_cluster_buffer(line_clusters); // Free heap memory
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after deallocating line clusters: %d", freeHeap);
          vPortFree(cluster_means.buffer);
          cluster_means.len = 0;
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after deallocating cluster_means: %d", freeHeap);
          
        }


        // Merge lines of line_buffer

        merge_linebuffer(&line_buffer, 45*DEG2RAD, 60);

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
        }


        for (int i=0; i<line_buffer.len; i++) {
          msg.startPoint = (coordinate_t) {.x = line_buffer.buffer[i].start.x, .y = line_buffer.buffer[i].start.y};
          msg.endPoint = (coordinate_t) {.x = line_buffer.buffer[i].end.x, .y = line_buffer.buffer[i].end.y };
          msg.sigma_r2 = line_buffer.buffer[i].sigma_r2;
          msg.sigma_theta2 = line_buffer.buffer[i].sigma_theta2;
          msg.sigma_rtheta = line_buffer.buffer[i].sigma_rtheta;
          publish_line("v2/robot/NRF_5/line", msg, sizeof(mqttsn_line_msg_t), 0, 0);
          TickType_t lastWakeTime = xTaskGetTickCount();
          vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);

          vPortFree(line_buffer.buffer[i].points.buffer);
          line_buffer.buffer[i].points.len = 0;
        
        }




        
        freeHeap = xPortGetFreeHeapSize();
        NRF_LOG_INFO("Free heap after: %d", freeHeap);

        vTaskSuspend(NULL);
        
      } // ulTaskNotify [end]

      //lastWakeTime = xTaskGetTickCount();
      //vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}