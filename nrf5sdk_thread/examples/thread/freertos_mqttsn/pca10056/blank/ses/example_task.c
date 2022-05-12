
#include "mqttsn_client.h"
#include "example_task.h"
#include "thread_mqttsn.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "nrf_log.h"
#include <string.h>


void example_task(void *arg) {

  TickType_t lastWakeTime;
  const TickType_t delay = 5;

  int num_pub_lines = 0;

  /*mqttsn_line_msg_t line_msg;
  line_msg.identifier = LINE_IDENTIFIER;
  line_msg.xdelta = 0;
  line_msg.ydelta = 0;
  line_msg.thetadelta = 0;
  line_msg.startPoint = (coordinate_t) {.x = -500, .y = 500};
  line_msg.endPoint = (coordinate_t) {.x = 500, .y = 500};*/

  while(!mqttsn_client_is_connected()) {
    NRF_LOG_INFO("example task is waiting for mqttsn client to connect...")
    lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }

  while(num_pub_lines < 1000) {
    //NRF_LOG_INFO("Publish (%d, %d) --- (%d, %d)", line_msg.startPoint.x, line_msg.startPoint.y, line_msg.endPoint.x, line_msg.endPoint.y);
   // publish_line("v2/robot/NRF_5/line", line_msg, sizeof(mqttsn_line_msg_t), 1, 0);
    num_pub_lines++;
    const TickType_t xDelay = 0.1*configTICK_RATE_HZ;
    vTaskDelay(xDelay);
  
  }
  
  while(1) {
    NRF_LOG_INFO("Published %d lines", num_pub_lines);
    const TickType_t xDelay = 10*configTICK_RATE_HZ;
    vTaskDelay(xDelay);
  }
  
}

void example_task_B(void *arg) {
  
  TickType_t lastWakeTime, tickCount;
  const TickType_t delay = 5;


  mqttsn_line_msg_t rx_msg;
  volatile int num_rx_msg = 0;

  while(!mqttsn_client_is_connected()) {
    lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }

  while(1) {


      if (xQueueReceive(get_queue_handle("v2/robot/NRF_5/line"), &rx_msg, 0) == pdPASS) {
        //NRF_LOG_INFO("Example task B received publish with identifier: 0x%X", rx_msg.identifier);
        //NRF_LOG_INFO("dx: %d", rx_msg.xdelta);
        //NRF_LOG_INFO("dy: %d", rx_msg.ydelta);
        //NRF_LOG_INFO("dtheta: %d", rx_msg.thetadelta);
        NRF_LOG_INFO("line: (%d, %d), (%d, %d) ", rx_msg.startPoint.x, rx_msg.startPoint.y, rx_msg.endPoint.x, rx_msg.endPoint.y);
        num_rx_msg += 1;
      }

      const TickType_t xDelay = 0.1*configTICK_RATE_HZ;
      vTaskDelay(xDelay);
  
     

  }
  
}

void example_task_C(void *arg) {
  
  TickType_t lastWakeTime;
  const TickType_t delay = 5;

  int num_pub_points = 0;

  mqttsn_update_msg_t update_msg;
  update_msg.identifier = UPDATE_IDENTIFIER;
  update_msg.xdelta = 0;
  update_msg.ydelta = 0;
  update_msg.thetadelta = 0;
  update_msg.ir1 = (coordinate_t) {.x = 100, .y = 100};
  update_msg.ir2 = (coordinate_t) {.x = 100, .y = 100};
  update_msg.ir3 = (coordinate_t) {.x = 100, .y = 100};
  update_msg.ir4 = (coordinate_t) {.x = 100, .y = 100};
  update_msg.valid = 0b00001111;


  while(!mqttsn_client_is_connected()) {
    NRF_LOG_INFO("example task is waiting for mqttsn client to connect...")
    lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }

  while(num_pub_points < 1000) {
    publish_update("v2/robot/NRF_5/point", update_msg, sizeof(mqttsn_update_msg_t), 0, 0);
    num_pub_points++;
    const TickType_t xDelay = 0.1*configTICK_RATE_HZ;
    vTaskDelay(xDelay);
  }
  
  while(1) {
    NRF_LOG_INFO("Published %d points", num_pub_points);
    const TickType_t xDelay = 10*configTICK_RATE_HZ;
    vTaskDelay(xDelay);
  
  }
  
}

