
#include "mqttsn_client.h"
#include "example_task.h"
#include "thread_mqttsn.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "nrf_log.h"
#include <string.h>


typedef struct example_rx_msg {
  uint8_t identifier;
  uint16_t data;
} __attribute__((packed)) example_rx_msg_t;

void example_task(void *arg) {


  NRF_LOG_INFO("example task setup");
  TickType_t lastWakeTime;
  const TickType_t delay = 5;

  uint8_t payload = 0x01;
  mqttsn_target_msg_t rx_msg;
  
  mqttsn_update_msg_t msg;
  msg.identifier = UPDATE_IDENTIFIER;
  msg.xdelta = 0;
  msg.ydelta = 0;
  msg.thetadelta = 0;
  msg.ir1 = (coordinate_t) {.x = 400, .y = 400};
  msg.ir2 = (coordinate_t) {.x = 400, .y = 400};
  msg.ir3 = (coordinate_t) {.x = 400, .y = 400};
  msg.ir4 = (coordinate_t) {.x = 400, .y = 400};
  msg.valid = 0x0F;

  mqttsn_line_msg_t line_msg;
  line_msg.identifier = LINE_IDENTIFIER;
  line_msg.xdelta = 0;
  line_msg.ydelta = 0;
  line_msg.thetadelta = 0;
  line_msg.startPoint = (coordinate_t) {.x = -500, .y = 500};
  line_msg.endPoint = (coordinate_t) {.x = 500, .y = 500};

  mqttsn_line_msg_t line_msg_list[4];
  line_msg_list[0] = line_msg;
  
  line_msg.startPoint = (coordinate_t) {.x = 500, .y = 500};
  line_msg.endPoint = (coordinate_t) {.x = 500, .y = -500};
  line_msg_list[1] = line_msg;

  line_msg.startPoint = (coordinate_t) {.x = 500, .y = -500};
  line_msg.endPoint = (coordinate_t) {.x = -500, .y = -500};
  line_msg_list[2] = line_msg;

  while(!mqttsn_client_is_connected()) {
    NRF_LOG_INFO("example task is waiting for mqttsn client to connect...")
    lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ);
  }

  while(1) {
    
    for (int i=0; i<3; i++) {
      mqttsn_line_msg_t line = line_msg_list[i];
      NRF_LOG_INFO("Publish (%d, %d) --- (%d, %d)", line.startPoint.x, line.startPoint.y, line.endPoint.x, line.endPoint.y);
      publish_line("v2/robot/NRF_5/line", line, sizeof(line), 0, 0);
      
      //lastWakeTime = xTaskGetTickCount();
      //vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
    
    }
    
    NRF_LOG_INFO("example task loop");
    lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}

void example_task_B(void *arg) {
  
  TickType_t lastWakeTime, tickCount;
  const TickType_t delay = 5;


  mqttsn_init_msg_t rx_msg;
  long i = 0;

  while(!mqttsn_client_is_connected()) {
    lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }

  NRF_LOG_INFO("Example task B initialized");

  while(1) {

      /*QueueHandle_t queue = get_queue_handle("v2/server/NRF_5/init");

      if (mqttsn_client_is_connected() && xQueueReceive(queue, &rx_msg, 0) == pdPASS) {
        NRF_LOG_INFO("Example task B received publish with identifier: 0x%X", rx_msg.identifier);
        NRF_LOG_INFO("Init x: %d", rx_msg.init_x);
        NRF_LOG_INFO("Init y: %d", rx_msg.init_y);
        NRF_LOG_INFO("Init theta: %d", rx_msg.init_theta);
      }

      lastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);*/
     

     // Do nothing

  }
  
}

