
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

  while(1) {
    publish("v2/robot/NRF_5/adv", &msg, sizeof(msg), 0, 0);
    
    NRF_LOG_INFO("example task loop");
    lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
  /*TickType_t lastWakeTime;
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

  while(1) {
      
      publish("v2/robot/NRF_5/adv", &msg, sizeof(msg), 0, 0);

      QueueHandle_t queue = get_queue_handle("v2/server/NRF_5/cmd");
     
      if (xQueueReceive(queue, &rx_msg, 0) == pdPASS) {
        NRF_LOG_INFO("Example task received publish with identifier: 0x%X", rx_msg.identifier);
        NRF_LOG_INFO("Target x: %d", rx_msg.target_x);
        NRF_LOG_INFO("Target y: %d", rx_msg.target_y);
      }

      lastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }*/
  
}

void example_task_B(void *arg) {
  
  TickType_t lastWakeTime;
  const TickType_t delay = 5;


  mqttsn_init_msg_t rx_msg;
  

  while(1) {

      QueueHandle_t queue = get_queue_handle("v2/server/NRF_5/init");

      if (mqttsn_client_is_connected() && xQueueReceive(queue, &rx_msg, 0) == pdPASS) {
        NRF_LOG_INFO("Example task B received publish with identifier: 0x%X", rx_msg.identifier);
        NRF_LOG_INFO("Init x: %d", rx_msg.init_x);
        NRF_LOG_INFO("Init y: %d", rx_msg.init_y);
        NRF_LOG_INFO("Init theta: %d", rx_msg.init_theta);
      }

      lastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}

