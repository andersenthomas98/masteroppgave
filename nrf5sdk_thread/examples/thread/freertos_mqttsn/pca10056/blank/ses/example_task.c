
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
  
  TickType_t lastWakeTime;
  const TickType_t delay = 5;

  uint8_t payload = 0x01;
  example_rx_msg_t rx_msg;
  
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
      
     // NRF_LOG_INFO("Publish from task A");
     // publish("v2/robot/NRF_5/adv", &msg, sizeof(msg), 0, 0);

      publish("v2/robot/NRF_5/adv", &payload, sizeof(payload), 0, 0);
      QueueHandle_t queue = get_queue_handle("v2/server/NRF_5/adv");
      const char* name = pcQueueGetName(queue);
      NRF_LOG_INFO("Queue name: %s", NRF_LOG_PUSH(name));
      if (xQueueReceive(queue, &rx_msg, 0) == pdPASS) {
        NRF_LOG_INFO("Example task received publish with identifier: 0x%X", rx_msg.identifier);
        NRF_LOG_INFO("Data received: %d", rx_msg.data);
      }

      lastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}
