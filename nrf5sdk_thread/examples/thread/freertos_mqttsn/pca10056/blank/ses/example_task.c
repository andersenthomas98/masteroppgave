
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
  uint16_t             m_msg_id           = 0;                         /**< Message ID thrown with MQTTSN_EVENT_TIMEOUT. */
  char                 m_topic_name[]     = "v2/robot/NRF_5/adv"; /**< Name of the topic corresponding to subscriber's BSP_LED_2. */
  mqttsn_topic_t       m_topic            =                            /**< Topic corresponding to subscriber's BSP_LED_2. */
  {
      .p_topic_name = (unsigned char *)m_topic_name,
      .topic_id     = NULL,
  };

  uint8_t payload = 0x01;
  mqttsn_msg_queue_element_t msg;
  msg.msg_id = m_msg_id;
  msg.topic = m_topic;
  msg.payload = &payload;
  msg.payload_size = sizeof(uint8_t);
  NRF_LOG_INFO("%d", msg.payload_size);

  char* rx_msg;
  
  while(1) {
      
      NRF_LOG_INFO("Publish from task A");
      publish(msg);

      /*if (xQueueReceive(get_queue_handle("tester"), &rx_msg, 0) == pdPASS) {
        NRF_LOG_INFO("Example task received from test topic: %s", NRF_LOG_PUSH(rx_msg));
      }*/

      lastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}

void example_task_B(void *arg) {
  
  TickType_t lastWakeTime;
  const TickType_t delay = 5;
  uint16_t             m_msg_id           = 0;                         /**< Message ID thrown with MQTTSN_EVENT_TIMEOUT. */
  char                 m_topic_name[]     = "nRF52840_resources/led1"; /**< Name of the topic corresponding to subscriber's BSP_LED_2. */
  mqttsn_topic_t       m_topic            =                            /**< Topic corresponding to subscriber's BSP_LED_2. */
  {
      .p_topic_name = (unsigned char *)m_topic_name,
      .topic_id     = NULL,
  };
  
  mqttsn_msg_queue_element_t msg;
  msg.msg_id = m_msg_id;
  msg.topic = m_topic;
  msg.payload = "B";
  msg.payload_size = strlen(msg.payload);
  
  while(1) {
      
      NRF_LOG_INFO("Publish from task B");
      publish(msg);

      lastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}