
#include "mqttsn_client.h"
#include "example_task.h"
#include "thread_mqttsn.h"
#include "FreeRTOS.h"
#include "nrf_log.h"
#include <string.h>

void example_task(void *arg) {
  
  TickType_t lastWakeTime;
  const TickType_t delay = 5;
  uint16_t             m_msg_id           = 0;                         /**< Message ID thrown with MQTTSN_EVENT_TIMEOUT. */
  char                 m_topic_name[]     = "nRF52840_resources/led3"; /**< Name of the topic corresponding to subscriber's BSP_LED_2. */
  mqttsn_topic_t       m_topic            =                            /**< Topic corresponding to subscriber's BSP_LED_2. */
  {
      .p_topic_name = (unsigned char *)m_topic_name,
      .topic_id     = NULL,
  };
  
  mqttsn_msg_queue_element_t msg;
  msg.msg_id = m_msg_id;
  msg.topic = m_topic;
  msg.payload = "{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 1.0, 2.0, 3.0, 4.0, 5.0}";
  msg.payload_size = strlen(msg.payload);
  
  while(1) {
      
      //NRF_LOG_INFO("Publish from task A");
      publish(msg);

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
  msg.payload = "{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 7.0, 8.0, 9.0, 10.0, 12.0, 13.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 7.0, 8.0, 9.0, 10.0}";
  msg.payload_size = strlen(msg.payload);
  
  while(1) {
      
      NRF_LOG_INFO("Publish from task B");
      publish(msg);

      lastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}