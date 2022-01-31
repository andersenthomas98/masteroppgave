
#include "mqttsn_client.h"
#include "example_task.h"
#include "thread_mqttsn.h"
#include "FreeRTOS.h"

void example_task(void *arg) {
  
  TickType_t lastWakeTime;
  const TickType_t delay = 5;
  uint16_t             m_msg_id           = 0;                         /**< Message ID thrown with MQTTSN_EVENT_TIMEOUT. */
  char                 m_topic_name[]     = "test"; /**< Name of the topic corresponding to subscriber's BSP_LED_2. */
  mqttsn_topic_t       m_topic            =                            /**< Topic corresponding to subscriber's BSP_LED_2. */
  {
      .p_topic_name = (unsigned char *)m_topic_name,
      .topic_id     = 0,
  };


  while(1) {
      
      uint8_t payload = 0;
      publish(m_topic.topic_id, payload, m_msg_id);
      payload++;

      lastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }



  
}