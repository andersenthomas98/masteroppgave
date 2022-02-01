
#include "mqttsn_client.h"
#include "example_task.h"
#include "thread_mqttsn.h"
#include "FreeRTOS.h"

void example_task(void *arg) {
  
  TickType_t lastWakeTime;
  const TickType_t delay = 5;
  uint16_t             m_msg_id           = 0;                         /**< Message ID thrown with MQTTSN_EVENT_TIMEOUT. */
  char                 m_topic_name[]     = "nRF52840_resources/led3"; /**< Name of the topic corresponding to subscriber's BSP_LED_2. */
  mqttsn_topic_t       m_topic            =                            /**< Topic corresponding to subscriber's BSP_LED_2. */
  {
      .p_topic_name = (unsigned char *)m_topic_name,
      .topic_id     = 1,
  };
  
  mqttsn_msg_queue_element_t msg;
  msg.msg_id = m_msg_id;
  msg.topic = m_topic;
  msg.payload = 1;
  msg.payload_size = 1;

  mqttsn_register_topic_queue_element_t reg;
  reg.msg_id = 0;
  reg.topic = m_topic;

  //register_topic(reg);

  
  while(1) {
      
      
      publish(msg);

      lastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }



  
}