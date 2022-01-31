#include <stdint.h>
#include "mqttsn_client.h"

typedef struct mqttsn_msg_queue_element {
  mqttsn_topic_t topic;
  uint16_t msg_id;
  uint16_t payload_size;
  void* payload;
} mqttsn_msg_queue_element_t;

typedef struct mqttsn_register_topic_queue_element {
  mqttsn_topic_t topic;
  uint16_t msg_id
} mqttsn_register_topic_queue_element_t;


mqttsn_topic_t test_topic = {
  .p_topic_name = "test",
  .topic_id     = 0
}


#define NUM_TOPICS 1
mqttsn_topic_t topics_to_register[NUM_TOPICS] = {
  test_topic
};



void thread_stack_task(void * arg);

void mqttsn_task(void * arg);

void publish(mqttsn_msg_queue_element_t msg);
