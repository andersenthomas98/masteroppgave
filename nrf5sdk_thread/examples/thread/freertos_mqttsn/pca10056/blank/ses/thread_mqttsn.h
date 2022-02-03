/************************************************************************/
// File:            thread_mqttsn.h                                     //
// Author:                                                              //
// Purpose:         Implements a multithreaded mqttsn client.           //
//                                                                      //
/************************************************************************/


#include <stdint.h>
#include "mqttsn_client.h"
#define NUM_TOPICS 1

typedef struct mqttsn_msg_queue_element {
  mqttsn_topic_t topic;
  uint16_t msg_id;
  uint16_t payload_size;
  void* payload;
} mqttsn_msg_queue_element_t;


void thread_stack_task(void * arg);

void mqttsn_task(void * arg);

void publish(mqttsn_msg_queue_element_t msg);
