#include <stdint.h>

void thread_stack_task(void * arg);

void mqttsn_task(void * arg);

void publish(uint16_t topic_id, uint8_t payload, uint16_t msg_id);