/************************************************************************/
// File:            thread_mqttsn.h                                     //
// Author:                                                              //
// Purpose:         Implements a multithreaded mqttsn client.           //
//                                                                      //
/************************************************************************/


#include <stdint.h>
#include "mqttsn_client.h"
#include "FreeRTOS.h"
#include "queue.h"

#define SCAN_BORDER_IDENTIFIER          0x01
#define UPDATE_IDENTIFIER               0x02
#define COLLISION_DETECTED_IDENTIFIER   0x03


typedef struct coordinate {
  uint16_t x;
  uint16_t y;
} __attribute__((packed)) coordinate_t; // __attribute__((packed)) to explicitly tell the compiler to not add any padding

typedef struct mqttsn_update_msg { // 24 bytes
  uint8_t identifier;
  int16_t xdelta;     // Change in x-position relative to global frame [mm]
  int16_t ydelta;     // Change in y-position relative to global frame [mm]
  int16_t thetadelta; // Change in heading relative to global frame [deg]
  coordinate_t ir1;   // Coordinates of obstacle detected by ir sensor 1 [mm]
  coordinate_t ir2;   // Coordinates of obstacle detected by ir sensor 2 [mm]
  coordinate_t ir3;   // Coordinates of obstacle detected by ir sensor 3 [mm]
  coordinate_t ir4;   // Coordinates of obstacle detected by ir sensor 4 [mm]
  uint8_t valid;       // 4 LSBs signify if corresponding ir sensor is a valid detection
} __attribute__((packed)) mqttsn_update_msg_t; // __attribute__((packed)) to explicitly tell the compiler to not add any padding


typedef struct mqttsn_msg_queue_element {
  uint16_t topic_id;
  uint16_t msg_id;
  uint16_t payload_size;
  uint8_t qos;
  void* payload;
} mqttsn_msg_queue_element_t;


void thread_stack_task(void * arg);

void mqttsn_task(void * arg);

int publish(char* topic_name, void* p_payload, uint8_t payload_size, uint8_t qos, uint16_t msg_id);

int publish_fromISR(char* topic_name, void* p_payload, uint8_t payload_size, uint8_t qos, uint16_t msg_id);

int publish_scan_border(char* topic_name);

QueueHandle_t get_queue_handle(char* topic_name);
