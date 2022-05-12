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

/* Incomming message identifiers (robot <- server) */
#define INIT_IDENTIFIER                 0x01
#define TARGET_IDENTIFIER               0x02

/* Outgoing message identifiers (robot -> server)*/
#define SCAN_BORDER_IDENTIFIER          0x01
#define UPDATE_IDENTIFIER               0x02
#define COLLISION_DETECTED_IDENTIFIER   0x03
#define LINE_IDENTIFIER                 0x04

typedef struct mqttsn_init_msg {
  uint8_t identifier;
  int16_t init_x;
  int16_t init_y;
  int16_t init_theta;
} __attribute__((packed)) mqttsn_init_msg_t;

typedef struct mqttsn_target_msg {
  uint8_t identifier;
  int16_t target_x;
  int16_t target_y;
} __attribute__((packed)) mqttsn_target_msg_t;

typedef struct mqttsn_scan_border_msg {
  uint8_t identifier;
} __attribute__((packed)) mqttsn_scan_border_msg_t;

typedef struct mqttsn_collision_detection_msg {
  uint8_t identifier;
} __attribute__((packed)) mqttsn_collision_detection_msg_t;


typedef struct coordinate {
  int16_t x; // was uint16_t
  int16_t y; // was uint16_t
} __attribute__((packed)) coordinate_t; // __attribute__((packed)) to explicitly tell the compiler to not add any padding

typedef struct mqttsn_update_msg { // 24 bytes
  uint8_t identifier;
  int16_t xdelta;      // Change in x-position relative to global frame [mm]
  int16_t ydelta;      // Change in y-position relative to global frame [mm]
  int16_t thetadelta;  // Change in heading relative to global frame [deg]
  coordinate_t ir1;    // Coordinates of obstacle detected by ir sensor 1 [mm]
  coordinate_t ir2;    // Coordinates of obstacle detected by ir sensor 2 [mm]
  coordinate_t ir3;    // Coordinates of obstacle detected by ir sensor 3 [mm]
  coordinate_t ir4;    // Coordinates of obstacle detected by ir sensor 4 [mm]
  uint8_t valid;       // 4 LSBs signify if corresponding ir sensor is a valid detection
} __attribute__((packed)) mqttsn_update_msg_t; // __attribute__((packed)) to explicitly tell the compiler to not add any padding

typedef struct mqttsn_controller_msg {
  float time;
  float x;
  float y;
  float theta;
  float left_u;
  float right_u;
} __attribute__((packed)) mqttsn_controller_msg_t;

typedef struct mqttsn_estimator_msg {
  float time;
  float x;
  float y;
  float theta;
  float enc_speed;
  float gyro;
} __attribute__((packed)) mqttsn_estimator_msg_t;

typedef struct mqttsn_line_msg {
  uint8_t identifier;      // 1 byte
  //int16_t xdelta;          // 2 bytes
  //int16_t ydelta;          // 2 bytes
  //int16_t thetadelta;      // 2 bytes
  coordinate_t startPoint; // 4 bytes
  coordinate_t endPoint;   // 4 bytes
  float sigma_r2;          // 4 bytes
  float sigma_theta2;      // 4 bytes
  float sigma_rtheta;      // 4 bytes
} __attribute__((packed)) mqttsn_line_msg_t;

typedef struct mqttsn_test_msg {
  uint8_t point_buffer;    // 1 byte
  uint8_t point_buffer_len;// 1 byte
  int16_t label;           // 2 bytes
  coordinate_t point;      // 2+2 bytes
} __attribute__((packed)) mqttsn_test_msg_t;

typedef struct mqttsn_cluster_msg_t {
  int8_t cluster_id;
  coordinate_t point;
} mqttsn_cluster_msg_t;

typedef struct mqttsn_cluster_msg_queue_element {
  uint16_t topic_id;
  uint16_t msg_id;
  uint16_t payload_size;
  uint8_t qos;
  mqttsn_cluster_msg_t payload;
} mqttsn_cluster_msg_queue_element_t;


typedef struct mqttsn_msg_queue_element {
  uint16_t topic_id;
  uint16_t msg_id;
  uint16_t payload_size;
  uint8_t qos;
  void* payload;
} mqttsn_msg_queue_element_t;

typedef struct mqttsn_line_msg_queue_element {
  uint16_t topic_id;
  uint16_t msg_id;
  uint16_t payload_size;
  uint8_t qos;
  mqttsn_line_msg_t payload;
} mqttsn_line_msg_queue_element_t;

typedef struct mqttsn_estimator_msg_queue_element {
   uint16_t topic_id;
  uint16_t msg_id;
  uint16_t payload_size;
  uint8_t qos;
  mqttsn_estimator_msg_t payload;
} mqttsn_estimator_msg_queue_element_t;

typedef struct mqttsn_update_msg_queue_element {
  uint16_t topic_id;
  uint16_t msg_id;
  uint16_t payload_size;
  uint8_t qos;
  mqttsn_update_msg_t payload;
} mqttsn_update_msg_queue_element_t;



void thread_stack_task(void * arg);

void mqttsn_task(void * arg);

uint32_t publish(char* topic_name, void* p_payload, uint8_t payload_size, uint8_t qos, uint16_t msg_id);

uint32_t publish_update(char* topic_name, mqttsn_update_msg_t payload, uint8_t payload_size, uint8_t qos, uint16_t msg_id);

uint32_t publish_line(char* topic_name, mqttsn_line_msg_t payload, uint8_t payload_size, uint8_t qos, uint16_t msg_id);

uint32_t publish_cluster_point(char* topic_name, mqttsn_cluster_msg_t payload, uint8_t payload_size, uint8_t qos, uint16_t msg_id);

uint32_t publish_estimator(char* topic_name, mqttsn_estimator_msg_t payload, uint8_t payload_size, uint8_t qos, uint16_t msg_id);

uint8_t mqttsn_client_is_connected(void);

QueueHandle_t get_queue_handle(char* topic_name);
