#define PB_MAX_SIZE 100

#include "stdint.h"

typedef struct point {
  float x;
  float y;
  int16_t label;
} point_t;

typedef struct polar {
  float r;
  float theta;
} polar_t;

typedef struct point_buffer {
  uint8_t len;
  point_t buffer[PB_MAX_SIZE]
} point_buffer_t;

typedef struct point_reference_buffer {
  uint8_t len;
  point_t* buffer[PB_MAX_SIZE]
} point_reference_buffer_t;