#define PB_MAX_SIZE 100
#define LB_MAX_SIZE 100

#include <stdint.h>

typedef struct point {
  float x;
  float y;
  int16_t label;
} point_t;

typedef struct polar {
  float r;
  float theta;
} polar_t;

typedef struct line {
  point_t P;
  point_t Q;
} line_t;

typedef struct line_buffer {
  uint8_t len;
  line_t buffer[LB_MAX_SIZE];
} line_buffer_t;

typedef struct point_buffer {
  uint8_t len;
  point_t buffer[PB_MAX_SIZE];
} point_buffer_t;

typedef struct point_buffer_common {
  uint16_t len;
  point_t buffer[PB_MAX_SIZE*4];
} point_buffer_common_t;


typedef struct point_reference_buffer {
  uint8_t len;
  point_t* buffer[PB_MAX_SIZE];
} point_reference_buffer_t;

typedef struct point_buffer_dynamic {
  uint8_t len;
  point_t* buffer;
} point_buffer_dynamic_t;

typedef struct cluster_buffer {
  uint8_t len;
  point_buffer_dynamic_t* buffer;
} cluster_buffer_t;

typedef struct line_segment {
  float r;
  float theta;
  point_t start;
  point_t end;
  float sigma_r2;
  float sigma_theta2;
  float sigma_rtheta;
  point_buffer_dynamic_t points;
} line_segment_t;

typedef  struct line_segment_buffer {
  uint8_t len;
  line_segment_t buffer[LB_MAX_SIZE];
} line_segment_buffer_t;