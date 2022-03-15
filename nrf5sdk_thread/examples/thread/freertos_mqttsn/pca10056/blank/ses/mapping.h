
#define PB_MAX_SIZE 100

typedef struct point {
  float x;
  float y;
} point_t;

typedef struct point_buffer {
  uint8_t len;
  point_t buffer[PB_MAX_SIZE]
} point_buffer_t;




uint8_t mapping_task_init_complete(void);

void mapping_task(void *arg);