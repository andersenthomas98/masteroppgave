#include "mapping_types.h"
#include "IEPF.h"
#include "nrf_log.h"
#include "math.h"
#include <stdlib.h>

float get_length(line_t line) {
  return (sqrtf((line.P.x - line.Q.x)*(line.P.x - line.Q.x) + (line.P.y - line.Q.y)*(line.P.y - line.Q.y)));
}

float dot_product(point_t v1, point_t v2) {
  return (v1.x*v2.x + v1.y*v2.y);
}

/*void append(point_t point, point_buffer_dynamic_t* point_buffer) {
  point_buffer->len += 1;
  if (point_buffer->len == 0) {
    point_buffer->buffer = pvPortMalloc(sizeof(point_t)*point_buffer->len);
  } else {
    point_buffer_dynamic_t tmp;
    tmp.buffer
    point_buffer->buffer = nrf_realloc(point_buffer->buffer, sizeof(point_t)*point_buffer->len);
  }
  if (point_buffer->buffer == NULL) {
    NRF_LOG_ERROR("Could not allocate memory for dynamic point buffer");
    return;
  }
  point_buffer->buffer[point_buffer->len - 1] = (point_t)point;
}*/

/*
void append_cluster(point_buffer_dynamic_t point_buffer, cluster_buffer_dynamic_t* cluster_buffer) {  
  cluster_buffer->len += 1;
  if (cluster_buffer->len == 0) {
    cluster_buffer->buffer = nrf_malloc(sizeof(point_buffer)*cluster_buffer->len);
  } else {
    cluster_buffer->buffer = nrf_realloc(cluster_buffer->buffer, sizeof(point_buffer)*cluster_buffer->len);
  }
  if (cluster_buffer->buffer == NULL) {
    NRF_LOG_ERROR("Could not allocate memory fpr dynamic cluster buffer");
    return;
  }
  cluster_buffer->buffer[cluster_buffer->len - 1] = point_buffer;   
}*/


point_t get_projected_point_on_line(line_t line, point_t point) {
  point_t e1 = (point_t) {.x = line.Q.x - line.P.x, .y = line.Q.y - line.P.y};
  point_t e2 = (point_t) {.x = point.x - line.P.x, .y = point.y - line.P.y};
  float dp = dot_product(e1, e2);
  float len2 = e1.x*e1.x + e1.y*e1.y;
  point_t proj_point = (point_t) {.x = (line.P.x + (dp * e1.x) / len2), .y = (line.P.y + (dp*e1.y) / len2)}; // TODO: Check for division by zero
  return proj_point;
  
}

void IEPF(point_buffer_t input_buffer, cluster_buffer_t* p_output_buffer, float T) {
  point_t start_point = input_buffer.buffer[0];
  point_t middle_point = input_buffer.buffer[(int)(input_buffer.len / 2)];
  point_t end_point = input_buffer.buffer[input_buffer.len - 1];
  line_t split_line = (line_t) {.P = start_point, .Q = end_point};

  point_t proj_point = get_projected_point_on_line(split_line, middle_point);

  line_t middle_proj_point = {.P = middle_point, .Q = proj_point};

  float line_length = get_length(middle_proj_point);

  if (line_length < T || input_buffer.len == 2) {
    
    // Append input buffer to output cluster buffer
    // TODO: append_cluster(p_output_buffer, &input_buffer);
    
  } else {
    // Left side of split
    point_buffer_t left_points;
    left_points.len = 0;
    for (int i=0; i<(int)(input_buffer.len / 2 + 1); i++) {
      left_points.buffer[i] = input_buffer.buffer[i];
      left_points.len++;
    }
    IEPF(left_points, p_output_buffer, T);

    // Right side of split
    point_buffer_t right_points;
    right_points.len = 0;
    for (int i=(int)(input_buffer.len / 2); i < input_buffer.len; i++) {
      right_points.buffer[i-(int)(input_buffer.len / 2)] = input_buffer.buffer[i];
      right_points.len++;
    }
    IEPF(right_points, p_output_buffer, T);
    

  }

}