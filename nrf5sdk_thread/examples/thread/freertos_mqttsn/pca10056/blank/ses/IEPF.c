#include "mapping_types.h"
#include "mapping_utils.h"
#include "IEPF.h"
#include "nrf_log.h"
#include "math.h"
#include <stdlib.h>


void IEPF(point_buffer_dynamic_t input_buffer, cluster_buffer_t* p_output_buffer, float T) {
  point_t start_point = input_buffer.buffer[0];
  point_t middle_point = input_buffer.buffer[(int)(input_buffer.len / 2)];
  point_t end_point = input_buffer.buffer[input_buffer.len - 1];
  line_t split_line = (line_t) {.P = start_point, .Q = end_point};

  point_t proj_point = get_projected_point_on_line(split_line, middle_point);

  line_t middle_proj_point = {.P = middle_point, .Q = proj_point};

  float line_length = get_length(middle_proj_point);

  if (line_length < T || input_buffer.len == 2) {
    
    // *********** Append input buffer to output cluster buffer ***************
    if (p_output_buffer->len <= 0) {
      //NRF_LOG_INFO("Allocating (bytes %d)", sizeof(point_buffer_dynamic_t));
      p_output_buffer->buffer = pvPortMalloc(sizeof(point_buffer_dynamic_t));
      if (p_output_buffer->buffer == NULL) {
        NRF_LOG_ERROR("Failed to allocate memory for output buffer");
        return;
      }
      p_output_buffer->buffer[0] = input_buffer;
      p_output_buffer->len++;
    } else {
      // Reallocate memory for output buffer (TODO: optimize memory reallocation scheme... implement pvPortRealloc?)
      cluster_buffer_t reallocated_clusters;
      reallocated_clusters.len = p_output_buffer->len + 1;
      //NRF_LOG_INFO("Allocating (bytes %d*%d)", sizeof(point_buffer_dynamic_t), reallocated_clusters.len);
      reallocated_clusters.buffer = pvPortMalloc(sizeof(point_buffer_dynamic_t)*reallocated_clusters.len);
      if (reallocated_clusters.buffer == NULL) {
        NRF_LOG_ERROR("Failed to reallocate memory for output buffer");
        return;
      }
      // Copy over existing output buffer to new memory location
      for (int i=0; i<p_output_buffer->len; i++) {
        point_buffer_dynamic_t points;
        points.len = p_output_buffer->buffer[i].len;
        if (points.len <= 0) {
          continue;
        }
        //NRF_LOG_INFO("Allocating (bytes %d*%d)", sizeof(point_t), points.len);
        points.buffer = pvPortMalloc(sizeof(point_t)*points.len);
        if (points.buffer == NULL) {
          NRF_LOG_ERROR("Failed to reallocate memory for output buffer");
          return;
        }

        for (int j=0; j<points.len; j++) {
          point_t point = p_output_buffer->buffer[i].buffer[j];
          points.buffer[j] = point;
        }
        reallocated_clusters.buffer[i] = points;
      }

      // Free old memory location
      //NRF_LOG_INFO("Free old cluster buffer");

      deallocate_cluster_buffer(*p_output_buffer);

      // Append new cluster to reallocated cluster buffer
      reallocated_clusters.buffer[reallocated_clusters.len - 1] = input_buffer;
      *p_output_buffer = reallocated_clusters;
    }
    // *********** Append input buffer to output cluster buffer [End] ***************
    
  } else {
    // Left side of split
    point_buffer_dynamic_t left_points;
    left_points.len = (int)(input_buffer.len / 2 + 1);
    //NRF_LOG_INFO("Allocating (bytes %d*%d)", sizeof(point_t), left_points.len);
    left_points.buffer = pvPortMalloc(sizeof(point_t)*left_points.len);
    if (left_points.buffer == NULL) {
      NRF_LOG_ERROR("Failed to allocate left points buffer");
      return;
    }
    for (int i=0; i<left_points.len; i++) {
      left_points.buffer[i] = input_buffer.buffer[i];
    }
    IEPF(left_points, p_output_buffer, T);

    // Right side of split
    point_buffer_dynamic_t right_points;
    right_points.len = input_buffer.len - (int)(input_buffer.len / 2);
    //NRF_LOG_INFO("Allocating (bytes %d*%d)", sizeof(point_t), right_points.len);
    right_points.buffer = pvPortMalloc(sizeof(point_t)*right_points.len);
    if (right_points.buffer == NULL) {
      NRF_LOG_ERROR("Failed to allocate right points buffer");
      return;
    }
    for (int i=(int)(input_buffer.len / 2); i < input_buffer.len; i++) {
      right_points.buffer[i-(int)(input_buffer.len / 2)] = input_buffer.buffer[i];
    }
    IEPF(right_points, p_output_buffer, T);
    
    // Free memory of input buffer (we have allocated memory for each split)
    vPortFree(input_buffer.buffer);

  }

}