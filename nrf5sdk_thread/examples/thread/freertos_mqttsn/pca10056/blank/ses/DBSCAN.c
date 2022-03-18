#include "mapping_types.h"
#include "nrf_log.h"
#include "math.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
#include "thread_mqttsn.h"

#include "DBSCAN.h"


void get_neighbors(point_buffer_t* p_neighbor_buffer, 
                   point_buffer_t* p_point_buffer, 
                   float(*dist_func)(point_t, point_t), 
                   point_t point, 
                   float epsilon) 
{
  for (int i=0; i<p_point_buffer->len; i++) {
    if (dist_func(point, p_point_buffer->buffer[i]) <= epsilon) {
      p_neighbor_buffer->buffer[p_neighbor_buffer->len] = p_point_buffer->buffer[i];
      p_neighbor_buffer->len++;
    }
  }

}

void expand_cluster(uint8_t num_clusters, 
                    point_buffer_t* p_neighbor_buffer, 
                    point_buffer_t* p_point_buffer, 
                    float(*dist_func)(point_t, point_t),
                    float epsilon) 
{
  for (int i=0; i<p_neighbor_buffer->len; i++) {
    if (p_neighbor_buffer->buffer[i].label == LABEL_UNDEFINED) {
      point_buffer_t neighbors_neighbor_buffer;
      neighbors_neighbor_buffer.len = 0;
      get_neighbors(&neighbors_neighbor_buffer, p_point_buffer, dist_func, p_neighbor_buffer->buffer[i], epsilon);
      for (int j = p_neighbor_buffer->len; j<p_neighbor_buffer->len + neighbors_neighbor_buffer.len; j++) {
        p_neighbor_buffer->buffer[j] = neighbors_neighbor_buffer.buffer[j - neighbors_neighbor_buffer.len];
      }
    }
    if (p_neighbor_buffer->buffer[i].label == LABEL_UNDEFINED || p_neighbor_buffer->buffer[i].label == LABEL_NOISE) {
      p_neighbor_buffer->buffer[i].label = num_clusters;
    }
    
  
  }


}

void DBSCAN(point_buffer_t* p_point_buffer, float(*dist_func)(point_t, point_t), float epsilon, uint8_t min_points) {
  uint8_t num_clusters = 0;
  uint8_t pb_len = p_point_buffer->len;
  NRF_LOG_INFO("len pb: %d", pb_len);
  for (int i=0; i<p_point_buffer->len; i++) {
    point_t point = p_point_buffer->buffer[i];
    if (point.label == LABEL_UNDEFINED) {
      point_buffer_t neighbor_buffer;
      neighbor_buffer.len = 0;
      get_neighbors(&neighbor_buffer, p_point_buffer, dist_func, point, epsilon);
      if (neighbor_buffer.len < min_points) {
        point.label = LABEL_NOISE;
      } 
      else {
        point.label = ++num_clusters;
        expand_cluster(num_clusters, &neighbor_buffer, p_point_buffer, dist_func, epsilon); // TODO: Should min_points be here?

      }
    }
  }
  
}