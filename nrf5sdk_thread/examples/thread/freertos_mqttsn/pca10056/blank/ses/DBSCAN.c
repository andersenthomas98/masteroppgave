#include "mapping_types.h"
#include "nrf_log.h"
#include "math.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
#include "thread_mqttsn.h"

#include "DBSCAN.h"


void get_neighbors(point_reference_buffer_t* p_neighbor_buffer, 
                   point_buffer_t* p_point_buffer, 
                   float(*dist_func)(point_t, point_t), 
                   point_t point, 
                   float epsilon) 
{
  for (int i=0; i<p_point_buffer->len; i++) {
    float metric = dist_func(point, p_point_buffer->buffer[i]);
    if (metric <= epsilon) {
      p_neighbor_buffer->buffer[p_neighbor_buffer->len] = &p_point_buffer->buffer[i];
      p_neighbor_buffer->len++;
    }
  }

}

void append_to_set(point_reference_buffer_t *p_point_reference_buffer, point_t* p_point) {
  uint8_t len = p_point_reference_buffer->len;
  for (int i=0; i<len; i++) {
    if ((p_point->x     == p_point_reference_buffer->buffer[i]->x) && 
        (p_point->y     == p_point_reference_buffer->buffer[i]->y) && 
        (p_point->label == p_point_reference_buffer->buffer[i]->label)) 
    {
      return;
    }
  }
  p_point_reference_buffer->buffer[len]->x = p_point->x;
  p_point_reference_buffer->buffer[len]->y = p_point->y;
  p_point_reference_buffer->buffer[len]->label = p_point->label;
  p_point_reference_buffer->len++;

}

void expand_cluster(uint8_t num_clusters, 
                    point_reference_buffer_t* p_neighbor_buffer, 
                    point_buffer_t* p_point_buffer, 
                    float(*dist_func)(point_t, point_t),
                    float epsilon,
                    uint8_t min_points) 
{
  for (int i=0; i<p_neighbor_buffer->len; i++) {
    if (p_neighbor_buffer->buffer[i]->label == LABEL_UNDEFINED) {
      
      point_reference_buffer_t neighbors_neighbor_buffer;
      neighbors_neighbor_buffer.len = 0;
      get_neighbors(&neighbors_neighbor_buffer, p_point_buffer, dist_func, *(p_neighbor_buffer->buffer[i]), epsilon);

      if (neighbors_neighbor_buffer.len >= min_points) {
        for (int j = p_neighbor_buffer->len; j<p_neighbor_buffer->len + neighbors_neighbor_buffer.len; j++) {
          //p_neighbor_buffer->buffer[j] = neighbors_neighbor_buffer.buffer[j - neighbors_neighbor_buffer.len];
          //p_neighbor_buffer->len++;
          append_to_set(p_neighbor_buffer, neighbors_neighbor_buffer.buffer[j - p_neighbor_buffer->len]);
        }
        NRF_LOG_INFO("neighbor buffer len: %d", p_neighbor_buffer->len);
      }

    }
    if (p_neighbor_buffer->buffer[i]->label == LABEL_UNDEFINED || p_neighbor_buffer->buffer[i]->label == LABEL_NOISE) {
      NRF_LOG_INFO("%d: Add to cluster %d", i, num_clusters);
      p_neighbor_buffer->buffer[i]->label = num_clusters;
    }
    
  
  }


}

void DBSCAN(point_buffer_t* p_point_buffer, float(*dist_func)(point_t, point_t), float epsilon, uint8_t min_points) {
  uint8_t num_clusters = 0;
  uint8_t pb_len = p_point_buffer->len;
  NRF_LOG_INFO("len pb: %d", pb_len);
  for (int i=0; i<p_point_buffer->len; i++) {
    point_t* p_point = &(p_point_buffer->buffer[i]);
    if (p_point->label == LABEL_UNDEFINED) {
      point_reference_buffer_t neighbor_buffer;
      neighbor_buffer.len = 0;
      get_neighbors(&neighbor_buffer, p_point_buffer, dist_func, *p_point, epsilon);
      if (neighbor_buffer.len < min_points) {
        p_point->label = LABEL_NOISE;
      } 
      else {
        num_clusters++;
        p_point->label = num_clusters;
        NRF_LOG_INFO("Expand cluster %d", num_clusters);
        expand_cluster(num_clusters, &neighbor_buffer, p_point_buffer, dist_func, epsilon, min_points); 
      }
    }
  }
  
}