#include "mapping_types.h"
#include "nrf_log.h"
#include "math.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
#include "thread_mqttsn.h"

#include "DBSCAN.h"
#include "mapping_utils.h"

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
  p_point_reference_buffer->buffer[len] = p_point;
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
        for (int j = 0; j<neighbors_neighbor_buffer.len; j++) {
          append_to_set(p_neighbor_buffer, neighbors_neighbor_buffer.buffer[j]);
        }
      }

    }
    if (p_neighbor_buffer->buffer[i]->label == LABEL_UNDEFINED || p_neighbor_buffer->buffer[i]->label == LABEL_NOISE) {
      p_neighbor_buffer->buffer[i]->label = num_clusters;
    }
    
  
  }

}

cluster_buffer_t DBSCAN(point_buffer_t* p_point_buffer, float(*dist_func)(point_t, point_t), float epsilon, uint8_t min_points) {
  uint8_t num_clusters = 0;
  uint8_t pb_len = p_point_buffer->len;
  //NRF_LOG_INFO("len pb: %d", pb_len);
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
        //NRF_LOG_INFO("Expand cluster %d", num_clusters);
        expand_cluster(num_clusters, &neighbor_buffer, p_point_buffer, dist_func, epsilon, min_points); 
      }
    }
  }
  // DBSCAN finished
  if (num_clusters <= 0) {
    return (cluster_buffer_t){.len = 0, .buffer = NULL};
  }

  // Group points in point buffer based on cluster label
  cluster_buffer_t clusters;
  //NRF_LOG_INFO("Allocating (bytes %d) cluster buffer for %d clusters", sizeof(point_buffer_dynamic_t)*num_clusters, num_clusters);
  clusters.buffer = pvPortMalloc(sizeof(point_buffer_dynamic_t)*num_clusters);
  if (clusters.buffer == NULL) {
    NRF_LOG_ERROR("Failed to allocate memory for cluster");
    return (cluster_buffer_t){.len = 0, .buffer = NULL};
  }
  clusters.len = num_clusters;
  uint8_t cluster_index = 0;
  for (int cluster_id=1; cluster_id<=num_clusters; cluster_id++) {
    uint8_t num_points_in_cluster = 0;
    for (int j=0; j<p_point_buffer->len; j++) {
      if (p_point_buffer->buffer[j].label == cluster_id) {
        num_points_in_cluster++;
      }
    }
    
    point_buffer_dynamic_t cluster;
    cluster.len = num_points_in_cluster;
    //NRF_LOG_INFO("Allocating (bytes %d) cluster for %d points", sizeof(point_t)*cluster.len, cluster.len);
    cluster.buffer = pvPortMalloc(sizeof(point_t)*cluster.len);
    if (cluster.buffer == NULL) {
      NRF_LOG_ERROR("Failed to allocate memory for cluster");
      return (cluster_buffer_t){.len = 0, .buffer = NULL};
    }
    uint8_t buffer_index = 0;
    for (int i=0; i<p_point_buffer->len; i++) {
      if (p_point_buffer->buffer[i].label == cluster_id) {
        cluster.buffer[buffer_index] = p_point_buffer->buffer[i];
        buffer_index++;
      }
    }
    clusters.buffer[cluster_index] = cluster;
    cluster_index++;
  
  }

  return clusters;
  
}

void get_neighbors2(point_reference_buffer_t* p_neighbor_buffer, 
                   point_buffer_common_t* p_point_buffer, 
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

void expand_cluster2(uint8_t num_clusters, 
                    point_reference_buffer_t* p_neighbor_buffer, 
                    point_buffer_common_t* p_point_buffer, 
                    float(*dist_func)(point_t, point_t),
                    float epsilon,
                    uint8_t min_points) 
{
  for (int i=0; i<p_neighbor_buffer->len; i++) {
    if (p_neighbor_buffer->buffer[i]->label == LABEL_UNDEFINED) {
      
      point_reference_buffer_t neighbors_neighbor_buffer;
      neighbors_neighbor_buffer.len = 0;
      get_neighbors2(&neighbors_neighbor_buffer, p_point_buffer, dist_func, *(p_neighbor_buffer->buffer[i]), epsilon);

      if (neighbors_neighbor_buffer.len >= min_points) {
        for (int j = 0; j<neighbors_neighbor_buffer.len; j++) {
          append_to_set(p_neighbor_buffer, neighbors_neighbor_buffer.buffer[j]);
        }
      }

    }
    if (p_neighbor_buffer->buffer[i]->label == LABEL_UNDEFINED || p_neighbor_buffer->buffer[i]->label == LABEL_NOISE) {
      p_neighbor_buffer->buffer[i]->label = num_clusters;
    }
    
  
  }

}

cluster_buffer_t DBSCAN2(point_buffer_common_t* p_point_buffer, float(*dist_func)(point_t, point_t), float epsilon, uint8_t min_points) {
  uint8_t num_clusters = 0;
  uint8_t pb_len = p_point_buffer->len;
  //NRF_LOG_INFO("len pb: %d", pb_len);
  for (int i=0; i<p_point_buffer->len; i++) {
    point_t* p_point = &(p_point_buffer->buffer[i]);
    if (p_point->label == LABEL_UNDEFINED) {
      point_reference_buffer_t neighbor_buffer;
      neighbor_buffer.len = 0;
      get_neighbors2(&neighbor_buffer, p_point_buffer, dist_func, *p_point, epsilon);
      if (neighbor_buffer.len < min_points) {
        p_point->label = LABEL_NOISE;
      } 
      else {
        num_clusters++;
        p_point->label = num_clusters;
        //NRF_LOG_INFO("Expand cluster %d", num_clusters);
        expand_cluster2(num_clusters, &neighbor_buffer, p_point_buffer, dist_func, epsilon, min_points); 
      }
    }
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    NRF_LOG_INFO("stack: %ld", uxHighWaterMark);
  }
  // DBSCAN finished
  if (num_clusters <= 0) {
    return (cluster_buffer_t){.len = 0, .buffer = NULL};
  }

  // Group points in point buffer based on cluster label
  cluster_buffer_t clusters;
  //NRF_LOG_INFO("Allocating (bytes %d) cluster buffer for %d clusters", sizeof(point_buffer_dynamic_t)*num_clusters, num_clusters);
  clusters.buffer = pvPortMalloc(sizeof(point_buffer_dynamic_t)*num_clusters);
  if (clusters.buffer == NULL) {
    NRF_LOG_ERROR("Failed to allocate memory for cluster");
    return (cluster_buffer_t){.len = 0, .buffer = NULL};
  }
  clusters.len = num_clusters;
  uint8_t cluster_index = 0;
  for (int cluster_id=1; cluster_id<=num_clusters; cluster_id++) {
    uint8_t num_points_in_cluster = 0;
    for (int j=0; j<p_point_buffer->len; j++) {
      if (p_point_buffer->buffer[j].label == cluster_id) {
        num_points_in_cluster++;
      }
    }
    
    point_buffer_dynamic_t cluster;
    cluster.len = num_points_in_cluster;
    //NRF_LOG_INFO("Allocating (bytes %d) cluster for %d points", sizeof(point_t)*cluster.len, cluster.len);
    cluster.buffer = pvPortMalloc(sizeof(point_t)*cluster.len);
    if (cluster.buffer == NULL) {
      NRF_LOG_ERROR("Failed to allocate memory for cluster");
      return (cluster_buffer_t){.len = 0, .buffer = NULL};
    }
    uint8_t buffer_index = 0;
    for (int i=0; i<p_point_buffer->len; i++) {
      if (p_point_buffer->buffer[i].label == cluster_id) {
        cluster.buffer[buffer_index] = p_point_buffer->buffer[i];
        buffer_index++;
      }
    }
    clusters.buffer[cluster_index] = cluster;
    cluster_index++;
  
  }
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  NRF_LOG_INFO("stack: %ld", uxHighWaterMark);

  return clusters;
  
}