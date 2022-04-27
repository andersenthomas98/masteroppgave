

#include "mapping_types.h"
#include "mapping_utils.h"
#include "MSE_line_fit.h"
#include "math.h"
#include "FreeRTOS.h"
#include "float.h"

line_segment_t MSE_line_fit(point_buffer_dynamic_t point_buffer) {
  float mean_x = 0;
  float mean_y = 0;
  float mean_ss = 0;
  float sum_weights = 0;
  
  float sigma_xx = 0;
  float sigma_yy = 0;
  float sigma_xy = 0;
  
  float r_hat = 0;
  float theta_hat = 0;

  for (int i=0; i<point_buffer.len; i++) {
    mean_x += (float)point_buffer.buffer[i].x / (float)point_buffer.len;
    mean_y += (float)point_buffer.buffer[i].y / (float)point_buffer.len;
    mean_ss += (float)((point_buffer.buffer[i].label)*(point_buffer.buffer[i].label)) / (float)point_buffer.len;
  }

  float* weights = pvPortMalloc(sizeof(float)*point_buffer.len);
  for (int i=0; i<point_buffer.len; i++) {
    weights[i] = (float)((point_buffer.buffer[i].label)*(point_buffer.buffer[i].label)) / mean_ss;
    sum_weights += weights[i];
  }

  // Normalize weights
  for (int i=0; i<point_buffer.len; i++) {
    weights[i] = weights[i] / sum_weights;
  }

  for (int i=0; i<point_buffer.len; i++) {
    float weight = weights[i];
    sigma_xx += weight*(point_buffer.buffer[i].x - mean_x)*(point_buffer.buffer[i].x - mean_x) / (float) point_buffer.len;
    sigma_yy += weight*(point_buffer.buffer[i].y - mean_y)*(point_buffer.buffer[i].y - mean_y) / (float) point_buffer.len;
    sigma_xy += weight*(point_buffer.buffer[i].x - mean_x)*(point_buffer.buffer[i].y - mean_y) / (float) point_buffer.len;
  
  }
  
  vPortFree(weights);

  theta_hat = 0.5*atan2(-2*sigma_xy, sigma_yy - sigma_xx);
  r_hat = mean_x*cos(theta_hat) + mean_y*sin(theta_hat);

  if (r_hat < 0) {
    r_hat = fabs(r_hat);
    theta_hat += M_PI;
  }

  // Find endpoints of line r = x*cos(theta) + y*sin(theta)
  int start_index = 0;
  int end_index = 0;
  float max_dist = 0;
  for (int i=0; i<point_buffer.len; i++) {
    for (int j=0; j<point_buffer.len; j++) {
      float dist = get_length((line_t){.P = (point_t){.x = point_buffer.buffer[i].x, .y = point_buffer.buffer[i].y}, 
                                        .Q = (point_t){.x = point_buffer.buffer[j].x, .y = point_buffer.buffer[j].y}});
      if (dist > max_dist) {
         max_dist = dist;
         start_index = i;
         end_index = j;
      }
    }
  
  }
  point_t proj_point_start = get_projected_point_on_line_hesse(r_hat, theta_hat, point_buffer.buffer[start_index]);
  point_t proj_point_end = get_projected_point_on_line_hesse(r_hat, theta_hat, point_buffer.buffer[end_index]);
  line_segment_t line_segment = {.r = r_hat, .theta = theta_hat, .start = proj_point_start, .end = proj_point_end};

  
  return line_segment;

}

