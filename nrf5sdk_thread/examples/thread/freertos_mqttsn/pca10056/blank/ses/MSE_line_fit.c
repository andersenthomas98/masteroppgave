

#include "mapping_types.h"
#include "mapping_utils.h"
#include "MSE_line_fit.h"
#include "math.h"
#include "FreeRTOS.h"
#include "float.h"

line_t MSE_line_fit(point_buffer_dynamic_t point_buffer) {
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

  // Find endpoints of line segment r = x*cos(theta) + y*sin(theta)

  point_t p, q;
  if (sin(theta_hat) <= 0.0001 && sin(theta_hat) >= -0.0001) {
    p = (point_t){
      .x = point_buffer.buffer[0].x,
      .y = -FLT_MAX * point_buffer.buffer[0].x + FLT_MAX
    };

    q = (point_t){
      .x = point_buffer.buffer[point_buffer.len-1].x,
      .y = -FLT_MAX * point_buffer.buffer[point_buffer.len-1].x + FLT_MAX
    
    };
  }
  else {
    p = (point_t){
      .x = point_buffer.buffer[0].x,
      .y = -(cos(theta_hat) / sin(theta_hat))*(point_buffer.buffer[0].x) + (r_hat / sin(theta_hat))
    };

    q = (point_t){
      .x = point_buffer.buffer[point_buffer.len-1].x,
      .y = -(cos(theta_hat) / sin(theta_hat))*(point_buffer.buffer[point_buffer.len-1].x) + (r_hat / sin(theta_hat))
    };
  }
  line_t line = {.P = p, .Q = q};
  point_t proj_point_start = get_projected_point_on_line(line, point_buffer.buffer[0]);
  point_t proj_point_end = get_projected_point_on_line(line, point_buffer.buffer[point_buffer.len-1]);
  line_t line_segment = {.P = proj_point_start, .Q = proj_point_end};
  
  return line_segment;

}

