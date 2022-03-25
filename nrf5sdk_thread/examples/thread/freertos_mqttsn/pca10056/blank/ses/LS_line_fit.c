

#include "mapping_types.h"
#include "mapping_utils.h"
#include "LS_line_fit.h"
#include "float.h"

line_t LS_line_fit(point_buffer_dynamic_t points) {
  float sum_x = 0;
  float sum_yy = 0;
  float sum_y = 0;
  float sum_xy = 0;
  float sum_xx = 0;
  for (int l=0; l<points.len; l++) {
    point_t point = points.buffer[l];
    sum_x += point.x;
    sum_y += point.y;
    sum_xx += point.x * point.x;
    sum_yy += point.y*point.y;
    sum_xy += point.x * point.y;
  }
  float a_hat = sum_x*sum_yy - sum_y*sum_xy;
  float b_hat = sum_y*sum_xx - sum_x*sum_xy;
  float c_hat = sum_xy*sum_xy - sum_xx*sum_yy;
  if (b_hat == 0) {
    a_hat = FLT_MAX;
    b_hat = FLT_MAX;
    b_hat = 1.0;
  }
  point_t p = {
    .x = points.buffer[0].x,
    .y = -(a_hat / b_hat)*(points.buffer[0].x) - (c_hat / b_hat)
  };
  point_t q = {
    .x = points.buffer[points.len-1].x,
    .y = -(a_hat / b_hat)*(points.buffer[points.len-1].x) - (c_hat / b_hat)
  };
  line_t line = {.P = p, .Q = q};
  point_t proj_point_start = get_projected_point_on_line(line, points.buffer[0]);
  point_t proj_point_end = get_projected_point_on_line(line, points.buffer[points.len-1]);
  line_t ls_fit_line = {.P = proj_point_start, .Q = proj_point_end};

  return ls_fit_line;

}