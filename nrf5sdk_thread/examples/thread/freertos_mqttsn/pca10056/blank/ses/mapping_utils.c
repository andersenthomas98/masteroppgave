
#include "mapping_types.h"
#include "mapping_utils.h"
#include "math.h"
#include "nrf_log.h"

/* Wrap any angle in radians into the interval [0,2pi) */
void wrap_to_2pi(float *angle_in_radians) {
    do {
        if (*angle_in_radians >= 2*M_PI) *angle_in_radians -= 2*M_PI;
        else if (*angle_in_radians < 0) *angle_in_radians += 2*M_PI;
    } while (fabs(*angle_in_radians) >= 2*M_PI);
}

/* Convert from polar to cartesian coordinates */
point_t polar2cartesian(float theta, float r) {
    float x = r * cos(theta);
    float y = r * sin(theta);
    return (point_t) { x, y };
}

polar_t cartesian2polar(float x, float y) {
  float r = sqrt(x*x+y*y);
  float theta = atan2(y, x);
  return (polar_t) {r, theta};
}

float euclidean_distance(point_t P, point_t Q) {
  return sqrtf((P.x - Q.x)*(P.x - Q.x) + (P.y - Q.y)*(P.y - Q.y));
}

void deallocate_cluster_buffer(cluster_buffer_t cluster_buffer) {
  for (int i=0; i<cluster_buffer.len; i++) {
    // Free each cluster
    //NRF_LOG_INFO("Deallocating (bytes %d*%d)", sizeof(cluster_buffer.buffer[i].buffer), cluster_buffer.buffer[i].len);
    vPortFree(cluster_buffer.buffer[i].buffer);
    cluster_buffer.buffer[i].len = 0;
  }
  // Free collection of clusters
  //NRF_LOG_INFO("Deallocating (bytes %d*%d)", sizeof(cluster_buffer.buffer), cluster_buffer.len);
  vPortFree(cluster_buffer.buffer);
  cluster_buffer.len = 0;

}

float get_length(line_t line) {
  return (sqrtf((line.P.x - line.Q.x)*(line.P.x - line.Q.x) + (line.P.y - line.Q.y)*(line.P.y - line.Q.y)));
}

float dot_product(point_t v1, point_t v2) {
  return (v1.x*v2.x + v1.y*v2.y);
}

point_t get_projected_point_on_line(line_t line, point_t point) {
  point_t e1 = (point_t) {.x = line.Q.x - line.P.x, .y = line.Q.y - line.P.y};
  point_t e2 = (point_t) {.x = point.x - line.P.x, .y = point.y - line.P.y};
  float dp = dot_product(e1, e2);
  float len2 = e1.x*e1.x + e1.y*e1.y;
  point_t proj_point = (point_t) {.x = (line.P.x + (dp * e1.x) / len2), .y = (line.P.y + (dp*e1.y) / len2)}; // TODO: Check for division by zero
  return proj_point;
  
}