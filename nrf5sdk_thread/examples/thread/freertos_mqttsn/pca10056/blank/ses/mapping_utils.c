
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

void copy_points_to_line_segment(line_segment_t* line, point_buffer_dynamic_t points) {
  line->points.buffer = pvPortMalloc(sizeof(point_t)*points.len);
  line->points.len = points.len;
  
  for (int i=0; i<points.len; i++) {
    line->points.buffer[i] = points.buffer[i];
  }
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

point_t get_projected_point_on_line_hesse(float r, float theta, point_t point) {
  float r_p = point.x*cos(theta) + point.y*sin(theta);
  float d_x = fabs(r_p - r)*cos(theta);
  float d_y = fabs(r_p - r)*sin(theta);
  if (r_p - r >= 0) {
    return (point_t){.x = point.x - d_x, .y = point.y - d_y};
  } 
  return (point_t){.x = point.x + d_x, .y = point.y + d_y};


}

point_t rotate(point_t point, float theta) {
	float x = point.x*cos(theta) - point.y*sin(theta);
	float y = point.x*sin(theta) + point.y*cos(theta);
	return (point_t) {x, y};
}

point_t translate(point_t point, point_t delta) {
	float x = point.x + delta.x;
	float y = point.y + delta.y;
	return (point_t) {x, y};
}

point_t transform(point_t point, point_t origin, float theta) {
	float x = (point.y - origin.y)*sin(theta) + (point.x - origin.x)*cos(theta);
	float y = (point.y - origin.y)*cos(theta) - (point.x - origin.x)*sin(theta);
	return (point_t) {x, y};
}

// Returns shortest distance from a point to a line passing through two points https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
float distance_from_point_to_line(point_t point, line_t line) {
    return fabs((line.Q.x - line.P.x)*(line.P.y - point.y) - (line.P.x - point.x)*(line.Q.y - line.P.y))/get_length(line);
}

// Returns shortest distance from a point to a line segment https://math.stackexchange.com/questions/2248617/shortest-distance-between-a-point-and-a-line-segment
float distance_from_point_to_line_segment(point_t point, line_t line) {
    float t = -((line.P.x - point.x)*(line.Q.x - line.P.x) + (line.P.y - point.y)*(line.Q.y - line.P.y)) / (pow(get_length(line), 2));
    if (t >= 0 && t <= 1) {
        // The point is perpendicular to the line segment
        return distance_from_point_to_line(point, line);
    }
    float d1 = get_length((line_t){.P = point, .Q = line.P});
    float d2 = get_length((line_t){.P = point, .Q = line.Q});
    if (d1 < d2) {
        return d1;
    }
    return d2;
}