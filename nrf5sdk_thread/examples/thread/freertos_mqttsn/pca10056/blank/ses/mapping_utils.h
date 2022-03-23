

void wrap_to_2pi(float *angle_in_radians);

point_t polar2cartesian(float theta, float r);

polar_t cartesian2polar(float x, float y);

float euclidean_distance(point_t P, point_t Q);

void deallocate_cluster_buffer(cluster_buffer_t cluster_buffer);

float get_length(line_t line);

float dot_product(point_t v1, point_t v2);

point_t get_projected_point_on_line(line_t line, point_t point);