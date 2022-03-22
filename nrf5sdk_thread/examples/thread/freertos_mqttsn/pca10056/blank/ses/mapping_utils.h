

void wrap_to_2pi(float *angle_in_radians);

point_t polar2cartesian(float theta, float r);

polar_t cartesian2polar(float x, float y);

float euclidean_distance(point_t P, point_t Q);

void append_cluster(cluster_buffer_t* cluster_buffer, point_buffer_t* p_point_buffer);