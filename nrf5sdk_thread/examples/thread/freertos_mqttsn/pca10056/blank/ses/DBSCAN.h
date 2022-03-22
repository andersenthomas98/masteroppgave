
#define LABEL_UNDEFINED 0
#define LABEL_NOISE -1


cluster_buffer_t DBSCAN(point_buffer_t* p_point_buffer, float(*dist_func)(point_t, point_t), float epsilon, uint8_t min_points);