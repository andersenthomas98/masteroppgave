

map_line_segment_t merge_segments(map_line_segment_t Line1, map_line_segment_t Line2);

uint8_t is_map_line_mergeable(map_line_segment_t line1, map_line_segment_t line2, float angle_threshold, float distance_threshold);

void join_line_segments(int8_t cluster_id, line_segment_t* output_line, line_segment_t line1, line_segment_t line2);

uint8_t is_mergeable(line_segment_t line1, line_segment_t line2, float angle_threshold, float distance_threshold);

void merge_linebuffer(line_segment_buffer_t* lb, float angle_threshold, float distance_threshold);