

line_segment_t merge_segments(line_segment_t Line1, line_segment_t Line2);

void join_line_segments(line_segment_t* output_line, line_segment_t* line1, line_segment_t* line2);

uint8_t is_mergeable(line_segment_t line1, line_segment_t line2, float angle_threshold, float distance_threshold);

void merge_linebuffer(line_segment_buffer_t* lb, float angle_threshold, float distance_threshold);