
#include "mapping_types.h"
#include "mapping_utils.h"
#include "line_merging.h"
#include "math.h"
#include "FreeRTOS.h"
#include "MSE_line_fit.h"
#include "nrf_log.h"
#include "thread_mqttsn.h"

line_segment_t merge_segments(line_segment_t Line1, line_segment_t Line2) {
	
	//Define the coordinates of the centroid (x_m, y_m) of the set formed by the four endpoints
    //taking the respective segment lengths as the point "masses" 
	float l1 = get_length((line_t){.P = Line1.start, .Q = Line1.end});
	float l2 = get_length((line_t){.P = Line2.start, .Q = Line2.end});

	float x_m = (l1*(Line1.start.x + Line1.end.x) + l2*(Line2.start.x + Line2.end.x)) / (2*(l1+l2));
	float y_m = (l1*(Line1.start.y + Line1.end.y) + l2*(Line2.start.y + Line2.end.y)) / (2*(l1+l2));
	
	point_t M = {.x = x_m, .y = y_m};

	// Define the orientation of the merged line as the weighted sum of the orientations of the given segments taking
        // for weights their respective lengths. The orientations of the merged line thetaR is thus given by:
	float theta1 = Line1.theta + M_PI / 2;
	float theta2 = Line2.theta + M_PI / 2;
	float thetaR = 0.0;
	if (fabs(theta1 - theta2) <= M_PI/2.0) {
		thetaR = (l1*theta1 + l2*theta2) / (l1 + l2);
	} else {
		thetaR = (l1*theta1 + l2*(theta2 - M_PI*theta2/fabs(theta2))) / (l1 + l2);
	}

	// Determine coordinates of endpoints of both segments in the M (merge) frame.
	point_t P1 = transform(Line1.start, M, thetaR);
	point_t Q1 = transform(Line1.end, M, thetaR);
	point_t P2 = transform(Line2.start, M, thetaR);
	point_t Q2 = transform(Line2.end, M, thetaR);

	// Find orthogonal projections
	point_t a = {.x = P1.x, .y = 0};
	point_t b = {.x = Q1.x, .y = 0};
	point_t c = {.x = P2.x, .y = 0};
	point_t d = {.x = Q2.x, .y = 0};

	// Find largest distance among endpoints
	point_t orth[4] = {a, b, c, d};
	float largestDist = get_length((line_t){.P = a, .Q = b});
	point_t start = a;
	point_t end = b;
	for (int i=0; i<4; i++) {
		for (int j=i+1; j<4; j++) {
			float d = get_length((line_t){.P = orth[i], .Q = orth[j]});
			if (d > largestDist) {
				largestDist = d;
				start = orth[i];
				end = orth[j];
			}
		}
	}
	// Transform start and endpoint back to global frame
	start = rotate(start, thetaR);
	end = rotate(end, thetaR);
	start = translate(start, M);
	end = translate(end, M);

	return (line_segment_t) {.start = start, .end = end};
}


void join_line_segments(int8_t cluster_id, line_segment_t* output_line, line_segment_t* line1, line_segment_t* line2) {
  output_line->points.len = line1->points.len + line2->points.len;
  output_line->points.buffer = pvPortMalloc(sizeof(point_t)*output_line->points.len);
  if (output_line->points.buffer == NULL) {
    NRF_LOG_INFO("Failed to allocate point buffer");
  }
  for (int i=0; i<line1->points.len; i++) {
    output_line->points.buffer[i] = (point_t) {.x = line1->points.buffer[i].x, .y = line1->points.buffer[i].y, .label = 1};
  }
  for (int i=0; i<line2->points.len; i++) {
    output_line->points.buffer[i+(line1->points.len)] = (point_t) {.x = line2->points.buffer[i].x, .y = line2->points.buffer[i].y, .label = 1};
  }

  for (int i=0; i<output_line->points.len; i++) {
    mqttsn_cluster_msg_t msg;
    msg.point.x = output_line->points.buffer[i].x;
    msg.point.y = output_line->points.buffer[i].y;
    msg.cluster_id = cluster_id;
    publish_cluster_point("v2/robot/NRF_5/join", msg, sizeof(mqttsn_cluster_msg_t), 0, 0);
  
  }

  vPortFree(line1->points.buffer);
  line1->points.len = 0;
  vPortFree(line2->points.buffer);
  line2->points.len = 0;

  line_segment_t joint_line_segment = MSE_line_fit(output_line->points);

  output_line->r = joint_line_segment.r;
  output_line->theta = joint_line_segment.theta;
  output_line->start = joint_line_segment.start;
  output_line->end = joint_line_segment.end;
}


uint8_t is_mergeable(line_segment_t line1, line_segment_t line2, float angle_threshold, float distance_threshold) {
  // Test angle between line segments
  if (fabs(line1.theta - line2.theta) > angle_threshold) {
    return 0;
  }

  
  // Test if one of the endpoints of a line-segment is sufficiently near the other line-segment
  float dist1 = distance_from_point_to_line_segment(line1.start, (line_t){.P = line2.start, .Q = line2.end});

  float dist2 = distance_from_point_to_line_segment(line1.end, (line_t){.P = line2.start, .Q = line2.end});

  if (dist1 < distance_threshold || dist2 < distance_threshold) {
    return 1;
  }

  return 0;

}

void merge_linebuffer(line_segment_buffer_t* lb, float angle_threshold, float distance_threshold) {
	int converged = 0;
	int i = 0;
        
        int8_t msg_identifier = 0;

	while(!converged) {
		if (lb->len <= 1) {
			return;
		}
		for (i=0; i<lb->len-1; i++) {
			line_segment_t* line = &(lb->buffer[i]);
			line_segment_t* nextLine = &(lb->buffer[i+1]);
			if (is_mergeable(*line, *nextLine, angle_threshold, distance_threshold)) {
                                NRF_LOG_INFO("Mergeable!");
				line_segment_t joint_line_segment;
                                joint_line_segment.points.buffer = NULL;
                                joint_line_segment.points.len = 0;
                                join_line_segments(msg_identifier, &joint_line_segment, line, nextLine);
                                
                                mqttsn_line_msg_t msg_line;
                                msg_line.identifier = msg_identifier;
                                msg_line.startPoint = (coordinate_t){.x = line->start.x, .y = line->start.y};
                                msg_line.endPoint = (coordinate_t) {.x = line->end.x, .y = line->end.y};
                                msg_line.sigma_r2 = 0;
                                msg_line.sigma_rtheta = 0;
                                msg_line.sigma_theta2 = 0;
                                publish_line("v2/robot/NRF_5/merge", msg_line, sizeof(mqttsn_line_msg_t), 0, 0);

                                mqttsn_line_msg_t msg_nextLine;
                                msg_nextLine.identifier = msg_identifier;
                                msg_nextLine.startPoint = (coordinate_t){.x = nextLine->start.x, .y = nextLine->start.y};
                                msg_nextLine.endPoint = (coordinate_t) {.x = nextLine->end.x, .y = nextLine->end.y};
                                msg_nextLine.sigma_r2 = 0;
                                msg_nextLine.sigma_rtheta = 0;
                                msg_nextLine.sigma_theta2 = 0;
                                publish_line("v2/robot/NRF_5/merge", msg_nextLine, sizeof(mqttsn_line_msg_t), 0, 0);

                                mqttsn_line_msg_t msg_joint;
                                msg_joint.identifier = msg_identifier;
                                msg_joint.startPoint = (coordinate_t){.x = joint_line_segment.start.x, .y = joint_line_segment.start.y};
                                msg_joint.endPoint = (coordinate_t) {.x = joint_line_segment.end.x, .y = joint_line_segment.end.y};
                                msg_joint.sigma_r2 = 1;
                                msg_joint.sigma_rtheta = 1;
                                msg_joint.sigma_theta2 = 1;
                                publish_line("v2/robot/NRF_5/merge", msg_joint, sizeof(mqttsn_line_msg_t), 0, 0);

                                msg_identifier++;

				lb->buffer[i].start = joint_line_segment.start;
                                lb->buffer[i].end = joint_line_segment.end;
                                lb->buffer[i].r = joint_line_segment.r;
                                lb->buffer[i].theta = joint_line_segment.theta;
                                vPortFree(lb->buffer[i].points.buffer);
                                copy_points_to_line_segment(&(lb->buffer[i]), joint_line_segment.points);
                                vPortFree(joint_line_segment.points.buffer);
				// Copy next element value to current element to write over element i+1
				for (int j=i+1; j<lb->len-1; j++) {
					lb->buffer[j].start = lb->buffer[j+1].start;
                                        lb->buffer[j].end = lb->buffer[j+1].end;
                                        lb->buffer[j].r = lb->buffer[j+1].r;
                                        lb->buffer[j].theta = lb->buffer[j+1].theta;
                                        vPortFree(lb->buffer[j].points.buffer);
                                        copy_points_to_line_segment(&(lb->buffer[j]), lb->buffer[j+1].points);

				}
				lb->len -= 1;
				i = 0;
				break;
			}
		}
		if (i == lb->len-1) {
			converged = 1;
		}
	}
}