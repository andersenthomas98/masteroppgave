
#include "mqttsn_client.h"
#include "mapping.h"
#include "thread_mqttsn.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "nrf_log.h"
#include "SensorTowerTask.h"
#include "robot_config.h"

#include "math.h"
#include "defines.h"
#include "mapping_types.h"
#include "mapping_utils.h"
#include "DBSCAN.h"
#include "IEPF.h"
#include "LS_line_fit.h"
#include "MSE_line_fit.h"
#include "line_merging.h"
#include "float.h"

#define UNDEFINED -1

extern QueueHandle_t ir_measurement_queue;
extern TaskHandle_t mapping_task_handle;

int sgn(float val) {
  if (val > 0) {
    return 1;
  }
  if (val < 0) {
    return -1;
  }
  return 0;
}

void update_point_buffers(point_buffer_t* point_buffers, ir_measurement_t measurement) {

  /*Convert range-bearing measurement to obstacle position in global reference frame and update point buffers with obstacle positions */
  float theta = measurement.servo_angle*DEG2RAD + measurement.theta;
  //NRF_LOG_INFO("Servo angle: %d", measurement.servo_angle);
  for (int i=0; i<NUM_DIST_SENSORS; i++) {

    //NRF_LOG_INFO("Sensor %d range: %d", i, measurement.measurements[i]);

    if (i > 0) {
      theta += 0.5 * M_PI; // 90 degrees between each ir sensor
    }
    wrap_to_2pi(&theta);
    uint16_t range = measurement.measurements[i];

    // Prune away measurements outside valid range
    if (range <= 0 || range > IR_MAX_DETECT_DISTANCE_MM) {
      continue;
    }

    point_t point = polar2cartesian(theta, (float) range);
    point.x += measurement.x;
    point.y += measurement.y;
    uint8_t pb_len = point_buffers[i].len;
    //NRF_LOG_INFO("%d", pb_len);
    point_buffers[i].len++;
    point_buffers[i].buffer[pb_len].x = point.x;
    point_buffers[i].buffer[pb_len].y = point.y;
    point_buffers[i].buffer[pb_len].label = LABEL_UNDEFINED;

    if (pb_len >= PB_MAX_SIZE) {
      // Start line extraction
      xTaskNotifyGive(mapping_task_handle);

    }
  }
}

void mapping_task(void *arg) {

  TickType_t lastWakeTime;
  const TickType_t delay = 0.1;
  
  point_buffer_t point_buffers[NUM_DIST_SENSORS];
  point_buffer_common_t point_buffer_common;
  point_buffer_common.len = 0;

  static volatile line_segment_buffer_t line_buffer;
  line_buffer.len = 0;

  for (int i=0; i<NUM_DIST_SENSORS; i++) {
    point_buffers[i].len = 0;
  }

  point_buffers[0].buffer[0] = (point_t){.x = 191, .y = 0, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[1] = (point_t){.x = 190, .y = 3, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[2] = (point_t){.x = 190, .y = 6, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[3] = (point_t){.x = 191, .y = 10, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[4] = (point_t){.x = 189, .y = 13, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[5] = (point_t){.x = 199, .y = 17, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[6] = (point_t){.x = 189, .y = 19, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[7] = (point_t){.x = 190, .y = 23, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[8] = (point_t){.x = 165, .y = 23, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[9] = (point_t){.x = 183, .y = 29, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[10] = (point_t){.x = 186, .y = 32, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[11] = (point_t){.x = 164, .y = 32, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[12] = (point_t){.x = 186, .y = 39, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[13] = (point_t){.x = 184, .y = 42, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[14] = (point_t){.x = 185, .y = 46, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[15] = (point_t){.x = 183, .y = 49, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[16] = (point_t){.x = 182, .y = 52, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[17] = (point_t){.x = 184, .y = 56, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[18] = (point_t){.x = 185, .y = 60, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[19] = (point_t){.x = 182, .y = 62, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[20] = (point_t){.x = 183, .y = 66, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[21] = (point_t){.x = 181, .y = 69, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[22] = (point_t){.x = 183, .y = 74, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[23] = (point_t){.x = 181, .y = 76, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[24] = (point_t){.x = 178, .y = 79, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[25] = (point_t){.x = 179, .y = 83, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[26] = (point_t){.x = 159, .y = 77, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[27] = (point_t){.x = 181, .y = 92, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[28] = (point_t){.x = 179, .y = 95, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[29] = (point_t){.x = 177, .y = 98, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[30] = (point_t){.x = 177, .y = 102, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[31] = (point_t){.x = 177, .y = 106, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[32] = (point_t){.x = 178, .y = 111, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[33] = (point_t){.x = 176, .y = 114, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[34] = (point_t){.x = 179, .y = 120, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[35] = (point_t){.x = 176, .y = 123, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[36] = (point_t){.x = 176, .y = 128, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[37] = (point_t){.x = 176, .y = 133, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[38] = (point_t){.x = 176, .y = 137, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[39] = (point_t){.x = 174, .y = 141, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[40] = (point_t){.x = 173, .y = 145, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[41] = (point_t){.x = 175, .y = 152, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[42] = (point_t){.x = 173, .y = 155, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[43] = (point_t){.x = 173, .y = 161, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[44] = (point_t){.x = 171, .y = 166, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[45] = (point_t){.x = 162, .y = 162, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[46] = (point_t){.x = 170, .y = 176, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[47] = (point_t){.x = 151, .y = 162, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[48] = (point_t){.x = 171, .y = 190, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[49] = (point_t){.x = 153, .y = 176, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[50] = (point_t){.x = 168, .y = 200, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[51] = (point_t){.x = 156, .y = 192, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[52] = (point_t){.x = 169, .y = 217, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[53] = (point_t){.x = 168, .y = 223, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[54] = (point_t){.x = 169, .y = 233, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[55] = (point_t){.x = 165, .y = 235, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[56] = (point_t){.x = 163, .y = 242, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[57] = (point_t){.x = 161, .y = 248, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[58] = (point_t){.x = 156, .y = 251, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[59] = (point_t){.x = 148, .y = 246, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[60] = (point_t){.x = 141, .y = 245, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[61] = (point_t){.x = 133, .y = 240, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[62] = (point_t){.x = 129, .y = 243, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[63] = (point_t){.x = 122, .y = 240, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[64] = (point_t){.x = 117, .y = 241, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[65] = (point_t){.x = 110, .y = 236, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[66] = (point_t){.x = 107, .y = 242, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[67] = (point_t){.x = 100, .y = 236, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[68] = (point_t){.x = 95, .y = 237, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[69] = (point_t){.x = 92, .y = 240, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[70] = (point_t){.x = 85, .y = 233, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[71] = (point_t){.x = 82, .y = 238, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[72] = (point_t){.x = 76, .y = 234, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[73] = (point_t){.x = 72, .y = 236, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[74] = (point_t){.x = 65, .y = 229, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[75] = (point_t){.x = 62, .y = 231, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[76] = (point_t){.x = 58, .y = 233, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[77] = (point_t){.x = 53, .y = 230, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[78] = (point_t){.x = 49, .y = 231, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[79] = (point_t){.x = 44, .y = 227, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[80] = (point_t){.x = 34, .y = 195, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[81] = (point_t){.x = 35, .y = 226, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[82] = (point_t){.x = 31, .y = 226, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[83] = (point_t){.x = 27, .y = 226, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[84] = (point_t){.x = 23, .y = 224, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[85] = (point_t){.x = 19, .y = 227, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[86] = (point_t){.x = 15, .y = 222, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[87] = (point_t){.x = 11, .y = 224, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[88] = (point_t){.x = 7, .y = 224, .label = LABEL_UNDEFINED}; 
  point_buffers[0].buffer[89] = (point_t){.x = 3, .y = 206, .label = LABEL_UNDEFINED}; 
  point_buffers[0].len = 90;

  point_buffers[1].buffer[0] = (point_t){.x = 0, .y = 211, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[1] = (point_t){.x = -3, .y = 219, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[2] = (point_t){.x = -7, .y = 214, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[3] = (point_t){.x = -11, .y = 216, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[4] = (point_t){.x = -15, .y = 219, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[5] = (point_t){.x = -19, .y = 221, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[6] = (point_t){.x = -21, .y = 206, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[7] = (point_t){.x = -26, .y = 217, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[8] = (point_t){.x = -29, .y = 211, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[9] = (point_t){.x = -33, .y = 213, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[10] = (point_t){.x = -37, .y = 212, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[11] = (point_t){.x = -41, .y = 214, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[12] = (point_t){.x = -45, .y = 215, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[13] = (point_t){.x = -49, .y = 215, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[14] = (point_t){.x = -53, .y = 213, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[15] = (point_t){.x = -56, .y = 210, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[16] = (point_t){.x = -60, .y = 211, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[17] = (point_t){.x = -64, .y = 209, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[18] = (point_t){.x = -68, .y = 212, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[19] = (point_t){.x = -70, .y = 204, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[20] = (point_t){.x = -72, .y = 200, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[21] = (point_t){.x = -74, .y = 194, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[22] = (point_t){.x = -80, .y = 198, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[23] = (point_t){.x = -88, .y = 208, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[24] = (point_t){.x = -93, .y = 209, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[25] = (point_t){.x = -95, .y = 205, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[26] = (point_t){.x = -101, .y = 207, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[27] = (point_t){.x = -105, .y = 206, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[28] = (point_t){.x = -110, .y = 208, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[29] = (point_t){.x = -114, .y = 206, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[30] = (point_t){.x = -118, .y = 206, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[31] = (point_t){.x = -124, .y = 207, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[32] = (point_t){.x = -127, .y = 203, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[33] = (point_t){.x = -131, .y = 202, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[34] = (point_t){.x = -126, .y = 187, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[35] = (point_t){.x = -145, .y = 207, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[36] = (point_t){.x = -147, .y = 203, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[37] = (point_t){.x = -154, .y = 205, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[38] = (point_t){.x = -158, .y = 203, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[39] = (point_t){.x = -159, .y = 197, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[40] = (point_t){.x = -167, .y = 199, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[41] = (point_t){.x = -172, .y = 198, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[42] = (point_t){.x = -179, .y = 199, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[43] = (point_t){.x = -170, .y = 182, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[44] = (point_t){.x = -195, .y = 202, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[45] = (point_t){.x = -198, .y = 198, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[46] = (point_t){.x = -203, .y = 196, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[47] = (point_t){.x = -214, .y = 199, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[48] = (point_t){.x = -245, .y = 220, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[49] = (point_t){.x = -281, .y = 244, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[50] = (point_t){.x = -368, .y = 309, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[51] = (point_t){.x = -403, .y = 326, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[52] = (point_t){.x = -339, .y = 265, .label = LABEL_UNDEFINED}; 
  point_buffers[1].buffer[53] = (point_t){.x = -430, .y = 324, .label = LABEL_UNDEFINED}; 
  point_buffers[1].len = 54;

  point_buffers[2].buffer[0] = (point_t){.x = -258, .y = -368, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[1] = (point_t){.x = -296, .y = -439, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[2] = (point_t){.x = -193, .y = -297, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[3] = (point_t){.x = -161, .y = -258, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[4] = (point_t){.x = -135, .y = -225, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[5] = (point_t){.x = -155, .y = -269, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[6] = (point_t){.x = -146, .y = -264, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[7] = (point_t){.x = -143, .y = -269, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[8] = (point_t){.x = -122, .y = -240, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[9] = (point_t){.x = -131, .y = -269, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[10] = (point_t){.x = -123, .y = -264, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[11] = (point_t){.x = -119, .y = -268, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[12] = (point_t){.x = -100, .y = -237, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[13] = (point_t){.x = -107, .y = -267, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[14] = (point_t){.x = -99, .y = -260, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[15] = (point_t){.x = -93, .y = -257, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[16] = (point_t){.x = -88, .y = -257, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[17] = (point_t){.x = -82, .y = -254, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[18] = (point_t){.x = -77, .y = -254, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[19] = (point_t){.x = -73, .y = -254, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[20] = (point_t){.x = -67, .y = -251, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[21] = (point_t){.x = -63, .y = -253, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[22] = (point_t){.x = -58, .y = -254, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[23] = (point_t){.x = -52, .y = -249, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[24] = (point_t){.x = -48, .y = -250, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[25] = (point_t){.x = -43, .y = -244, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[26] = (point_t){.x = -39, .y = -246, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[27] = (point_t){.x = -34, .y = -244, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[28] = (point_t){.x = -30, .y = -246, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[29] = (point_t){.x = -26, .y = -249, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[30] = (point_t){.x = -19, .y = -222, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[31] = (point_t){.x = -14, .y = -203, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[32] = (point_t){.x = -12, .y = -240, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[33] = (point_t){.x = -7, .y = -220, .label = LABEL_UNDEFINED}; 
  point_buffers[2].buffer[34] = (point_t){.x = -4, .y = -246, .label = LABEL_UNDEFINED}; 
  point_buffers[2].len = 35;

  point_buffers[3].buffer[0] = (point_t){.x = 0, .y = -257, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[1] = (point_t){.x = 4, .y = -257, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[2] = (point_t){.x = 9, .y = -259, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[3] = (point_t){.x = 13, .y = -255, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[4] = (point_t){.x = 17, .y = -257, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[5] = (point_t){.x = 21, .y = -247, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[6] = (point_t){.x = 27, .y = -257, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[7] = (point_t){.x = 31, .y = -256, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[8] = (point_t){.x = 35, .y = -255, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[9] = (point_t){.x = 40, .y = -252, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[10] = (point_t){.x = 44, .y = -254, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[11] = (point_t){.x = 47, .y = -242, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[12] = (point_t){.x = 53, .y = -250, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[13] = (point_t){.x = 57, .y = -247, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[14] = (point_t){.x = 61, .y = -247, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[15] = (point_t){.x = 68, .y = -255, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[16] = (point_t){.x = 70, .y = -244, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[17] = (point_t){.x = 76, .y = -248, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[18] = (point_t){.x = 81, .y = -251, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[19] = (point_t){.x = 85, .y = -248, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[20] = (point_t){.x = 89, .y = -247, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[21] = (point_t){.x = 97, .y = -253, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[22] = (point_t){.x = 100, .y = -248, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[23] = (point_t){.x = 105, .y = -248, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[24] = (point_t){.x = 109, .y = -246, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[25] = (point_t){.x = 114, .y = -246, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[26] = (point_t){.x = 120, .y = -247, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[27] = (point_t){.x = 128, .y = -252, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[28] = (point_t){.x = 132, .y = -249, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[29] = (point_t){.x = 136, .y = -246, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[30] = (point_t){.x = 139, .y = -241, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[31] = (point_t){.x = 149, .y = -248, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[32] = (point_t){.x = 153, .y = -245, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[33] = (point_t){.x = 155, .y = -239, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[34] = (point_t){.x = 166, .y = -246, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[35] = (point_t){.x = 159, .y = -227, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[36] = (point_t){.x = 168, .y = -232, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[37] = (point_t){.x = 182, .y = -241, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[38] = (point_t){.x = 184, .y = -236, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[39] = (point_t){.x = 178, .y = -220, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[40] = (point_t){.x = 195, .y = -232, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[41] = (point_t){.x = 202, .y = -232, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[42] = (point_t){.x = 198, .y = -219, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[43] = (point_t){.x = 216, .y = -231, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[44] = (point_t){.x = 227, .y = -235, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[45] = (point_t){.x = 236, .y = -236, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[46] = (point_t){.x = 236, .y = -228, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[47] = (point_t){.x = 242, .y = -226, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[48] = (point_t){.x = 247, .y = -222, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[49] = (point_t){.x = 247, .y = -215, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[50] = (point_t){.x = 242, .y = -203, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[51] = (point_t){.x = 240, .y = -194, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[52] = (point_t){.x = 237, .y = -185, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[53] = (point_t){.x = 241, .y = -182, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[54] = (point_t){.x = 239, .y = -173, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[55] = (point_t){.x = 237, .y = -166, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[56] = (point_t){.x = 239, .y = -161, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[57] = (point_t){.x = 237, .y = -154, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[58] = (point_t){.x = 234, .y = -146, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[59] = (point_t){.x = 229, .y = -138, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[60] = (point_t){.x = 228, .y = -131, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[61] = (point_t){.x = 227, .y = -126, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[62] = (point_t){.x = 215, .y = -114, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[63] = (point_t){.x = 221, .y = -113, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[64] = (point_t){.x = 225, .y = -110, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[65] = (point_t){.x = 229, .y = -106, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[66] = (point_t){.x = 218, .y = -97, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[67] = (point_t){.x = 220, .y = -93, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[68] = (point_t){.x = 218, .y = -88, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[69] = (point_t){.x = 217, .y = -83, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[70] = (point_t){.x = 214, .y = -77, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[71] = (point_t){.x = 212, .y = -73, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[72] = (point_t){.x = 210, .y = -68, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[73] = (point_t){.x = 211, .y = -64, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[74] = (point_t){.x = 214, .y = -61, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[75] = (point_t){.x = 210, .y = -56, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[76] = (point_t){.x = 202, .y = -50, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[77] = (point_t){.x = 208, .y = -48, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[78] = (point_t){.x = 196, .y = -41, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[79] = (point_t){.x = 205, .y = -39, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[80] = (point_t){.x = 204, .y = -36, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[81] = (point_t){.x = 202, .y = -32, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[82] = (point_t){.x = 204, .y = -28, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[83] = (point_t){.x = 202, .y = -24, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[84] = (point_t){.x = 202, .y = -21, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[85] = (point_t){.x = 200, .y = -17, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[86] = (point_t){.x = 193, .y = -13, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[87] = (point_t){.x = 202, .y = -10, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[88] = (point_t){.x = 206, .y = -7, .label = LABEL_UNDEFINED}; 
  point_buffers[3].buffer[89] = (point_t){.x = 201, .y = -3, .label = LABEL_UNDEFINED}; 
  point_buffers[3].len = 90;




  mqttsn_line_msg_t msg;
  msg.identifier = LINE_IDENTIFIER;

  mqttsn_update_msg_t update_msg;
  update_msg.identifier = UPDATE_IDENTIFIER;

  float lastPublishedX = 0;
  float lastPublishedY = 0;
  float lastPublishedTheta = 0;

  int servoDir = UNDEFINED;
  
  mqttsn_cluster_msg_t dbscan_msg;
  mqttsn_cluster_msg_t iepf_msg;

  ir_measurement_t new_measurement;
  
  /*while(1) {
     point_t p1 = {.x = 0, .y = 0};
     point_t p2 = {.x = 0, .y = 0.5};
     point_t p3 = {.x = 0, .y = 1};
     point_t p4 = {.x = 0.5, .y = 1};
     point_t p5 = {.x = 1, .y = 1};
     point_t p6 = {.x = 1.5, .y = 1};
     point_t p7 = {.x = 2, .y = 1};
     point_t p8 = {.x = 2, .y = 0.5};
     point_t p9 = {.x = 2, .y = 0};

     point_buffer_dynamic_t buffer;
     buffer.len = 9;
     buffer.buffer = pvPortMalloc(sizeof(point_t)*buffer.len);
     buffer.buffer[0] = p1;
     buffer.buffer[1] = p2;
     buffer.buffer[2] = p3;
     buffer.buffer[3] = p4;
     buffer.buffer[4] = p5;
     buffer.buffer[5] = p6;
     buffer.buffer[6] = p7;
     buffer.buffer[7] = p8;
     buffer.buffer[8] = p9;

     cluster_buffer_t output;
     output.len = 0;
     output.buffer = NULL;

     IEPF(buffer, &output, 0.1);

     for (int i = 0; i<output.len; i++) {
      NRF_LOG_INFO("cluster %d", i);
      for (int j=0; j<output.buffer[i].len; j++) {
        point_t point = output.buffer[i].buffer[j];
        NRF_LOG_INFO("point %d", j);
        NRF_LOG_INFO("x: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(point.x))
        NRF_LOG_INFO("y: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(point.y))
      }

     }
  }*/

  while(!mqttsn_client_is_connected()) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*5);
  }

  /*while(1) {
    int freeHeap = xPortGetFreeHeapSize();
    NRF_LOG_INFO("Free heap before line extraction: %d", freeHeap);

    point_t p1 = (point_t) {.x = 100, .y = -200, .label = 1};
    point_t p2 = (point_t) {.x = 200, .y = -200, .label = 1};
    point_t p3 = (point_t) {.x = 300, .y = -200, .label = 1};
    point_t p4 = (point_t) {.x = 200, .y = -200, .label = 1};
    point_t p5 = (point_t) {.x = 300, .y = -200, .label = 1};
    point_t p6 = (point_t) {.x = 450, .y = -200, .label = 1};
    point_t p7 = (point_t) {.x = 450, .y = -200, .label = 1};
    point_t p8 = (point_t) {.x = 500, .y = -200, .label = 1};

    point_buffer_dynamic_t pb1;
    pb1.len = 2;
    pb1.buffer = pvPortMalloc(sizeof(point_t)*pb1.len);
    pb1.buffer[0] = p1;
    pb1.buffer[1] = p2;
    line_segment_t l1 = MSE_line_fit(pb1);

    point_buffer_dynamic_t pb2;
    pb2.len = 2;
    pb2.buffer = pvPortMalloc(sizeof(point_t)*pb2.len);
    pb2.buffer[0] = p3;
    pb2.buffer[1] = p4;
    line_segment_t l2 = MSE_line_fit(pb2);

    point_buffer_dynamic_t pb3;
    pb3.len = 2;
    pb3.buffer = pvPortMalloc(sizeof(point_t)*pb3.len);
    pb3.buffer[0] = p5;
    pb3.buffer[1] = p6;
    line_segment_t l3 = MSE_line_fit(pb3);

    point_buffer_dynamic_t pb4;
    pb4.len = 2;
    pb4.buffer = pvPortMalloc(sizeof(point_t)*pb4.len);
    pb4.buffer[0] = p7;
    pb4.buffer[1] = p8;
    line_segment_t l4 = MSE_line_fit(pb4);

    line_buffer.len = 4;

    line_buffer.buffer[0] = l1;
    line_buffer.buffer[0].points.len = 2;
    line_buffer.buffer[0].points.buffer = pvPortMalloc(sizeof(point_t)*2);
    line_buffer.buffer[0].points.buffer[0] = p1;
    line_buffer.buffer[0].points.buffer[1] = p2;

    vPortFree(pb1.buffer);

    line_buffer.buffer[1] = l2;
    line_buffer.buffer[1].points.len = 2;
    line_buffer.buffer[1].points.buffer = pvPortMalloc(sizeof(point_t)*2);
    line_buffer.buffer[1].points.buffer[0] = p3;
    line_buffer.buffer[1].points.buffer[1] = p4;

    vPortFree(pb2.buffer);

    line_buffer.buffer[2] = l3;
    line_buffer.buffer[2].points.len = 2;
    line_buffer.buffer[2].points.buffer = pvPortMalloc(sizeof(point_t)*2);
    line_buffer.buffer[2].points.buffer[0] = p5;
    line_buffer.buffer[2].points.buffer[1] = p6;

    vPortFree(pb3.buffer);

    line_buffer.buffer[3] = l4;
    line_buffer.buffer[3].points.len = 2;
    line_buffer.buffer[3].points.buffer = pvPortMalloc(sizeof(point_t)*2);
    line_buffer.buffer[3].points.buffer[0] = p7;
    line_buffer.buffer[3].points.buffer[1] = p8;

    vPortFree(pb4.buffer);
    
    merge_linebuffer(&line_buffer, 45*DEG2RAD, 60);

    for (int i=0; i<line_buffer.len; i++) {
      mqttsn_line_msg_t msg;
      msg.startPoint = (coordinate_t) {.x = line_buffer.buffer[i].start.x, .y = line_buffer.buffer[i].start.y};
      msg.endPoint = (coordinate_t) {.x = line_buffer.buffer[i].end.x, .y = line_buffer.buffer[i].end.y };
      msg.sigma_r2 = 0;
      msg.sigma_theta2 = 0;
      msg.sigma_rtheta = 0;
      publish_line("v2/robot/NRF_5/line", msg, sizeof(mqttsn_line_msg_t), 0, 0);
      TickType_t lastWakeTime = xTaskGetTickCount();
      vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
      vPortFree(line_buffer.buffer[i].points.buffer);
    }

    freeHeap = xPortGetFreeHeapSize();
    NRF_LOG_INFO("Free heap after line extraction: %d", freeHeap);

    vTaskSuspend(NULL);

  }*/
  
  /*while(1) {

    point_t p1 = (point_t) {.x = 200, .y = 200, .label = 1};
    point_t p2 = (point_t) {.x = 210, .y = 0, .label = 1};
    point_t p3 = (point_t) {.x = 220, .y = -10, .label = 1};
    point_t p4 =(point_t) {.x = 240, .y = -200, .label = 1};
    line_segment_t l1;
    l1.points.len = 0;
    l1.points.buffer = NULL;
    
    line_segment_t l2;
    l2.points.len = 2;
    l2.points.buffer = pvPortMalloc(sizeof(point_t)*l2.points.len);
    l2.points.buffer[0] = p1;
    l2.points.buffer[1] = p2;
    l2.start = p1;
    l2.end = p2;

    line_segment_t l3;
    l3.points.len = 2;
    l3.points.buffer = pvPortMalloc(sizeof(point_t)*l3.points.len);
    l3.points.buffer[0] = p3;
    l3.points.buffer[1] = p4;
    l3.start = p3;
    l3.end = p4;
    
    int cluster_id = 0;

    join_line_segments(cluster_id, &l1, &l2, &l3);

    mqttsn_line_msg_t msg_line;
    int msg_identifier = 0;
    msg_line.identifier = msg_identifier;
    msg_line.startPoint = (coordinate_t){.x = l2.start.x, .y = l2.start.y};
    msg_line.endPoint = (coordinate_t) {.x = l2.end.x, .y = l2.end.y};
    msg_line.sigma_r2 = 0;
    msg_line.sigma_rtheta = 0;
    msg_line.sigma_theta2 = 0;
    publish_line("v2/robot/NRF_5/merge", msg_line, sizeof(mqttsn_line_msg_t), 0, 0);
    TickType_t lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);

    mqttsn_line_msg_t msg_nextLine;
    msg_nextLine.identifier = msg_identifier;
    msg_nextLine.startPoint = (coordinate_t){.x = l3.start.x, .y = l3.start.y};
    msg_nextLine.endPoint = (coordinate_t) {.x = l3.end.x, .y = l3.end.y};
    msg_nextLine.sigma_r2 = 0;
    msg_nextLine.sigma_rtheta = 0;
    msg_nextLine.sigma_theta2 = 0;
    publish_line("v2/robot/NRF_5/merge", msg_nextLine, sizeof(mqttsn_line_msg_t), 0, 0);
    lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);

    mqttsn_line_msg_t msg_joint;
    msg_joint.identifier = msg_identifier;
    msg_joint.startPoint = (coordinate_t){.x = l1.start.x, .y = l1.start.y};
    msg_joint.endPoint = (coordinate_t) {.x = l1.end.x, .y = l1.end.y};
    msg_joint.sigma_r2 = 1;
    msg_joint.sigma_rtheta = 1;
    msg_joint.sigma_theta2 = 1;
    publish_line("v2/robot/NRF_5/merge", msg_joint, sizeof(mqttsn_line_msg_t), 0, 0);
    lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
    
    vTaskSuspend(NULL);
  }*/


  // Block until sensor tower task has initialized ir measurement queue
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  NRF_LOG_INFO("mapping task initialized");

  

  while(1) {
      
      
      /* Receive ir sensor measurement + robot pose from sensor tower task */
      /*if (xQueueReceive(ir_measurement_queue, &(new_measurement), (TickType_t) 10) == pdPASS) {
          update_point_buffers(&point_buffers, new_measurement);
          update_msg.xdelta = new_measurement.x;
          update_msg.ydelta = new_measurement.y;
          update_msg.thetadelta = new_measurement.theta;
          update_msg.ir1 = (coordinate_t) {
            .x = point_buffers[0].buffer[point_buffers[0].len-1].x,
            .y = point_buffers[0].buffer[point_buffers[0].len-1].y
          };
          update_msg.ir2 = (coordinate_t) {
            .x = point_buffers[1].buffer[point_buffers[1].len-1].x,
            .y = point_buffers[1].buffer[point_buffers[1].len-1].y
          };
          update_msg.ir3 = (coordinate_t) {
            .x = point_buffers[2].buffer[point_buffers[2].len-1].x,
            .y = point_buffers[2].buffer[point_buffers[2].len-1].y
          };
          update_msg.ir4 = (coordinate_t) {
            .x = point_buffers[3].buffer[point_buffers[3].len-1].x,
            .y = point_buffers[3].buffer[point_buffers[3].len-1].y
          };
          update_msg.valid = 0x0f;
          //publish_update("v2/robot/NRF_5/adv", update_msg, sizeof(update_msg), 0, 0);


      }*/
      
      /* Start line extraction */
      if (1/*ulTaskNotifyTake(pdTRUE, (TickType_t) 0) == pdPASS*/) {
        
        NRF_LOG_INFO("Start line extraction");

        for (int i=0; i<NUM_DIST_SENSORS; i++) {
          for (int j=0; j<point_buffers[i].len; j++) {
            mqttsn_cluster_msg_t msg;
            msg.cluster_id = i;
            msg.point.x = point_buffers[i].buffer[j].x;
            msg.point.y = point_buffers[i].buffer[j].y;
            publish_cluster_point("v2/robot/NRF_5/point", msg, sizeof(mqttsn_cluster_msg_t), 0, 0);
            TickType_t lastWakeTime = xTaskGetTickCount();
            vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
          }
          
        
        }

        int freeHeap = xPortGetFreeHeapSize();
        NRF_LOG_INFO("Free heap before line extraction: %d", freeHeap);

        int dbscan_msg_cluster_id = 0;
        int iepf_msg_cluster_id = 0;
        int mse_cluster_id = 0;
        for (int i=0; i<NUM_DIST_SENSORS; i++) {
          //NRF_LOG_INFO("DBSCAN #%d", i);
          
          // First filtering - Find points near each other
          cluster_buffer_t clusters = DBSCAN(&point_buffers[i], euclidean_distance, 20, 5); // Allocates memory on heap // 20, 5
          for (int j = 0; j < clusters.len; j++) {
            for (int k=0; k < clusters.buffer[j].len; k++) {
              dbscan_msg.cluster_id = dbscan_msg_cluster_id;
              dbscan_msg.point = (coordinate_t) {.x = clusters.buffer[j].buffer[k].x, .y = clusters.buffer[j].buffer[k].y};
              publish_cluster_point("v2/robot/NRF_5/DBSCAN", dbscan_msg, sizeof(dbscan_msg), 0, 0);
              TickType_t lastWakeTime = xTaskGetTickCount();
              vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
            }
            dbscan_msg_cluster_id++;
          
          }
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after DBSCAN: %d", freeHeap);

          if (clusters.len <= 0) {
            continue; // Go to next ir sensor readings
          }

          point_buffers[i].len = 0;
        
          // Second filtering - Find clusters of points which make up line segments
          for (int j=0; j<clusters.len; j++) {
            cluster_buffer_t line_clusters;
            line_clusters.len = 0;
            line_clusters.buffer = NULL;
            if (clusters.buffer[j].len < 2) {
              continue;
            }
          
            // No need for cluster means, use the DBSCAN clusters directly
            IEPF(clusters.buffer[j], &line_clusters, 10); // Allocates memory on heap

            for (int k=0; k<line_clusters.len; k++) {
              for (int l=0; l<line_clusters.buffer[k].len; l++) {
                // Publish IEPF-based clustered points
                iepf_msg.cluster_id = iepf_msg_cluster_id;
                iepf_msg.point = (coordinate_t) {.x = line_clusters.buffer[k].buffer[l].x, .y = line_clusters.buffer[k].buffer[l].y};
                publish_cluster_point("v2/robot/NRF_5/IEPF", iepf_msg, sizeof(iepf_msg), 0, 0);
                TickType_t lastWakeTime = xTaskGetTickCount();
                vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
              }
              iepf_msg_cluster_id++;
            
            }
            
            // Third filtering - Least-square line fitting using line clusters found in previous step
            for (int k=0; k<line_clusters.len; k++) {
              if (line_clusters.buffer[k].len >= 2 && line_buffer.len < LB_MAX_SIZE) {
                line_segment_t fitted_line = MSE_line_fit(line_clusters.buffer[k]);
                copy_points_to_line_segment(&(line_buffer.buffer[line_buffer.len]), line_clusters.buffer[k]);
                for (int l=0; l<line_buffer.buffer[line_buffer.len].points.len; l++) {
                  mqttsn_cluster_msg_t mse_point;
                  mse_point.cluster_id = mse_cluster_id;
                  mse_point.point.x = line_buffer.buffer[line_buffer.len].points.buffer[l].x;
                  mse_point.point.y = line_buffer.buffer[line_buffer.len].points.buffer[l].y;
                  publish_cluster_point("v2/robot/NRF_5/mse_point", mse_point, sizeof(mqttsn_cluster_msg_t), 0, 0);
                  TickType_t lastWakeTime = xTaskGetTickCount();
                  vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);
                }
                line_buffer.buffer[line_buffer.len].start = fitted_line.start;
                line_buffer.buffer[line_buffer.len].end = fitted_line.end;
                line_buffer.buffer[line_buffer.len].r = fitted_line.r;
                line_buffer.buffer[line_buffer.len].theta = fitted_line.theta;
                line_buffer.len++;

                msg.startPoint = (coordinate_t) {.x = fitted_line.start.x, .y = fitted_line.start.y};
                msg.endPoint = (coordinate_t) {.x = fitted_line.end.x, .y = fitted_line.end.y};
                msg.sigma_r2 = 0;
                msg.sigma_theta2 = 0;
                msg.sigma_rtheta = 0;
                publish_line("v2/robot/NRF_5/MSE", msg, sizeof(mqttsn_line_msg_t), 0, 0);
                TickType_t lastWakeTime = xTaskGetTickCount();
                vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);


              }
              mse_cluster_id++;
        
            }



            deallocate_cluster_buffer(line_clusters); // Free heap memory
            line_clusters.len = 0;
            freeHeap = xPortGetFreeHeapSize();
            NRF_LOG_INFO("Free heap after deallocating line clusters: %d", freeHeap);

          }

          deallocate_cluster_buffer(clusters);
          freeHeap = xPortGetFreeHeapSize();
          NRF_LOG_INFO("Free heap after deallocating DBSCAN clusters: %d", freeHeap);
          
        }


        // Recursively merge lines of line_buffer
        merge_linebuffer(&line_buffer, 45*DEG2RAD, 60);

        for (int i=0; i<line_buffer.len; i++) {
          // Estimate line uncertainty
          float sigma_rho2 = 0;
          uint16_t N = line_buffer.buffer[i].points.len;
          for (int j=0; j<N; j++) {
            float x = line_buffer.buffer[i].points.buffer[j].x;
            float y = line_buffer.buffer[i].points.buffer[j].y;
            float s = line_buffer.buffer[i].points.buffer[j].label;
            float rho = x*cos(line_buffer.buffer[i].theta) + y*sin(line_buffer.buffer[i].theta) - line_buffer.buffer[i].r;
            sigma_rho2 += s * rho*rho;
          }

          float L = get_length((line_t){.P = line_buffer.buffer[i].start, .Q = line_buffer.buffer[i].end});
          float theta_N = atan2(line_buffer.buffer[i].end.y, line_buffer.buffer[i].end.x);
          float theta_1 = atan2(line_buffer.buffer[i].start.y, line_buffer.buffer[i].start.x);
          float x_off = (line_buffer.buffer[i].r / 2.0) * (tan(line_buffer.buffer[i].theta - theta_N) + tan(line_buffer.buffer[i].theta - theta_1)); 
          float sigma_r2 = 12*sigma_rho2*x_off*x_off / ((float)(L*L * N)) + 1.0 / ((float)N);
          float sigma_theta2 = (12*sigma_rho2 / ((float)(L*L * N))) * ((float)N - 1.0) / ((float)N + 1.0);
          float sigma_rtheta = -12*sigma_rho2*x_off / ((float)(L*L * N));
          line_buffer.buffer[i].sigma_r2 = sigma_r2;
          line_buffer.buffer[i].sigma_theta2 = sigma_theta2;
          line_buffer.buffer[i].sigma_rtheta = sigma_rtheta;
        }


        for (int i=0; i<line_buffer.len; i++) {
          msg.startPoint = (coordinate_t) {.x = line_buffer.buffer[i].start.x, .y = line_buffer.buffer[i].start.y};
          msg.endPoint = (coordinate_t) {.x = line_buffer.buffer[i].end.x, .y = line_buffer.buffer[i].end.y };
          msg.sigma_r2 = line_buffer.buffer[i].sigma_r2;
          msg.sigma_theta2 = line_buffer.buffer[i].sigma_theta2;
          msg.sigma_rtheta = line_buffer.buffer[i].sigma_rtheta;
          publish_line("v2/robot/NRF_5/line", msg, sizeof(mqttsn_line_msg_t), 0, 0);
          TickType_t lastWakeTime = xTaskGetTickCount();
          vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*0.1);

          vPortFree(line_buffer.buffer[i].points.buffer);
          line_buffer.buffer[i].points.len = 0;
        
        }




        
        freeHeap = xPortGetFreeHeapSize();
        NRF_LOG_INFO("Free heap after: %d", freeHeap);

        vTaskSuspend(NULL);
        
      } // ulTaskNotify [end]

      //lastWakeTime = xTaskGetTickCount();
      //vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  }
  
}