#include "positionEstimate.h"
#include "nrf_log.h"

position_estimate_t position_estimate;

SemaphoreHandle_t xPoseMutex = NULL;

void init_position_estimate() {
  xPoseMutex = xSemaphoreCreateMutex();
  if (xPoseMutex == NULL) {
    NRF_LOG_ERROR("Not enough heap memory for pose mutex");
  }
}

void get_position_estimate(position_estimate_t* pos_est)
{
    pos_est->heading = position_estimate.heading / 1000.0;
    pos_est->x    = position_estimate.x;
    pos_est->y     = position_estimate.y;
}

void set_position_estimate(position_estimate_t* pos_est)
{
    position_estimate.heading   = (int) pos_est->heading*1000;
    position_estimate.x         = pos_est->x;
    position_estimate.y         = pos_est->y;
}



void set_position_estimate_heading(float heading)
{
    position_estimate.heading = heading;
}

void set_position_estimate_x(float x)
{
    position_estimate.x = x;
}

void set_position_estimate_y(float y)
{
    position_estimate.y = y;
}

float get_position_estimate_heading()
{
    return position_estimate.heading;
}

float get_position_estimate_x()
{
    return position_estimate.x;
}

float get_position_estimate_y()
{
    return position_estimate.y;
}