#ifndef _ENCODER_COUNTER_H_
#define _ENCODER_COUNTER_H_
#include "nrfx_gpiote.h"
#include "nrfx_timer.h"
#include "FreeRTOS.h"
#include "queue.h"

/*
typedef struct {
    long int left;
    long int right;
}encoderTicks; */
#include "encoder.h"

extern QueueHandle_t encoderTicksToEstimatorTaskQ, encoderTicksToMotorSpeedControllerQ;


/**
* @brief Initiate encoder driver with tomer/counter/PPI and interrupts
* @details This driver uses counters to count the number of encoder ticks and
* interrupt to decide the rotating direction
*
*
*/
void encoder_with_counter_init();

/**
* @brief Get updated encoder ticks
* @details Also pushes the encoder ticks to queues that can be read by FreeRTOS tasks 
*
* @param[out] encoderTicks		Number of encoder ticks since last time the function was called
*
*/
encoderTicks encoder_with_counter_get_ticks_since_last_time();

/**
* @brief Get total number of encoder ticks
*
* @param[out] encoderTicks		Total number of encoder ticks since startup
*
*/
encoderTicks encoder_with_counter_get_all_ticks();
#endif /* _ENCODER_H_ */