#ifndef POSITION_ESTIMATE_H
#define POSITION_ESTIMATE_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t xPoseMutex;

typedef struct{

    float heading; // gTheta_hat
    float x;  //gX_hat
    float y;  //gY_hat
    float vel;
} position_estimate_t;

void init_position_estimate();

/**
 * @brief Get estimate of robot position x, y and theta
 *
 * @param[in] pos_est PositionEstimate struct pointer
 */
void get_position_estimate(position_estimate_t* pos_est);

/**
 * @brief Set estimate of robot position x, y and theta
 *
 * @param[in] pos_est PositionEstimate struct pointer
 */
void set_position_estimate(position_estimate_t* pos_est);


/**
 * @brief Set estimate of robot theta
 *
 * @param[in]  theta_hat Estimate of theta
 */
void set_position_estimate_heading(float heading);

/**
 * @brief Set estimate of robot x
 *
 * @param[in]  x_hat Estimate of theta
 */
void set_position_estimate_x(float x_hat);

/**
 * @brief Set estimate of robot y
 *
 * @param[in]  y_hat Estimate of y
 */
void set_position_estimate_y(float y_hat);


/**
 * @brief get estimate of robot theta
 *
 * @param[out]  theta_hat Estimate of theta
 */
float get_position_estimate_heading();

/**
 * @brief get estimate of robot x
 *
 * @param[out]  x_hat Estimate of theta
 */
float get_position_estimate_x();

/**
 * @brief get estimate of robot y
 *
 * @param[out]  y_hat Estimate of y
 */
float get_position_estimate_y();



#endif