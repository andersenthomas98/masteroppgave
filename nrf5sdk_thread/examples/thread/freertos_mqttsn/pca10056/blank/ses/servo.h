#ifndef _SERVO_H_
#define _SERVO_H_

/**
 * @brief Function for initializing servo driver.
 */
void servo_init();

/**
 * @brief Function for changing servo position.
 * @details The raw duty cycle is given in percentage. Using the library instead
 * of the raw drivers for this resulted in poor resolution. Consider migrating
 * away from the library, and go as low as to the hardware abstraction layer
 * (HAL) to obtain more acurate control.
 *
 * @param[in] duty	  The duty cycle, which translates to angle.
 */
void vServo_setAngle(int angle);


#endif /* _SERVO_H_ */