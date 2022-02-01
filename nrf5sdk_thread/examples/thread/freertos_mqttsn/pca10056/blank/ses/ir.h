#ifndef _IR_H_
#define _IR_H_

#include <stdint.h>

/**@brief Number of samples to average with before returning reading. */
#define N_SAMPLES 4

/**@brief Used to specify which IR-sensor to read from. */
typedef enum {
	IR_SENSOR_1,
	IR_SENSOR_2,
	IR_SENSOR_3,
	IR_SENSOR_4
} IR_Sensor_t;

/**@brief ADC resolution for IR readings. */
typedef enum {
	IR_RESOLUTION_8,
	IR_RESOLUTION_10,
	IR_RESOLUTION_12
} IR_Resolution_t;

/**@brief Callback when the readings are ready. */
typedef void(* ir_cb_t)(uint16_t reading);

/**
 * @brief Function for init of IR.
 * @details This function sets up and configures the SAADC peripheral for use
 * with the IR-sensors.
 *
 * @param[in] resolution	ADC readings resolution. Removed 2020
 */
void ir_init();

/**
 * @brief Function for blocking reading of IR-sensor.
 * @details This function blocks until it returns a reading from the selcted
 * IR.
 *
 * @param[in] sensor	Which IR-sensor to read from.
 */
uint16_t ir_read_blocking(IR_Sensor_t sensor);

/**
 * @brief Function for reading of IR-sensor.
 * @details This function will call the user-supplied callback function when
 * readings have been performed, and DMA transferred them to buffer.
 *
 * @param[in] sensor	Which IR-sensor to read from.
 * @param[in] ir_cb		User-supplied callback function.
 */
void ir_read(IR_Sensor_t sensor, ir_cb_t ir_cb);

/**
 * @brief Function for performing calibration of ADC.
 * @details Unlike the native calibration function part of the SDK, this
 * function will not return until calibration has completed. This makes it
 * safer and easier to use.
 */
void ir_calibrate();


/**
 * @brief Function for converting IR-reading to millimeter
*
 * @param[in] reading Analog reading from sensor
 * 
 * @param[in] sensor	Which IR-sensor the reading corresponds to
 */
int16_t IrAnalogToMM(uint16_t reading, IR_Sensor_t sensor);


#endif /* _IR_H_ */