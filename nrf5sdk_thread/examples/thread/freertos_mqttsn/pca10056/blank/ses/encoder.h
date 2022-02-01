/**
 * Unfortunately the encoders installed are not qudrature encoders (consider
 * upgrading), so the application developer will have to keep track of direction
 * when using this driver. E.g. consider the case of using the PPI version of
 * this driver: if there is a change of direction, the accumulator should be
 * cleared and accounted for before continue on counting.
 * 
 * The GPIOs will be configured as high-resolution, as power saving is not
 * used anyway.
 */
#ifndef _ENCODER_H_
#define _ENCODER_H_
#include "nrfx_gpiote.h"
#include "nrfx_timer.h"

/**@brief Enum describing valid sides Left/Right on robot. */
typedef enum {
	ENCODER_LEFT,
	ENCODER_RIGHT
}EncoderSide_t;


typedef struct {
    long int left;
    long int right;
}encoderTicks;

/**
 * @brief Function for init an encoder with a supplied timer instance.
 * @details This function associates an encoder with a supplied timer.
 * The application developer can then at any time read the count by
 * capturing the counter-value to a channel, and then read the channel:
 * 1. nrfx_timer_capture(counter, NRF_TIMER_CC_CHANNEL0);
 * 2. nrfx_timer_capture_get(counter, NRF_TIMER_CC_CHANNEL0)
 * In the example the count is saved to channel 0, and afterward read.
 *
 * @param[in] side		Left or right encoder.
 * @param[in] counter	Timer to perform the counting via PPI.
 */
//void encoder_init_ppi(EncoderSide_t side, const nrfx_timer_t* counter);
 
/**
* @brief Associate encoder events with a supplied callback function.
* @details This function associates an encoder with a supplied callback
* function. While this may require the processor to intervene more, it does 
* not need any timer peripherals.
*
* @param[in] side		Left or right encoder.
* @param[in] cb_fn		Callback function, format: void fn(void)
*
*/
void encoder_init_int();

encoderTicks encoder_get_ticks_since_last_time();


encoderTicks encoder_get_all_ticks();
#endif /* _ENCODER_H_ */