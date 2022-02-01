#include "encoder.h"

#include "nrfx_ppi.h"

#include "nrf_delay.h"

#include "nrf_log.h"

#include "globals.h"

#include <stdint.h>

//Define encoder pins as described from PCB pinout (added)
/*
#define ENCODER_PIN_LEFT_1_1 39
#define ENCODER_PIN_LEFT_1_2 40

#define ENCODER_PIN_RIGHT_2_1 42
#define ENCODER_PIN_RIGHT_2_2 43
*/
#define ENCODER_PIN_LEFT_1_1 42
#define ENCODER_PIN_LEFT_1_2 43

#define ENCODER_PIN_RIGHT_2_1 39
#define ENCODER_PIN_RIGHT_2_2 40

encoderTicks encoder = {
  .left = 0,
  .right = 0,
};

encoderTicks encoder_old = {
  .left = 0,
  .right = 0,
};

void timer_event_handler_dummy(nrf_timer_event_t event_type, void * p_context) {
  // dummy
}

void encoder_cb_left() {

  if (nrf_gpio_pin_read(ENCODER_PIN_LEFT_1_2) > 0) {
    encoder.left++;
  } else {
    encoder.left--;
  }
};

void encoder_cb_right() {
  //If conditions may need to be switched, depending on direction of wheels/encoders

  if (nrf_gpio_pin_read(ENCODER_PIN_RIGHT_2_2) > 0) {
    encoder.right--;
  } else {
    encoder.right++;

  }
};

void cb_trampoline(nrfx_gpiote_pin_t pin,
  nrf_gpiote_polarity_t action) {
  //	NRF_LOG_WARNING("PIN interrrupt %d\n\r", (int)pin);
  if ((int) pin == ENCODER_PIN_LEFT_1_1)
    encoder_cb_left();
  else if ((int) pin == ENCODER_PIN_RIGHT_2_1)
    encoder_cb_right();
}

void encoder_init_int() {
  nrfx_err_t err;
  if (!nrfx_gpiote_is_init())
    err = nrfx_gpiote_init();

  nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  config.is_watcher = false;
  config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
  config.pull = NRF_GPIO_PIN_NOPULL;
  //Configure interrupt pins, interrupts on LO to HIGH
  err = nrfx_gpiote_in_init(ENCODER_PIN_LEFT_1_1, & config, cb_trampoline);
  nrfx_gpiote_in_event_enable(ENCODER_PIN_LEFT_1_1, true);
  err = nrfx_gpiote_in_init(ENCODER_PIN_RIGHT_2_1, & config, cb_trampoline);
  nrfx_gpiote_in_event_enable(ENCODER_PIN_RIGHT_2_1, true);
  //Configure the two other pins as input pins
  nrf_gpio_cfg_input(ENCODER_PIN_LEFT_1_2, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(ENCODER_PIN_RIGHT_2_2, NRF_GPIO_PIN_NOPULL);

  APP_ERROR_CHECK(err);
}

encoderTicks encoder_get_ticks_since_last_time() {
  encoderTicks ticks;

  //temporary store old values
  ticks.left = encoder_old.left;
  ticks.right = encoder_old.right;
  //update old values
  encoder_old.left = encoder.left;
  encoder_old.right = encoder.right;
  //calculate ticks since last call
  ticks.left = encoder_old.left - ticks.left;
  ticks.right = encoder_old.right - ticks.right;

  xQueueSend(encoderTicksToMotorSpeedControllerQ, & ticks, 10);
  xQueueSend(encoderTicksToMotorPositionControllerQ, & ticks, 100);
  xQueueSend(encoderTicksToEstimatorTaskQ, & ticks, 10);

  return ticks;
}

encoderTicks encoder_get_all_ticks() {
  encoderTicks ticks;
  ticks.left = encoder.left;
  ticks.right = encoder.right;
  return ticks;
}