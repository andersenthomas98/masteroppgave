#include "servo.h"

#include "app_pwm.h"

#include "nrf_log.h"

#include "nrfx_gpiote.h"

#include "nrf_drv_pwm.h"

#include "nrf_drv_clock.h"

//TODO: Is this used? APP_PWM_INSTANCE(PWM2, 2);

#define PIN_SERVO 41

// This array cannot be allocated on stack (hence "static") and it must
// be in RAM (hence no "const", though its content is not changed).
uint16_t seq_values[] = {
  0x000 | (1 << 15),
};

static nrf_drv_pwm_t m_pwm_servo = NRF_DRV_PWM_INSTANCE(0);

void vServo_setAngle(int angle) {
  if (angle < -90 || angle > 90) {
    NRF_LOG_INFO("Servo angle outside range");
    return;
  }
  int ticks = 166 + (int)(2.33 * angle);
  //int ticks = 375 + (int)(1.5*angle); 
  /*
  was 1.5*angle, did not turn 90, 2.33 makes -90 go where it is supposed to
  but +90 overshoots and get stuck trying to force it beyond physical limit. 
  Edit: How to fix: reduce the offset and expand the multiplication factor (I don't have access to a robot at the moment)
  Edit: Fixed per 6.9.2021. Switched sensor direction in robot_config.h.
  */
  seq_values[0] = ticks | (1 << 15);
  //seq_values[1] = ticks  | (1 << 15);
}

void servo_init() {

  nrf_drv_pwm_config_t
  const config_servo = {
    .output_pins = {
      PIN_SERVO | NRF_DRV_PWM_PIN_INVERTED, // channel 0
      NRF_DRV_PWM_PIN_NOT_USED, // channel 1
      NRF_DRV_PWM_PIN_NOT_USED, // channel 2
      NRF_DRV_PWM_PIN_NOT_USED, // channel 3
    },
    .irq_priority = APP_IRQ_PRIORITY_LOWEST,
    .base_clock = NRF_PWM_CLK_250kHz, //NRF_PWM_CLK_125kHz, NRF_PWM_CLK_2MHz
    .count_mode = NRF_PWM_MODE_UP,
    .top_value = 5000, // = 0x8000
    .load_mode = NRF_PWM_LOAD_COMMON,
    .step_mode = NRF_PWM_STEP_AUTO
  };
  APP_ERROR_CHECK(nrf_drv_pwm_init( & m_pwm_servo, & config_servo, NULL));
  //m_used |= USED_PWM(0);

  nrf_pwm_sequence_t
  const seq = {
    .values.p_common = seq_values,
    .length = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats = 0,
    .end_delay = 0
  };

  nrf_gpio_cfg(
    PIN_SERVO,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_H0S1,
    NRF_GPIO_PIN_NOSENSE);

  (void) nrf_drv_pwm_simple_playback( & m_pwm_servo, & seq, 3, NRF_DRV_PWM_FLAG_LOOP);
}