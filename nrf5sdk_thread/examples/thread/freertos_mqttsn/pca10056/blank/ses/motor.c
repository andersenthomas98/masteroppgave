#include "motor.h"

#include "nrfx_gpiote.h"

#include "nrf_drv_pwm.h"

#include "nrf_drv_clock.h"

#include "nrf.h"

#include "nrf_log.h"

#include "app_error.h"

#include "bsp.h"

#include "nrf_delay.h"

#include <stdint.h>

#include "robot_config.h"

#define PIN_IN1 34 //32
#define PIN_IN2 35 //33
#define PIN_IN3 32
#define PIN_IN4 33
#define PIN_ENA 36
#define PIN_ENB 37
#define MOTOR_CHANNEL_RIGHT 0
#define MOTOR_CHANNEL_LEFT 1
#define MOTOR_FORWARDS 1
#define MOTOR_BACKWARDS - 1

#define MOTOR_PWM_CLK_CFG NRF_PWM_CLK_250kHz
#define MOTOR_PWM_CLK_Hz 250000 //Must match MOTOR_PWM_CLK_CFG 

#define MOTOR_PWM_PEROID_MS 20
#define MOTOR_PWM_TOP (MOTOR_PWM_CLK_Hz / 1000) * MOTOR_PWM_PEROID_MS

extern int RightMotorDirection;
extern int LeftMotorDirection;

static nrf_drv_pwm_t m_pwm_motor = NRF_DRV_PWM_INSTANCE(2); // Create the instance "PWM2" using TIMER2. Changed from using TIMER1(used by thread) to use TIMER2

// This array cannot be allocated on stack (hence "static") and it must
// be in RAM (hence no "const", though its content is not changed).
static nrf_pwm_values_grouped_t /*const*/ pwm_motor_seq_values[] = {
  {
    0 | (1 << 15), 0 | (1 << 15)
  },
};

void pwm_ready_callback(uint32_t pwm_id) // PWM callback function
{
  //
}

void set_left_motor_duty(uint32_t duty) {
  //Calculate count value
  uint32_t new_trigg_value = (MOTOR_PWM_TOP * duty) / 100;
  pwm_motor_seq_values -> group_0 = new_trigg_value | (1 << 15); // 1<<15 used for inverting duty
}

void set_right_motor_duty(uint32_t duty) {
  uint32_t new_trigg_value = (MOTOR_PWM_TOP * duty) / 100;
  pwm_motor_seq_values -> group_1 = new_trigg_value | (1 << 15);
}

static inline int SATURATE_DUTY(int duty) {
  if (duty < 0)
    duty = 0;
  if (duty > MOTOR_PWM_MAX_DUTY)
    duty = MOTOR_PWM_MAX_DUTY;
  return duty;
}

void motor_forward(int duty) {
  nrfx_gpiote_out_set(PIN_IN1);
  nrfx_gpiote_out_clear(PIN_IN2);
  nrfx_gpiote_out_set(PIN_IN3);
  nrfx_gpiote_out_clear(PIN_IN4);
  duty = SATURATE_DUTY(duty); //Make sure duty does not exceed 50% (this might damage motor)

  set_right_motor_duty(duty);
  set_left_motor_duty(duty);
}

void motor_right_forward(int duty) {
  nrfx_gpiote_out_set(PIN_IN1);
  nrfx_gpiote_out_clear(PIN_IN2);
  duty = SATURATE_DUTY(duty);
  set_right_motor_duty(duty);

}

void motor_left_forward(int duty) {
  nrfx_gpiote_out_set(PIN_IN3);
  nrfx_gpiote_out_clear(PIN_IN4);
  duty = SATURATE_DUTY(duty);

  set_left_motor_duty(duty);
}

void motor_backward(int duty) {
  nrfx_gpiote_out_clear(PIN_IN1);
  nrfx_gpiote_out_set(PIN_IN2);
  nrfx_gpiote_out_clear(PIN_IN3);
  nrfx_gpiote_out_set(PIN_IN4);
  duty = SATURATE_DUTY(duty);

  set_right_motor_duty(duty);
  set_left_motor_duty(duty);
}

void motor_right_backward(int duty) {
  nrfx_gpiote_out_clear(PIN_IN1);
  nrfx_gpiote_out_set(PIN_IN2);
  duty = SATURATE_DUTY(duty);

  set_right_motor_duty(duty);
}

void motor_left_backward(int duty) {
  nrfx_gpiote_out_clear(PIN_IN3);
  nrfx_gpiote_out_set(PIN_IN4);
  duty = SATURATE_DUTY(duty);
  set_left_motor_duty(duty);
}

void motor_right_stop() {
  set_right_motor_duty(0);
}

void motor_left_stop() {
  set_left_motor_duty(0);
}

void motor_stop() {
  motor_right_stop();
  motor_left_stop();
}

void motor_brake() {
  nrfx_gpiote_out_set(PIN_IN1);
  nrfx_gpiote_out_set(PIN_IN2);
  nrfx_gpiote_out_set(PIN_IN3);
  nrfx_gpiote_out_set(PIN_IN4);
}

void motor_brake_left() {
  nrfx_gpiote_out_set(PIN_IN3);
  nrfx_gpiote_out_set(PIN_IN4);
  //	while (app_pwm_channel_duty_set(&PWM1, 1, 0) == NRF_ERROR_BUSY);
}

void motor_brake_right() {
  nrfx_gpiote_out_set(PIN_IN1);
  nrfx_gpiote_out_set(PIN_IN2);
  //	while (app_pwm_channel_duty_set(&PWM1, 0, 0) == NRF_ERROR_BUSY);
}

void vMotorMovementSwitch(int leftSpeed, int rightSpeed) {
  if (PRINT_DEBUG) NRF_LOG_INFO("Movement Switch L: %d \t R: %d", leftSpeed, rightSpeed);
  if (leftSpeed > 0) {
    motor_left_forward(leftSpeed);
  } else if (leftSpeed < 0) {
    motor_left_backward(-leftSpeed);
  } else {
    motor_brake_left();
  }

  if (rightSpeed > 0) {
    motor_right_forward(rightSpeed);
  } else if (rightSpeed < 0) {
    motor_right_backward(-rightSpeed);
  } else {
    motor_brake_right();
  }
}

void motor_init() {

  //ret_code_t err_code;
  nrfx_err_t err = NRFX_SUCCESS;

  //GPIO
  if (!nrfx_gpiote_is_init())
    err = nrfx_gpiote_init();
  APP_ERROR_CHECK(err);

  //Set output pins in high drive mode to 
  nrf_gpio_cfg(
    PIN_IN1,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_H0S1, //High drive 0, standard drive 1
    NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(
    PIN_IN2,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_H0S1,
    NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(
    PIN_IN3,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_H0S1,
    NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(
    PIN_IN4,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_H0S1,
    NRF_GPIO_PIN_NOSENSE);

  //Motor PWM

  nrf_drv_pwm_config_t config = {
    // These are the common configuration options we use for all PWM
    // instances.
    .irq_priority = APP_IRQ_PRIORITY_LOWEST,
    .count_mode = NRF_PWM_MODE_UP,
    .step_mode = NRF_PWM_STEP_AUTO,
  };

  ////////////////////////////////////////////////////////////////////////////
  // PWM1 initialization.

  config.output_pins[0] = PIN_ENA | NRF_DRV_PWM_PIN_INVERTED;
  config.output_pins[1] = NRF_DRV_PWM_PIN_NOT_USED;
  config.output_pins[2] = PIN_ENB | NRF_DRV_PWM_PIN_INVERTED;
  config.output_pins[3] = NRF_DRV_PWM_PIN_NOT_USED;
  config.base_clock = MOTOR_PWM_CLK_CFG;
  config.top_value = MOTOR_PWM_TOP;
  config.load_mode = NRF_PWM_LOAD_GROUPED;
  APP_ERROR_CHECK(nrf_drv_pwm_init( & m_pwm_motor, & config, NULL));

  nrf_pwm_sequence_t
  const pwm_motor_seq = {
    .values.p_grouped = pwm_motor_seq_values,
    .length = NRF_PWM_VALUES_LENGTH(pwm_motor_seq_values),
    .repeats = 1,
    .end_delay = 0
  };

  nrf_gpio_cfg(
    PIN_ENA,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_H0S1,
    NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(
    PIN_ENB,
    NRF_GPIO_PIN_DIR_OUTPUT,
    NRF_GPIO_PIN_INPUT_DISCONNECT,
    NRF_GPIO_PIN_NOPULL,
    NRF_GPIO_PIN_H0S1,
    NRF_GPIO_PIN_NOSENSE);
  (void) nrf_drv_pwm_simple_playback( & m_pwm_motor, & pwm_motor_seq, 1,
    NRF_DRV_PWM_FLAG_LOOP);

}