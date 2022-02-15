#include "encoder_with_counter.h"

#include <stdbool.h>

#include "nrf.h"

#include "nrf_drv_gpiote.h"

#include "nrf_drv_ppi.h"

#include "nrf_drv_timer.h"

#include "nrf_delay.h"

#include "app_error.h"

#include "boards.h"

#include "nrf_log.h"

#include "globals.h"

#include <stdint.h>

#include <stdio.h>

#include "robot_config.h"

//Define encoder pins as described from PCB pinout (added)
/*
#define ENCODER_PIN_LEFT_1 39
#define ENCODER_PIN_LEFT_2 40

#define ENCODER_PIN_RIGHT_1 42
#define ENCODER_PIN_RIGHT_2 43
*/
#define ENCODER_PIN_LEFT_1 42
#define ENCODER_PIN_LEFT_2 43

#define ENCODER_PIN_RIGHT_1 39
#define ENCODER_PIN_RIGHT_2 40

static
const nrf_drv_timer_t m_timer_count_left = NRF_DRV_TIMER_INSTANCE(3);
static
const nrf_drv_timer_t m_timer_count_right = NRF_DRV_TIMER_INSTANCE(4);
nrf_ppi_channel_t ppi_channel_1, ppi_channel_2;

MOTOR_DIRECTION left_direction, right_direction;
uint32_t left_counter_value, right_counter_value;

encoderTicks encoderCnt = {
  .left = 0,
  .right = 0,
};

encoderTicks encoderCnt_old = {
  .left = 0,
  .right = 0,
};

//Check which way the motor is going 
void in_pin_handler_left(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  if (nrf_gpio_pin_read(ENCODER_PIN_LEFT_2) > 0) {
    left_direction = ENCODER_PIN_LEFT_2_HIGH_DIRECTION;
  } else {
    left_direction = ENCODER_PIN_LEFT_2_LOW_DIRECTION;
  }
}
//Check which way the motor is going 
void in_pin_handler_right(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  if (nrf_gpio_pin_read(ENCODER_PIN_RIGHT_2) > 0) {
    right_direction = ENCODER_PIN_RIGHT_2_HIGH_DIRECTION;
  } else {
    right_direction = ENCODER_PIN_RIGHT_2_LOW_DIRECTION;

  }
}

//Not used
void dir_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  //void
  if (PRINT_DEBUG) printf("dir pin handler\n\r");
}

void timer_handler_count_left(nrf_timer_event_t event_type, void * p_context) {
  //void
  if (PRINT_DEBUG) printf("left cnt handler\n\r");
}

void timer_handler_count_right(nrf_timer_event_t event_type, void * p_context) {
  //void
  if (PRINT_DEBUG) printf("right cnt handler\n\r");
}

void encoder_with_counter_init() {

  left_direction = right_direction = FORWARD;

  ret_code_t err_code;

  if (!nrf_drv_gpiote_is_init()) {
    NRF_LOG_INFO("Init gpiote\n\r");
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }

  //Use secondary encoder sensors for count
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
  in_config.pull = NRF_GPIO_PIN_PULLDOWN;

  //Configure GPIO pins
  err_code = nrf_drv_gpiote_in_init(ENCODER_PIN_LEFT_2, & in_config, dir_pin_handler);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_gpiote_in_init(ENCODER_PIN_RIGHT_2, & in_config, dir_pin_handler);
  APP_ERROR_CHECK(err_code);

  //Configure count pins
  in_config.pull = NRF_GPIO_PIN_PULLDOWN;
  in_config.hi_accuracy = true; //If not enabled, port event address is uses for ppi This will make a encoder tick count both left and right
  in_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;

  err_code = nrf_drv_gpiote_in_init(ENCODER_PIN_LEFT_1, & in_config, in_pin_handler_left);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_gpiote_in_init(ENCODER_PIN_RIGHT_1, & in_config, in_pin_handler_right);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(ENCODER_PIN_LEFT_1, true);
  nrf_drv_gpiote_in_event_enable(ENCODER_PIN_RIGHT_1, true);

  //Configure counters
  nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
  timer_cfg.mode = NRF_TIMER_MODE_LOW_POWER_COUNTER;

  err_code = nrf_drv_timer_init( & m_timer_count_left, & timer_cfg, timer_handler_count_left);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_timer_init( & m_timer_count_right, & timer_cfg, timer_handler_count_right);
  APP_ERROR_CHECK(err_code);
  //nrf_drv_timer_enable(&m_timer_count_left);
  //nrf_drv_timer_enable(&m_timer_count_right);

  err_code = nrf_drv_ppi_init();
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_ppi_channel_alloc( & ppi_channel_1);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_ppi_channel_alloc( & ppi_channel_2);
  APP_ERROR_CHECK(err_code);

  uint32_t gpiote_evt_addr_1 = nrf_drv_gpiote_in_event_addr_get(ENCODER_PIN_LEFT_1);
  uint32_t gpiote_evt_addr_2 = nrf_drv_gpiote_in_event_addr_get(ENCODER_PIN_RIGHT_1);

  uint32_t timer_count_count_left_task_addr = nrf_drv_timer_task_address_get( & m_timer_count_left, NRF_TIMER_TASK_COUNT);
  uint32_t timer_count_count_right_task_addr = nrf_drv_timer_task_address_get( & m_timer_count_right, NRF_TIMER_TASK_COUNT);

  err_code = nrf_drv_ppi_channel_assign(ppi_channel_1, gpiote_evt_addr_1, timer_count_count_left_task_addr); // Trigger timer count task when GPIOTE pin go from low to high.
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_ppi_channel_assign(ppi_channel_2, gpiote_evt_addr_2, timer_count_count_right_task_addr); // Trigger timer count task when GPIOTE pin go from low to high.
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_ppi_channel_enable(ppi_channel_1);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_ppi_channel_enable(ppi_channel_2);
  APP_ERROR_CHECK(err_code);
}

void update_encoder_ticks() {
  //Get new values
  uint32_t left_counter_value_new = nrf_drv_timer_capture( & m_timer_count_left, NRF_TIMER_CC_CHANNEL1); //nrf_drv_timer_capture_get(&m_timer_count_left, NRF_TIMER_CC_CHANNEL1); 
  uint32_t right_counter_value_new = nrf_drv_timer_capture( & m_timer_count_right, NRF_TIMER_CC_CHANNEL1); //nrf_drv_timer_capture_get(&m_timer_count_right, NRF_TIMER_CC_CHANNEL2);

  int32_t new_steps_left;
  int32_t new_steps_right;

  if (left_counter_value_new > left_counter_value) new_steps_left = left_counter_value_new - left_counter_value;
  else new_steps_left = 8589934592 - left_counter_value + left_counter_value_new; //Overflow of counter value 2^32 - prev value + new value

  if (right_counter_value_new > right_counter_value) new_steps_right = right_counter_value_new - right_counter_value;
  else new_steps_right = 8589934592 - right_counter_value + right_counter_value_new; //Overflow of counter value 2^32 - prev value + new value

  left_counter_value = left_counter_value_new;
  right_counter_value = right_counter_value_new;
  //Direction
  if (left_direction == BACKWARD) new_steps_left = -new_steps_left;

  if (right_direction == BACKWARD) new_steps_right = -new_steps_right;

  encoderCnt.left += new_steps_left;
  encoderCnt.right += new_steps_right;
}

encoderTicks encoder_with_counter_get_ticks_since_last_time() {
  encoderTicks ticks = {
    0,
    0
  };

  if (xSemaphoreTake(xTickBSem, 1) == pdTRUE) {
    update_encoder_ticks();

    //temporary store old values
    ticks.left = encoderCnt_old.left;
    ticks.right = encoderCnt_old.right;
    //update old values
    encoderCnt_old.left = encoderCnt.left; //nrf_drv_timer_capture_get(&m_timer_count_left, NRF_TIMER_CC_CHANNEL1); 
    encoderCnt_old.right = encoderCnt.right; //nrf_drv_timer_capture_get(&m_timer_count_right, NRF_TIMER_CC_CHANNEL2);
    //calculate ticks since last call
    ticks.left = encoderCnt_old.left - ticks.left;
    ticks.right = encoderCnt_old.right - ticks.right;

    xQueueSend(encoderTicksToMotorSpeedControllerQ, & ticks, 1);
    //	  xQueueSend(encoderTicksToMotorPositionControllerQ, &ticks, 0);
    xQueueSend(encoderTicksToEstimatorTaskQ, & ticks, 1);
    xSemaphoreGive(xTickBSem);
  } else {
    //printf("Semaphore not available\n\r");
  }

  return ticks;
}

encoderTicks encoder_with_counter_get_all_ticks() {
  encoderTicks ticks = {
    0,
    0
  };
  if (xSemaphoreTake(xTickBSem, 1) == pdTRUE) {
    update_encoder_ticks();

    ticks.left = encoderCnt.left;
    ticks.right = encoderCnt.right;
  } else {
    printf("Semaphore not available\n\r");

  }
  return ticks;
}