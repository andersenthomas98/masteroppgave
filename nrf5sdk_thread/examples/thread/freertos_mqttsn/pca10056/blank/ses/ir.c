#include "ir.h"

#include "nrfx_saadc.h"

#include "nrf_log.h"

#include "nrf_delay.h"

#include "math.h"

#include "defines.h"

#include "app_error.h"

#include "robot_config.h"

/*
pinout: PCB to NRF52
IR1 TOP LEFT    
IR2 TOP RIGHT  
IR3 BOTTOM LEFT
IR4 BOTTOM RIGHT

*/

static
const nrfx_saadc_config_t saadc_config = {
  .resolution = NRF_SAADC_RESOLUTION_12BIT,
  .oversample = NRF_SAADC_OVERSAMPLE_DISABLED, //oversampling should not be used in scan mode, if more than 1 channel is enabled the adc enters scan mode
  .interrupt_priority = 7,
  .low_power_mode = false
};

bool calibration_completed = false;
bool initial_calibration_completed = false;

ir_cb_t cb;

nrf_saadc_value_t conversion_buffer[N_SAMPLES];

void saadc_event_handler(nrfx_saadc_evt_t
  const * p_event) {
  nrfx_err_t err;
  switch (p_event -> type) {
  case NRFX_SAADC_EVT_DONE:
    NRF_LOG_INFO("SAADC_EVT_DONE");
    err = nrfx_saadc_buffer_convert(p_event -> data.done.p_buffer,
      N_SAMPLES);
    APP_ERROR_CHECK(err);

    uint16_t average = p_event -> data.done.p_buffer[0] +
      p_event -> data.done.p_buffer[1] +
      p_event -> data.done.p_buffer[2] +
      p_event -> data.done.p_buffer[3];
    cb(average);
    ///< Event generated when the buffer is filled with samples.
    break;
  case NRFX_SAADC_EVT_LIMIT:
    NRF_LOG_INFO("SAADC_EVT_LIMIT");
    //p_event->data.limit.channel;
    if (p_event -> data.limit.limit_type == NRF_SAADC_LIMIT_LOW ||
      p_event -> data.limit.limit_type == NRF_SAADC_LIMIT_HIGH);
    ///< Event generated after one of the limits is reached.
    break;
  case NRFX_SAADC_EVT_CALIBRATEDONE:
    NRF_LOG_INFO("SAADC_EVT_CALIBRATEDONE");
    if (initial_calibration_completed != true)
      initial_calibration_completed = true;
    else
      calibration_completed = true;
    break;
  default:
    break;
  }
};

void ir_init() {
  nrfx_err_t err = NRFX_SUCCESS;

  //channel configurations
  //Channel 0 IR1
  static nrf_saadc_channel_config_t channel_0_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(IR_SENSOR_1_ANALOG_INPUT);
  channel_0_config.gain = NRF_SAADC_GAIN1_4;
  channel_0_config.reference = NRF_SAADC_REFERENCE_VDD4;

  //channel 1 IR2
  static nrf_saadc_channel_config_t channel_1_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(IR_SENSOR_2_ANALOG_INPUT);
  channel_1_config.gain = NRF_SAADC_GAIN1_4;
  channel_1_config.reference = NRF_SAADC_REFERENCE_VDD4;

  //channel 2 IR3
  static nrf_saadc_channel_config_t channel_2_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(IR_SENSOR_3_ANALOG_INPUT);
  channel_2_config.gain = NRF_SAADC_GAIN1_4;
  channel_2_config.reference = NRF_SAADC_REFERENCE_VDD4;

  //channel 3 IR4
  static nrf_saadc_channel_config_t channel_3_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(IR_SENSOR_4_ANALOG_INPUT);
  channel_3_config.gain = NRF_SAADC_GAIN1_4;
  channel_3_config.reference = NRF_SAADC_REFERENCE_VDD4;

  err = nrfx_saadc_init( & saadc_config, saadc_event_handler);
  APP_ERROR_CHECK(err);

  err = nrfx_saadc_channel_init(0, & channel_0_config);
  APP_ERROR_CHECK(err);

  err = nrfx_saadc_channel_init(1, & channel_1_config);
  APP_ERROR_CHECK(err);

  err = nrfx_saadc_channel_init(2, & channel_2_config);
  APP_ERROR_CHECK(err);

  err = nrfx_saadc_channel_init(3, & channel_3_config);
  APP_ERROR_CHECK(err);
}

void ir_read(IR_Sensor_t sensor, ir_cb_t ir_cb) {
  nrfx_err_t err;

  cb = ir_cb;

  for (int i = 0; i < N_SAMPLES; i++) {
    err = nrfx_saadc_sample();
    // event handler interrupts here
    APP_ERROR_CHECK(err);
  }
}

uint16_t ir_read_blocking(IR_Sensor_t sensor) {
  nrfx_err_t err;
  uint8_t channel = sensor;

  static nrf_saadc_value_t val[N_SAMPLES];
  int acc = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    err = nrfx_saadc_sample_convert(channel, val + i);
    APP_ERROR_CHECK(err);
    acc += val[i];

  }
  return (uint16_t)(acc / N_SAMPLES);
}

void ir_calibrate() {
  if (nrfx_saadc_calibrate_offset() != NRFX_SUCCESS) {
    NRF_LOG_INFO("nrfx_saadc_calibration_offset() failed.");
    return;
  }

  while (!calibration_completed);;
  calibration_completed = false;
}

// The calibration data for IR3 and IR4 has been switched compared to the data presented in Leithes Thesis.
int16_t IrAnalogToMM(uint16_t reading, IR_Sensor_t sensor) {
  //Should be calibrated again using the excel method
  int16_t result = 0;
  float result_float = 0;
  // Cap by 1270 to make conversion to cm (127 * 10) safe from integer overflow
  // for both int8_t and int16_t
  const float result_max = 1270;

  if (sensor == IR_SENSOR_1) {
    result_float = IR_SENSOR_1_DIVIDER * pow(reading, IR_SENSOR_1_EXPONENT);
  }
  if (sensor == IR_SENSOR_2) {
    result_float = IR_SENSOR_2_DIVIDER * pow(reading, IR_SENSOR_2_EXPONENT);
  }
  if (sensor == IR_SENSOR_3) {
    result_float = IR_SENSOR_3_DIVIDER * pow(reading, IR_SENSOR_3_EXPONENT);
  }
  if (sensor == IR_SENSOR_4) {
    result_float = IR_SENSOR_4_DIVIDER * pow(reading, IR_SENSOR_4_EXPONENT);
  }

  result = (int16_t) fmin(ceil(result_float), result_max);
  return result;
}