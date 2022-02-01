/************************************************************************/
// File:            ICM_20948.h                                           //
// Author:                                                              //
// Purpose:         IMU driver                                          //
//                                                                      //
/************************************************************************/

#include "ICM_20948_REGISTERS.h"

#include "ICM_20948.h"

#include "i2c.h"

#include "stdint.h"

#include <inttypes.h>

static volatile uint8_t IMU_buff[14]; //Accel, gyro, temp and 9 bytes of mag

// Project functions
uint8_t IMU_read_byte(uint8_t reg) {
  uint8_t ret;
  i2c_recive(ICM_ADDR_AD1, reg, & ret, 1);
  return ret;
}

void IMU_read_burst(uint8_t reg, uint8_t * storage, uint8_t len8bit) {
  i2c_recive(ICM_ADDR_AD1, reg, storage, len8bit);
}

void IMU_write(uint8_t reg, uint8_t value) {
  i2c_send(ICM_ADDR_AD1, reg, & value, 1);
}

void IMU_write_bit(uint8_t reg, uint8_t bit, uint8_t data) {
  uint8_t b;
  b = IMU_read_byte(reg);
  b = (data != 0) ? (b | (1 << bit)) : (b & ~(1 << bit));
  IMU_write(reg, b);
}

void IMU_write_bits(uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data) {
  //      010 value to write
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  // 00011100 mask byte
  // 10101111 original value (sample)
  // 10100011 original & ~mask
  // 10101011 masked | value
  uint8_t b;
  b = IMU_read_byte(reg);
  uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
  data <<= (bitStart - length + 1); // shift data into correct position
  data &= mask; // zero all non-important bits in data
  b &= ~(mask); // zero all important bits in existing byte
  b |= data; // combine data with existing byte
  IMU_write(reg, b);
}

void IMU_set_bank(uint8_t bank) { //Might not work, may be a too simple approach
  IMU_write_bits(REG_BANK_SEL, REG_BANK_SEL_BIT, REG_BANK_SEL_LENGTH, bank);
}

void IMU_init(void) {

  i2c_init();

  //Set bank to 0
  IMU_set_bank(0x00);
  //Set clock source, set to 1 Auto select clock source
  IMU_write_bits(AGB0_REG_PWR_MGMT_1, PWR1_CLKSEL_BIT, PWR1_CLKSEL_LENGTH, 1);
  //Enable raw data ready interupt
  IMU_write_bit(AGB0_REG_INT_ENABLE_1, RAW_DATA_0_RDY_EN, false); //raw_data_0_rdy_en changed from 1 to 0
  //Disable sleep
  IMU_write_bit(AGB0_REG_PWR_MGMT_1, PWR1_SLEEP_BIT, false);
  //Set bank to 2
  IMU_set_bank(0x02);
  //Configure accelerometer +-2g
  IMU_write_bits(AGB2_REG_ACCEL_CONFIG, ACCEL_FS_SEL, ACCEL_FS_SEL_LENGTH, 0x00);
  //Configure gyro scale +- 250 deg/s
  IMU_write_bits(AGB2_REG_GYRO_CONFIG_1, GYRO_1_FS_SEL, GYRO_1_FS_SEL_LENGTH, 0x00);
  //Set gyro sample rate
  IMU_write(AGB2_REG_GYRO_SMPLRT_DIV, 23);
  //Enable gyro low pass filter
  IMU_write_bit(AGB2_REG_GYRO_CONFIG_1, GYRO_1_FCHOICE, true);
  //Set lowpass filter for gyro
  IMU_write_bits(AGB2_REG_GYRO_CONFIG_1, GYRO_1_DLPFCFG, GYRO_1_DLPFCFG_LENGTH, 4);
  //Enable accelerometer low pass filter
  IMU_write_bit(AGB2_REG_ACCEL_CONFIG, ACCEL_FCHOICE, false);
  //Set lowpass filter for accelerometer
  IMU_write_bits(AGB2_REG_ACCEL_CONFIG, ACCEL_DLPFCFG, ACCEL_DLPFCFG_LENGTH, 4);

}

int IMU_new_data(void) {
  IMU_set_bank(0x00);
  uint8_t data = IMU_read_byte(AGB0_REG_INT_STATUS_1); //A set bit 0 indicates new data, denoted R/C in datasheet, assuming cleared on read
  return (data << 7);
}

void IMU_read(void) {
  IMU_set_bank(0x00);
  IMU_read_burst(AGB0_REG_ACCEL_XOUT_H, (uint8_t * ) & IMU_buff, 23);
}

float gyro_to_deg(int16_t gyro_raw) {
  const float gyro_sens = 131.0; // Gyro_sensitivity = 32767bits/250dps
  return gyro_raw / gyro_sens;
}

float accel_to_g(int16_t acc_raw) {
  const float accel_sens = 16383.5; //Accel_sensitivity = 32767bits/2g
  return acc_raw / accel_sens;
}

float g_IMU_float_accelX(void) {
  int16_t temp;
  temp = ((int16_t) IMU_buff[0] << 8) | IMU_buff[1];
  return accel_to_g(-temp); //set to - to get correct direction
}

float g_IMU_float_accelY(void) {
  int16_t temp;
  temp = ((int16_t) IMU_buff[2] << 8) | IMU_buff[3];
  return accel_to_g(temp);
}

float g_IMU_float_accelZ(void) {
  int16_t temp;
  temp = ((int16_t) IMU_buff[4] << 8) | IMU_buff[5];
  return accel_to_g(temp);
}

float g_IMU_float_gyroX(void) {
  int16_t temp;
  temp = ((int16_t) IMU_buff[6] << 8) | IMU_buff[7];
  return gyro_to_deg(temp);
}

float g_IMU_float_gyroY(void) {
  int16_t temp;
  temp = ((int16_t) IMU_buff[8] << 8) | IMU_buff[9];
  return gyro_to_deg(temp);
}

float g_IMU_float_gyroZ(void) {
  int16_t temp;
  temp = ((int16_t) IMU_buff[10] << 8) | IMU_buff[11];
  return gyro_to_deg(-temp);
}

int g_IMU_temp(void) {
  int16_t temp;
  int room_temp_offset = 21;
  float temp_sensitivity = 333.87;
  temp = ((int16_t) IMU_buff[12] << 8) | IMU_buff[13];
  return (int)((((float) temp - room_temp_offset) / temp_sensitivity) + 21); //temperature in C 
}

IMU_reading_t IMU_getAccel(void) {
  IMU_reading_t accel;
  accel.x = g_IMU_float_accelX();
  accel.y = g_IMU_float_accelY();
  accel.z = g_IMU_float_accelZ();
  return accel;
}

IMU_reading_t IMU_getGyro(void) {
  IMU_reading_t gyro;
  gyro.x = g_IMU_float_gyroX();
  gyro.y = g_IMU_float_gyroY();
  gyro.z = g_IMU_float_gyroZ();
  return gyro;
}