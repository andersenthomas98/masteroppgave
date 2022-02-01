/************************************************************************/
// File:            ICM_20948.h                                           //
// Author:                                                              //
// Purpose:         IMU driver                                          //
//                                                                      //
/************************************************************************/
#ifndef ICM_20948_H
#define ICM_20948_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "ICM_20948_REGISTERS.h"
#include "ICM_20948_ENUMERATIONS.h" // This is to give users access to usable value definiitons
//#include "AK09916_ENUMERATIONS.h"


typedef struct {
	float x;
	float y;
	float z;
} IMU_reading_t;

#define ICM_ADDR_AD0 0x68 // Or 0x69 when AD0 is high 
#define ICM_ADDR_AD1 0x69 // (Default)
#define ICM_20948_WHOAMI 0xEA

#define MAG_AK09916_I2C_ADDR 0x0C
#define MAG_AK09916_WHO_AM_I 0x4809
#define MAG_REG_WHO_AM_I 0x00

//User bank 0

#define PWR1_DEVICE_RESET_BIT   7
#define PWR1_SLEEP_BIT          6
#define PWR1_CYCLE_BIT          5
#define PWR1_TEMP_DIS_BIT       3
#define PWR1_CLKSEL_BIT         2
#define PWR1_CLKSEL_LENGTH      3

//Project functions
uint8_t IMU_read_byte(uint8_t reg);
void IMU_read_burst(uint8_t reg, uint8_t* storage, uint8_t len8bit);
void IMU_write(uint8_t reg, uint8_t value);
void IMU_write_bit(uint8_t reg, uint8_t bit, uint8_t data);
void IMU_write_bits(uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);
void IMU_set_bank(uint8_t bank);
void IMU_init(void);
int IMU_new_data(void);

void IMU_read(void);
float gyro_to_deg(int16_t gyro_raw);
float accel_to_g(int16_t acc_raw);
float g_IMU_float_accelX(void);
float g_IMU_float_accelY(void);
float g_IMU_float_accelZ(void);
float g_IMU_float_gyroX(void);
float g_IMU_float_gyroY(void);
float g_IMU_float_gyroZ(void);
int g_IMU_temp(void);
IMU_reading_t IMU_getGyro(void);
IMU_reading_t IMU_getAccel(void);




#endif /* ICM_20948_H */