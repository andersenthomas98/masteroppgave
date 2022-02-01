#ifndef ICM_20948_REGISTERS_H
#define ICM_20948_REGISTERS_H

#include <stdint.h>

// Generalized
#define REG_BANK_SEL 0x7F // All defines must be changed to this format
#define REG_BANK_SEL_BIT 5
#define REG_BANK_SEL_LENGTH 2

// Gyroscope and Accelerometer		
// Gyroscope and Accelerometer
// Gyroscope and Accelerometer		
// User Bank 0
#define AGB0_REG_WHO_AM_I 0x00
// Break
#define AGB0_REG_USER_CTRL 0x03
// Break
#define AGB0_REG_LP_CONFIG 0x05
#define AGB0_REG_PWR_MGMT_1 0x06
#define AGB0_REG_PWR_MGMT_2 0x07
// Break
#define AGB0_REG_INT_PIN_CONFIG 0x0F
#define AGB0_REG_INT_ENABLE 0x10
#define AGB0_REG_INT_ENABLE_1 0x11
#define AGB0_REG_INT_ENABLE_2 0x12
#define AGB0_REG_INT_ENABLE_3 0x13
// Break
#define AGB0_REG_I2C_MST_STATUS 0x17
// Break
#define AGB0_REG_INT_STATUS 0x19
#define AGB0_REG_INT_STATUS_1 0x1A
//#define AGB0_REG_INT_STATUS_2,
//#define AGB0_REG_INT_STATUS_3,
// Break
#define AGB0_REG_DELAY_TIMEH 0x28
//#define AGB0_REG_DELAY_TIMEL
// Break
#define AGB0_REG_ACCEL_XOUT_H 0x2D
/*
#define AGB0_REG_ACCEL_XOUT_L,
#define AGB0_REG_ACCEL_YOUT_H,
#define AGB0_REG_ACCEL_YOUT_L,
#define AGB0_REG_ACCEL_ZOUT_H,
#define AGB0_REG_ACCEL_ZOUT_L,
#define AGB0_REG_GYRO_XOUT_H,
#define AGB0_REG_GYRO_XOUT_L,
#define AGB0_REG_GYRO_YOUT_H,
#define AGB0_REG_GYRO_YOUT_L,
#define AGB0_REG_GYRO_ZOUT_H,
#define AGB0_REG_GYRO_ZOUT_L,
#define AGB0_REG_TEMP_OUT_H, 
#define AGB0_REG_TEMP_OUT_L,
#define AGB0_REG_EXT_SLV_SENS_DATA_00,
#define AGB0_REG_EXT_SLV_SENS_DATA_01,
#define AGB0_REG_EXT_SLV_SENS_DATA_02,
#define AGB0_REG_EXT_SLV_SENS_DATA_03,
#define AGB0_REG_EXT_SLV_SENS_DATA_04,
#define AGB0_REG_EXT_SLV_SENS_DATA_05,
#define AGB0_REG_EXT_SLV_SENS_DATA_06,
#define AGB0_REG_EXT_SLV_SENS_DATA_07,
#define AGB0_REG_EXT_SLV_SENS_DATA_08,
#define AGB0_REG_EXT_SLV_SENS_DATA_09,
#define AGB0_REG_EXT_SLV_SENS_DATA_10,
#define AGB0_REG_EXT_SLV_SENS_DATA_11,
#define AGB0_REG_EXT_SLV_SENS_DATA_12,
#define AGB0_REG_EXT_SLV_SENS_DATA_13,
#define AGB0_REG_EXT_SLV_SENS_DATA_14,
#define AGB0_REG_EXT_SLV_SENS_DATA_15,
#define AGB0_REG_EXT_SLV_SENS_DATA_16,
#define AGB0_REG_EXT_SLV_SENS_DATA_17,
#define AGB0_REG_EXT_SLV_SENS_DATA_18,
#define AGB0_REG_EXT_SLV_SENS_DATA_19,
#define AGB0_REG_EXT_SLV_SENS_DATA_20,
#define AGB0_REG_EXT_SLV_SENS_DATA_21,
#define AGB0_REG_EXT_SLV_SENS_DATA_22,
#define AGB0_REG_EXT_SLV_SENS_DATA_23,
*/
// Break
#define AGB0_REG_FIFO_EN_1 0x66
//#define AGB0_REG_FIFO_EN_2,
//#define AGB0_REG_FIFO_MODE,
// Break
#define AGB0_REG_FIFO_COUNT_H 0x70
//#define AGB0_REG_FIFO_COUNT_L,
//#define AGB0_REG_FIFO_R_W,
// Break
#define AGB0_REG_DATA_RDY_STATUS 0x74
// Break
#define AGB0_REG_FIFO_CFG 0x76
// Break
#define AGB0_REG_MEM_START_ADDR 0x7C // Hmm, Invensense thought they were sneaky not listing these locations on the datasheet...
#define AGB0_REG_MEM_R_W 0x7D // These three locations seem to be able to access some memory within the device
#define AGB0_REG_MEM_BANK_SEL 0x7E // And that location is also where the DMP image gets loaded
#define AGB0_REG_REG_BANK_SEL 0x7F

// Bank 1
#define AGB1_REG_SELF_TEST_X_GYRO 0x02
//#define AGB1_REG_SELF_TEST_Y_GYRO,
//#define AGB1_REG_SELF_TEST_Z_GYRO,
// Break
#define AGB1_REG_SELF_TEST_X_ACCEL 0x0E
//#define AGB1_REG_SELF_TEST_Y_ACCEL,
//#define AGB1_REG_SELF_TEST_Z_ACCEL,
// Break
#define AGB1_REG_XA_OFFS_H 0x14
//#define AGB1_REG_XA_OFFS_L
// Break
#define AGB1_REG_YA_OFFS_H 0x17
//#define AGB1_REG_YA_OFFS_L,
// Break
#define AGB1_REG_ZA_OFFS_H 0x1A
//#define AGB1_REG_ZA_OFFS_L,
// Break
#define AGB1_REG_TIMEBASE_CORRECTION_PLL 0x28
// Break
#define AGB1_REG_REG_BANK_SEL 0x7F

// Bank 2
#define AGB2_REG_GYRO_SMPLRT_DIV 0x00
#define AGB2_REG_GYRO_CONFIG_1 0x01
#define AGB2_REG_GYRO_CONFIG_2 0x02
#define AGB2_REG_XG_OFFS_USRH 0x03
#define AGB2_REG_XG_OFFS_USRL 0x04
#define AGB2_REG_YG_OFFS_USRH 0x05
#define AGB2_REG_YG_OFFS_USRL 0x06
//#define AGB2_REG_ZG_OFFS_USRH,
//#define AGB2_REG_ZG_OFFS_USRL,
//#define AGB2_REG_ODR_ALIGN_EN,
// Break
#define AGB2_REG_ACCEL_SMPLRT_DIV_1 0x10
#define AGB2_REG_ACCEL_SMPLRT_DIV_2 0x11
#define AGB2_REG_ACCEL_INTEL_CTRL 0x12
#define AGB2_REG_ACCEL_WOM_THR 0x13
#define AGB2_REG_ACCEL_CONFIG 0x14
#define AGB2_REG_ACCEL_CONFIG_2 0x15
// Break
#define AGB2_REG_FSYNC_CONFIG 0x52
//#define AGB2_REG_TEMP_CONFIG
//#define AGB2_REG_MOD_CTRL_USR
// Break
//#define AGB2_REG_REG_BANK_SEL = 0x7F

// Bank 3
#define AGB3_REG_I2C_MST_ODR_CONFIG 0x00
/*
#define AGB3_REG_I2C_MST_CTRL,
#define AGB3_REG_I2C_MST_DELAY_CTRL,
#define AGB3_REG_I2C_SLV0_ADDR,
#define AGB3_REG_I2C_SLV0_REG,
#define AGB3_REG_I2C_SLV0_CTRL,
#define AGB3_REG_I2C_SLV0_DO,
#define AGB3_REG_I2C_SLV1_ADDR,
#define AGB3_REG_I2C_SLV1_REG,
#define AGB3_REG_I2C_SLV1_CTRL,
#define AGB3_REG_I2C_SLV1_DO,
#define AGB3_REG_I2C_SLV2_ADDR,
#define AGB3_REG_I2C_SLV2_REG,
#define AGB3_REG_I2C_SLV2_CTRL,
#define AGB3_REG_I2C_SLV2_DO,
#define AGB3_REG_I2C_SLV3_ADDR,
#define AGB3_REG_I2C_SLV3_REG,
#define AGB3_REG_I2C_SLV3_CTRL,
#define AGB3_REG_I2C_SLV3_DO,
#define AGB3_REG_I2C_SLV4_ADDR,
#define AGB3_REG_I2C_SLV4_REG,
#define AGB3_REG_I2C_SLV4_CTRL,
#define AGB3_REG_I2C_SLV4_DO,
#define AGB3_REG_I2C_SLV4_DI,
*/
// Break
#define AGB3_REG_REG_BANK_SEL 0x7F

// Magnetometer
#define M_REG_WIA2 0x01
// Break
#define M_REG_ST1 0x10
//#define M_REG_HXL,
//#define M_REG_HXH,
//#define M_REG_HYL,
//#define M_REG_HYH,
//#define M_REG_HZL,
//#define M_REG_HZH,
//#define M_REG_ST2,
// Break
//#define M_REG_CNTL2 = 0x31
//#define M_REG_CNTL3,
//#define M_REG_TS1,
//#define M_REG_TS2,
// These enums are not needed for the user, so they stay in this scope (simplifies naming among other things)

//INT_ENABLE_1	
#define RAW_DATA_0_RDY_EN 0
#define INT_ENABLE_1_reserved_0 7

//GYRO_CONFIG_1
#define GYRO_1_FCHOICE_LENGHT 1
#define GYRO_1_FCHOICE 0 //Not sure if it will find bit 0?
#define GYRO_1_FS_SEL 2
#define GYRO_1_DLPFCFG 5
#define GYRO_1_reserved_0 2

#define GYRO_1_FS_SEL_LENGTH 2
#define GYRO_1_DLPFCFG_LENGTH 3

// ACCEL_CONFIG
#define ACCEL_FCHOICE 0
#define ACCEL_FS_SEL 2
#define ACCEL_DLPFCFG 5
#define ACCEL_reserved_0 2
#define ACCEL_FS_SEL_LENGTH 2
#define ACCEL_DLPFCFG_LENGTH 3

#endif /* ICM_20948_REGISTERS_H */