/************************************************************************/
// File:            i2c.h												//
// Author:																//
// Purpose:                                                             //
//                                                                      //
/************************************************************************/

#ifndef I2C_H
#define I2C_H

#include "nrf_twi_mngr.h"

void i2c_init();

void i2c_recive(uint8_t device_address, uint8_t addr, uint8_t* data, uint8_t len);
void i2c_send(uint8_t device_address, uint8_t addr, uint8_t* data, uint8_t len);
void i2c_sendNOADDR(uint8_t device_address,uint8_t* data,uint8_t len);
void i2c_reciveNOADDR(uint8_t device_address, uint8_t* data, uint8_t len);
const nrf_twi_mngr_t* getTWIManagerAddress();
const nrf_drv_twi_config_t* getBusConfig();

//New functions
//void i2c_init();
//void i2c_sendNOADDR(uint8_t device_addr, uint8_t * pdata, size_t size);









#endif /* I2C.h */