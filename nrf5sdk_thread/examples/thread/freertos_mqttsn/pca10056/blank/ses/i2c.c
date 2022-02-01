/************************************************************************/
// File:            i2c.h												//
// Author:																//
// Purpose:                                                             //
//                                                                      //
/************************************************************************/

#include "i2c.h"
#include "freeRTOS.h"
#include "semphr.h"
#include "nrf_twi_mngr.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrfx_gpiote.h"
#define TWI_INSTANCE_ID             1
#define MAX_PENDING_TRANSACTIONS    5
#define BUFFER_LENGTH              6


#define PIN_I2C_SDA 26
#define PIN_I2C_SCL 27


SemaphoreHandle_t i2cSemaphore;


static const nrf_twi_mngr_t m_nrf_twi_mngr;

NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);

static nrf_drv_twi_config_t const bus_config = {
   .scl                = 27,
   .sda                = 26,
   .frequency          = NRF_DRV_TWI_FREQ_400K, //Should this be kept at 100 or changed back to 400? 
   .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
   .clear_bus_init     = false
};

/*
void i2c_callback(long unsigned int,  void *)
{
    NRF_LOG_WARNING("I2c callback\n");
}*/

void i2c_init(){
      i2cSemaphore = xSemaphoreCreateMutex();
        if(nrf_twi_mngr_init(&m_nrf_twi_mngr,&bus_config) != NRF_SUCCESS){
            NRF_LOG_WARNING("I2c INIT FAILED!\n");
        }
}

void i2c_send(uint8_t device_address,uint8_t addr,uint8_t* data,uint8_t len){
     static uint8_t buffer[BUFFER_LENGTH]; // max length of send is set by this now we can send BUFFERLENGTH-1  uint8_t's, if messages longer than this needs to be sendt this must be incresed
      buffer[0]=addr; // need a seperate buffer to send address first :( 
      for(int i=0; i<len;i++){
          buffer[i+1]=data[i];
      }
      nrf_twi_mngr_transfer_t const SendTransfer[] ={
      NRF_TWI_MNGR_WRITE(device_address,&buffer,len+1,NRF_TWI_MNGR_NO_STOP),
      };
      xSemaphoreTake(i2cSemaphore,200);
      if(nrf_twi_mngr_perform(&m_nrf_twi_mngr, &bus_config, SendTransfer,1,NULL) != NRF_SUCCESS){
          NRF_LOG_WARNING("I2C send failed for device address (hex) %x \n",device_address );
      }
      xSemaphoreGive(i2cSemaphore);
}

void i2c_recive(uint8_t device_address, uint8_t addr, uint8_t* data, uint8_t len){
      nrf_twi_mngr_transfer_t const ReciveTransfer[] ={
      NRF_TWI_MNGR_WRITE(device_address,&addr,1,NRF_TWI_MNGR_NO_STOP),
      NRF_TWI_MNGR_READ(device_address,data,len,0)
      };
      xSemaphoreTake(i2cSemaphore,200);
      if(nrf_twi_mngr_perform(&m_nrf_twi_mngr, &bus_config, ReciveTransfer, 2, NULL) != NRF_SUCCESS){
          NRF_LOG_WARNING("I2C recive failed for device address (hex) %x \n",device_address );
      }
      xSemaphoreGive(i2cSemaphore);
}

void i2c_sendNOADDR(uint8_t device_address, uint8_t* data, uint8_t len){
      nrf_twi_mngr_transfer_t const SendTransfer[] ={
      NRF_TWI_MNGR_WRITE(device_address,data,len,NRF_TWI_MNGR_NO_STOP),
      };
     /* nrf_twi_mngr_transaction_t const Transaction[] = {
        i2c_callback, 
        data, 
        SendTransfer, 
        1, 
        &bus_config
      }; */
      xSemaphoreTake(i2cSemaphore,200);
      if(nrf_twi_mngr_perform(&m_nrf_twi_mngr, &bus_config, SendTransfer,1,NULL) != NRF_SUCCESS){
          NRF_LOG_WARNING("I2C_sendNOADDR failed!\n"); //nrf_twi_mngr_perform(&m_nrf_twi_mngr, &bus_config, SendTransfer,1,NULL)     nrf_twi_mngr_schedule(&m_nrf_twi_mngr, Transaction)
      }
      xSemaphoreGive(i2cSemaphore);
}

void i2c_reciveNOADDR(uint8_t device_address, uint8_t* data, uint8_t len){
      nrf_twi_mngr_transfer_t const ReciveTransfer[] ={
      NRF_TWI_MNGR_READ(device_address,data,len,0)
      };
      xSemaphoreTake(i2cSemaphore,200);
      if(nrf_twi_mngr_perform(&m_nrf_twi_mngr, &bus_config, ReciveTransfer, 1, NULL) != NRF_SUCCESS){
          NRF_LOG_WARNING("I2C_reciveNOADDR failed!\n");
      }
      xSemaphoreGive(i2cSemaphore);
}


const nrf_twi_mngr_t* getTWIManagerAddress(){
        return &m_nrf_twi_mngr;
}

const nrf_drv_twi_config_t* getBusConfig(){
        return &bus_config;
} 
  
