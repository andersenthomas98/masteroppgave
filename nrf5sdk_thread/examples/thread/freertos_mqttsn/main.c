/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup freertos_coap_server_example_main main.c
 * @{
 * @ingroup freertos_coap_server_example
 *
 * @brief Thread CoAP server example with FreeRTOS Application main file.
 *
 * This file contains the source code for a sample application using Thread CoAP server and FreeRTOS.
 *
 */


#include "FreeRTOS.h"
#include "nrf_drv_clock.h"
#include "task.h"
#include "app_scheduler.h"
#include "app_timer.h"
#define NRF_LOG_MODULE_NAME APP
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();
#include "bsp_thread.h"
#include <openthread/instance.h>

#include "thread_mqttsn.h"
#include "example_task.h"

//#define SCHED_QUEUE_SIZE       32                                          /**< Maximum number of events in the scheduler queue. */
//#define SCHED_EVENT_DATA_SIZE  APP_TIMER_SCHED_EVENT_DATA_SIZE              /**< Maximum app_scheduler event size. */

#define THREAD_STACK_TASK_STACK_SIZE     (( 1024 * 8 ) / sizeof(StackType_t))   /**< FreeRTOS task stack size is determined in multiples of StackType_t. */
#define LOG_TASK_STACK_SIZE              ( 1024 / sizeof(StackType_t))          /**< FreeRTOS task stack size is determined in multiples of StackType_t. */
#define MQTTSN_TASK_STACK_SIZE           ((1024 * 14) / sizeof(StackType_t))
#define EXAMPLE_TASK_STACK_SIZE           (1024 / sizeof(StackType_t))
#define THREAD_STACK_TASK_PRIORITY       2
#define MQTTSN_TASK_PRIORITY             1
#define EXAMPLE_TASK_PRIORITY            1
#define LOG_TASK_PRIORITY                1
#define LED1_TASK_PRIORITY               1
#define LED2_TASK_PRIORITY               1
#define LED1_BLINK_INTERVAL              427
#define LED2_BLINK_INTERVAL              472
#define LOG_TASK_INTERVAL                10


TaskHandle_t thread_stack_task_handle = NULL;   /**< Thread stack task handle */
TaskHandle_t mqttsn_task_handle = NULL;         /**< MQTT-SN task handle */
TaskHandle_t example_task_handle = NULL;
TaskHandle_t led1_task_handle = NULL;           /**< LED1 task handle*/
TaskHandle_t led2_task_handle = NULL;           /**< LED2 task handle*/
#if NRF_LOG_ENABLED
  TaskHandle_t logger_task_handle = NULL;         /**< Definition of Logger thread. */
#endif

static inline void light_on(void)
{
    vTaskResume(led1_task_handle);
    vTaskResume(led2_task_handle);
}


static inline void light_off(void)
{
    vTaskSuspend(led1_task_handle);
    LEDS_OFF(BSP_LED_2_MASK);

    vTaskSuspend(led2_task_handle);
    LEDS_OFF(BSP_LED_3_MASK);
}


/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

 /**@brief Function for initializing the Application Timer Module
 */
static void timer_init(void)
{
    uint32_t error_code = app_timer_init();
    APP_ERROR_CHECK(error_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing scheduler module.
 */
/*static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}*/


#if NRF_LOG_ENABLED
static void logger_thread(void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    while (true)
    {
        if (!(NRF_LOG_PROCESS()))
        {
            /* No more logs, let's sleep and wait for any */
            vTaskDelay(LOG_TASK_INTERVAL);
        }
    }
}
#endif //NRF_LOG_ENABLED

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/

int main(void)
{
    log_init();
    //scheduler_init();
    clock_init();
    timer_init();

    // Start thread stack execution.
    if (pdPASS != xTaskCreate(thread_stack_task, "THR", THREAD_STACK_TASK_STACK_SIZE, NULL, THREAD_STACK_TASK_PRIORITY, &thread_stack_task_handle))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    // MQTT connection execution.
    if (pdPASS != xTaskCreate(mqttsn_task, "MQTT", MQTTSN_TASK_STACK_SIZE, NULL, MQTTSN_TASK_PRIORITY, &mqttsn_task_handle))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    if (pdPASS != xTaskCreate(example_task, "EX", EXAMPLE_TASK_STACK_SIZE, NULL, EXAMPLE_TASK_PRIORITY, &example_task_handle))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", LOG_TASK_STACK_SIZE, NULL, LOG_TASK_PRIORITY, &logger_task_handle))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    while (true)
    {
        /* FreeRTOS should not be here... FreeRTOS goes back to the start of stack
         * in vTaskStartScheduler function. */
    }
}


/**
 *@}
 **/
