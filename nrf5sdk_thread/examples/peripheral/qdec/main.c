/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
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
* @defgroup nrf_qdec_example main.c
* @{
* @ingroup nrf_qdec_example
* @brief QDEC example application main file.
*
* This is an example quadrature decoder application.
* The example requires that the QDEC A,B inputs are connected with the QENC A,B outputs and
* the QDEC LED output is connected with the QDEC LED input.
*
* The example uses the software quadrature encoder simulator QENC.
* The quadrature encoder simulator uses one channel of the GPIOTE module.
* The state of the encoder changes on the inactive edge of the sampling clock generated by the LED output.
*
* In an infinite loop, QENC produces a variable number of positive and negative pulses
* synchronously with bursts of clock impulses generated by QDEC at the LED output.
* The pulses are counted by QDEC operating in a REPORT mode.
* The pulses counted by QDEC are compared with the pulses generated by QENC.
* The test stops if there is a difference between the number of pulses counted and generated.
*
*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "nrf.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "nrf_drv_qdec.h"
#include "nrf_error.h"
#include "app_error.h"
#include "qenc_sim.h"
#include "nordic_common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static volatile bool m_report_ready_flag = false;
static volatile bool m_first_report_flag = true;
static volatile uint32_t m_accdblread;
static volatile int32_t m_accread;


#if (QDEC_CONFIG_LEDPRE >= 128)
    #warning "This example assumes that the QDEC LED changes state. Make sure that 'Sample Period' in QDEC config is less than 'LED pre-time'."
#endif

static void qdec_event_handler(nrf_drv_qdec_event_t event)
{
    if (event.type == NRF_QDEC_EVENT_REPORTRDY)
    {
        m_accdblread        = event.data.report.accdbl;
        m_accread           = event.data.report.acc;
        m_report_ready_flag = true;
        nrf_drv_qdec_disable();
    }
}

void check_report(int32_t expected)
{
    // only first run is specific...
    if ((expected > 0) && m_first_report_flag)
    {
       expected --;
    }
    else if ((expected < 0) && m_first_report_flag)
    {
       expected ++;
    }

    // Error checking and printing
    if ( m_accdblread != 0 )
    {
        NRF_LOG_ERROR("m_accdblread was expected to have value 0 but is %u", (unsigned int)m_accdblread);
        APP_ERROR_HANDLER(0);
    }
    if ( m_accread != expected )
    {
        NRF_LOG_ERROR("m_accread should be %d but is %d", (int)expected, (int)m_accread);
        APP_ERROR_HANDLER(0);
    }
    m_first_report_flag = false;  // clear silently after first run

}

int main(void)
{
    uint32_t err_code;
    uint32_t number_of_pulses;
    uint32_t min_number_of_pulses = 2;
    uint32_t max_number_of_pulses = 0;
    int32_t  pulses;
    int32_t  sign = 1;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // Initialize hardware
    err_code = nrf_drv_qdec_init(NULL, qdec_event_handler);
    APP_ERROR_CHECK(err_code);

    max_number_of_pulses = nrf_qdec_reportper_to_value(QDEC_CONFIG_REPORTPER);

    // Initialize quadrature encoder simulator
    qenc_init((nrf_qdec_ledpol_t)nrf_qdec_ledpol_get());

    NRF_LOG_INFO("QDEC testing started.");

    while (true)
    {
      // change a number and sign of pulses produced by simulator in a loop
      for (number_of_pulses=min_number_of_pulses; number_of_pulses<= max_number_of_pulses; number_of_pulses++ )
      {
        pulses = sign * number_of_pulses;       // pulses have sign
        qenc_pulse_count_set(pulses);           // set pulses to be produced by encoder
        nrf_drv_qdec_enable();                  // start burst sampling clock, clock will be stopped by REPORTRDY event
        while (! m_report_ready_flag)           // wait for a report
        {
          __WFE();
        }
        NRF_LOG_RAW_INFO("*");
        m_report_ready_flag = false;
        check_report(pulses);           // check if pulse count is as expected, assert otherwise
      }
      min_number_of_pulses  = 1;        // only first run is specific, for 1 there would be no call back...
      sign                  = -sign;    // change sign of pulses in a loop
      NRF_LOG_FLUSH();
    }
}

/** @} */
