/**
 * Copyright (c) 2018 - 2020, Nordic Semiconductor ASA
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
 * @defgroup zigbee_examples_light_coordinator main.c
 * @{
 * @ingroup zigbee_examples
 * @brief Simple Zigbee network coordinator implementation.
 */

#include "zboss_api.h"
#include "zb_mem_config_max.h"
#include "zb_error_handler.h"
#include "zigbee_helpers.h"

#include "app_timer.h"
#include "boards.h"
#include "bsp.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ERASE_PERSISTENT_CONFIG           ZB_FALSE                              /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. NOTE: If this option is set to ZB_TRUE then do full device erase for all network devices before running other samples. */
#define MAX_CHILDREN                      10                                    /**< The maximum amount of connected devices. Setting this value to 0 disables association to this device.  */
#define IEEE_CHANNEL_MASK                 (1l << ZIGBEE_CHANNEL)                /**< Scan only one, predefined channel to find the coordinator. */
#ifdef  BOARD_PCA10059                                                          /**< If it is Dongle */
#define ZIGBEE_NETWORK_STATE_LED          BSP_BOARD_LED_0                       /**< LED indicating that network is opened for new nodes. */
#else
#define ZIGBEE_NETWORK_STATE_LED          BSP_BOARD_LED_2                       /**< LED indicating that network is opened for new nodes. */
#endif
#define ZIGBEE_NETWORK_REOPEN_BUTTON      BSP_BOARD_BUTTON_0                    /**< Button which reopens the Zigbee Network. */
#define ZIGBEE_MANUAL_STEERING            ZB_FALSE                              /**< If set to 1 then device will not open the network after forming or reboot. */
#define ZIGBEE_PERMIT_LEGACY_DEVICES      ZB_FALSE                              /**< If set to 1 then legacy devices (pre-Zigbee 3.0) can join the network. */

#ifndef ZB_COORDINATOR_ROLE
#error Define ZB_COORDINATOR_ROLE to compile coordinator source code.
#endif


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Callback used in order to visualise network steering period.
 *
 * @param[in]   param   Not used. Required by callback type definition.
 */
static zb_void_t steering_finished(zb_uint8_t param)
{
    UNUSED_PARAMETER(param);
    NRF_LOG_INFO("Network steering finished");
    bsp_board_led_off(ZIGBEE_NETWORK_STATE_LED);
}

/**@brief Callback for button events.
 *
 * @param[in]   evt      Incoming event from the BSP subsystem.
 */
static void buttons_handler(bsp_event_t evt)
{
    zb_bool_t comm_status;

    switch(evt)
    {
        case BSP_EVENT_KEY_0:
            UNUSED_RETURN_VALUE(ZB_SCHEDULE_APP_ALARM_CANCEL(steering_finished, ZB_ALARM_ANY_PARAM));

            comm_status = bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
            if (comm_status)
            {
                NRF_LOG_INFO("Top level comissioning restated");
            }
            else
            {
                NRF_LOG_INFO("Top level comissioning hasn't finished yet!");
            }
            break;

        default:
            NRF_LOG_INFO("Unhandled BSP Event received: %d", evt);
            break;
    }
}


/**@brief Function for initializing LEDs and Buttons.
 */
static void leds_buttons_init(void)
{
    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, buttons_handler);
    APP_ERROR_CHECK(err_code);
    /* By default the bsp_init attaches BSP_KEY_EVENTS_{0-4} to the PUSH events of the corresponding buttons. */

    bsp_board_leds_off();
}

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
    /* Read signal description out of memory buffer. */
    zb_zdo_app_signal_hdr_t * p_sg_p      = NULL;
    zb_zdo_app_signal_type_t  sig         = zb_get_app_signal(bufid, &p_sg_p);
    zb_ret_t                  status      = ZB_GET_APP_SIGNAL_STATUS(bufid);
    zb_ret_t                  zb_err_code;
    zb_bool_t                 comm_status;
    zb_time_t                 timeout_bi;

    switch(sig)
    {
        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            // BDB initialization completed after device reboot, use NVRAM contents during initialization. Device joined/rejoined and started.
            if (status == RET_OK)
            {
                if (ZIGBEE_MANUAL_STEERING == ZB_FALSE)
                {
                    NRF_LOG_INFO("Start network steering");
                    comm_status = bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
                    ZB_COMM_STATUS_CHECK(comm_status);
                }
                else
                {
                    NRF_LOG_INFO("Coordinator restarted successfully");
                }
            }
            else
            {
                NRF_LOG_ERROR("Failed to initialize Zigbee stack using NVRAM data (status: %d)", status);
            }
            break;

        case ZB_BDB_SIGNAL_STEERING:
            if (status == RET_OK)
            {
                if (ZIGBEE_PERMIT_LEGACY_DEVICES == ZB_TRUE)
                {
                    NRF_LOG_INFO("Allow pre-Zigbee 3.0 devices to join the network");
                    zb_bdb_set_legacy_device_support(1);
                }

                /* Schedule an alarm to notify about the end of steering period */
                NRF_LOG_INFO("Network steering started");
                zb_err_code = ZB_SCHEDULE_APP_ALARM(steering_finished, 0, ZB_TIME_ONE_SECOND * ZB_ZGP_DEFAULT_COMMISSIONING_WINDOW);
                ZB_ERROR_CHECK(zb_err_code);
            }
#ifndef ZB_ED_ROLE
            zb_enable_auto_pan_id_conflict_resolution(ZB_FALSE);
#endif
            break;

        case ZB_ZDO_SIGNAL_DEVICE_ANNCE:
            {
                zb_zdo_signal_device_annce_params_t * dev_annce_params = ZB_ZDO_SIGNAL_GET_PARAMS(p_sg_p, zb_zdo_signal_device_annce_params_t);
                NRF_LOG_INFO("New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);

                zb_err_code = ZB_SCHEDULE_APP_ALARM_CANCEL(steering_finished, ZB_ALARM_ANY_PARAM);
                if (zb_err_code == RET_OK)
                {
                    NRF_LOG_INFO("Joining period extended.");
                    zb_err_code = ZB_SCHEDULE_APP_ALARM(steering_finished, 0, ZB_TIME_ONE_SECOND * ZB_ZGP_DEFAULT_COMMISSIONING_WINDOW);
                    ZB_ERROR_CHECK(zb_err_code);
                }
            }
            break;

        default:
            /* Call default signal handler. */
            ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
            break;
    }

    /* Update network status LED */
    if (ZB_JOINED() && (ZB_SCHEDULE_GET_ALARM_TIME(steering_finished, ZB_ALARM_ANY_PARAM, &timeout_bi) == RET_OK))
    {
        bsp_board_led_on(ZIGBEE_NETWORK_STATE_LED);
    }
    else
    {
        bsp_board_led_off(ZIGBEE_NETWORK_STATE_LED);
    }

    /* All callbacks should either reuse or free passed buffers. If bufid == 0, the buffer is invalid (not passed) */
    if (bufid)
    {
        zb_buf_free(bufid);
    }
}

/**@brief Function for application main entry. */
int main(void)
{
    zb_ret_t       zb_err_code;
    zb_ieee_addr_t ieee_addr;

    // Initialize.
    timers_init();
    log_init();
    leds_buttons_init();

    /* Set Zigbee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize Zigbee stack. */
    ZB_INIT("zc");

    /* Set device address to the value read from FICR registers. */
    zb_osif_get_ieee_eui64(ieee_addr);
    zb_set_long_address(ieee_addr);

    /* Set channels on which the coordinator will try to create a new network. */
    zb_set_network_coordinator_role(IEEE_CHANNEL_MASK);
    zb_set_max_children(MAX_CHILDREN);

    /* Keep or erase NVRAM to save the network parameters after device reboot or power-off. */
    zigbee_erase_persistent_storage(ERASE_PERSISTENT_CONFIG);

    /** Start Zigbee Stack. */
    zb_err_code = zboss_start_no_autostart();
    ZB_ERROR_CHECK(zb_err_code);

    while(1)
    {
        zboss_main_loop_iteration();
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    }
}


/**
 * @}
 */
