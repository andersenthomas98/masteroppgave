#include "thread_mqttsn.h"
#include "mqttsn_client.h"
#include "thread_utils.h"
#include "bsp_thread.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "nrf_log.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "mqttsn_platform.h"
#include <openthread/thread.h>
#include "message_buffer.h"
#include <inttypes.h>

#define SCHED_QUEUE_SIZE       32                                           /**< Maximum number of events in the scheduler queue. */
#define SCHED_EVENT_DATA_SIZE  APP_TIMER_SCHED_EVENT_DATA_SIZE              /**< Maximum app_scheduler event size. */

#define LED_ON_REQUEST                49                                    /**< LED ON command. */
#define LED_OFF_REQUEST               48                                    /**< LED OFF command. */
#define SEARCH_GATEWAY_TIMEOUT        5                                     /**< MQTT-SN Gateway discovery procedure timeout in [s]. */
#define MQTTSN_TASK_DELAY_SEC         1                                     /**< MQTTSN task delay */ 

extern TaskHandle_t thread_stack_task_handle, mqttsn_task_handle;

SemaphoreHandle_t registerTopicSemaphore;

static mqttsn_client_t      m_client;                                       /**< An MQTT-SN client instance. */
static mqttsn_remote_t      m_gateway_addr;                                 /**< A gateway address. */
static uint8_t              m_gateway_id;                                   /**< A gateway ID. */
static mqttsn_connect_opt_t m_connect_opt;                                  /**< Connect options for the MQTT-SN client. */
static bool                 m_subscribed       = 0;                         /**< Current subscription state. */
static uint16_t             m_msg_id           = 0;                         /**< Message ID thrown with MQTTSN_EVENT_TIMEOUT. */
static char                 m_client_id[]      = "nRF52840_subscriber";     /**< The MQTT-SN Client's ID. */
static char                 m_topic_name[]     = "nRF52840_resources/led3"; /**< Name of the topic corresponding to subscriber's BSP_LED_2. */
static mqttsn_topic_t       m_topic            =                            /**< Topic corresponding to subscriber's BSP_LED_2. */
{
    .p_topic_name = (unsigned char *)m_topic_name,
    .topic_id     = 0,
};
static uint8_t found_active_gateway = 0;

QueueHandle_t mqttsn_outgoing_message_queue, mqttsn_register_topic_queue;

/**@brief Initializes MQTT-SN client's connection options. */
static void connect_opt_init(void)
{
    m_connect_opt.alive_duration = MQTTSN_DEFAULT_ALIVE_DURATION,
    m_connect_opt.clean_session  = MQTTSN_DEFAULT_CLEAN_SESSION_FLAG,
    m_connect_opt.will_flag      = MQTTSN_DEFAULT_WILL_FLAG,
    m_connect_opt.client_id_len  = strlen(m_client_id),

    memcpy(m_connect_opt.p_client_id,  (unsigned char *)m_client_id,  m_connect_opt.client_id_len);
}

/***************************************************************************************************
 * @section MQTT-SN
 **************************************************************************************************/

/**@brief Processes GWINFO message from a gateway.
 *
 * @details This function updates MQTT-SN Gateway information.
 *
 * @param[in]    p_event  Pointer to MQTT-SN event.
 */
static void gateway_info_callback(mqttsn_event_t * p_event)
{
    m_gateway_addr  = *(p_event->event_data.connected.p_gateway_addr);
    m_gateway_id    = p_event->event_data.connected.gateway_id;
    found_active_gateway = 1;
}


/**@brief Processes CONNACK message from a gateway.
 *
 * @details This function launches the topic registration procedure if necessary.
 */
static void connected_callback(void)
{
    //for (uint8_t i=0; i<NUM_TOPICS; i++) {
      //mqttsn_topic_t topic = topics_to_register[i];
      uint32_t err_code = mqttsn_client_topic_register(&m_client,
                                                       m_topic.p_topic_name,
                                                       strlen(m_topic.p_topic_name),
                                                       &m_msg_id);
      if (err_code != NRF_SUCCESS)
      {
          NRF_LOG_ERROR("REGISTER message could not be sent. Error code: 0x%x\r\n", err_code);
      }
    //}

}


/**@brief Processes DISCONNECT message from a gateway. */
static void disconnected_callback(void)
{
    NRF_LOG_INFO("MQTT-SN event: Disconnected");
}


/**@brief Processes REGACK message from a gateway.
 *
 * @param[in] p_event Pointer to MQTT-SN event.
 */
static void regack_callback(mqttsn_event_t * p_event)
{
    m_topic.topic_id = p_event->event_data.registered.packet.topic.topic_id;
    NRF_LOG_INFO("MQTT-SN event: Topic has been registered with ID: %d.\r\n",
                 p_event->event_data.registered.packet.topic.topic_id);

    UNUSED_RETURN_VALUE(xTaskNotifyGive(mqttsn_task_handle));
}


/**@brief Processes data published by a broker.
 *
 * @details This function processes LED command.
 */
static void received_callback(mqttsn_event_t * p_event)
{
    if (p_event->event_data.published.packet.topic.topic_id == m_topic.topic_id)
    {
        NRF_LOG_INFO("MQTT-SN event: Content to subscribed topic received.\r\n");
    }
    else
    {
        NRF_LOG_INFO("MQTT-SN event: Content to unsubscribed topic received. Dropping packet.\r\n");
    }
}


/**@brief Processes retransmission limit reached event. */
static void timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_INFO("MQTT-SN event: Timed-out message: %d. Message ID: %d.\r\n",
                  p_event->event_data.error.msg_type,
                  p_event->event_data.error.msg_id);
}


/**@brief Processes results of gateway discovery procedure. */
static void searchgw_timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_INFO("MQTT-SN event: Gateway discovery result: 0x%x.\r\n", p_event->event_data.discovery);
    // Notify mqttsn task
    UNUSED_RETURN_VALUE(xTaskNotifyGive(mqttsn_task_handle));
}


/**@brief Function for handling MQTT-SN events. */
void mqttsn_evt_handler(mqttsn_client_t * p_client, mqttsn_event_t * p_event)
{
    switch(p_event->event_id)
    {
        case MQTTSN_EVENT_GATEWAY_FOUND:
            NRF_LOG_INFO("MQTT-SN event: Client has found an active gateway.\r\n");
            gateway_info_callback(p_event);
            break;

        case MQTTSN_EVENT_CONNECTED:
            NRF_LOG_INFO("MQTT-SN event: Client connected.\r\n");
            connected_callback();
            break;

        case MQTTSN_EVENT_DISCONNECT_PERMIT:
            NRF_LOG_INFO("MQTT-SN event: Client disconnected.\r\n");
            disconnected_callback();
            break;

        case MQTTSN_EVENT_REGISTERED:
            NRF_LOG_INFO("MQTT-SN event: Client registered topic.\r\n");
            regack_callback(p_event);
            break;

        case MQTTSN_EVENT_SUBSCRIBED:
            NRF_LOG_INFO("MQTT-SN event: Client subscribed to topic.\r\n");
            break;

        case MQTTSN_EVENT_UNSUBSCRIBED:
            NRF_LOG_INFO("MQTT-SN event: Client unsubscribed to topic.\r\n");
            break;

        case MQTTSN_EVENT_RECEIVED:
            NRF_LOG_INFO("MQTT-SN event: Client received content.\r\n");
            received_callback(p_event);
            break;

        case MQTTSN_EVENT_TIMEOUT:
            NRF_LOG_INFO("MQTT-SN event: Retransmission retries limit has been reached.\r\n");
            timeout_callback(p_event);
            break;

        case MQTTSN_EVENT_SEARCHGW_TIMEOUT:
            NRF_LOG_INFO("MQTT-SN event: Gateway discovery procedure has finished.\r\n");
            searchgw_timeout_callback(p_event);

            break;

        default:
            break;
    }
}

/***************************************************************************************************
 * @section Signal handling
 **************************************************************************************************/

void otTaskletsSignalPending(otInstance * p_instance)
{
    if (thread_stack_task_handle == NULL)
    {
        return;
    }
    UNUSED_RETURN_VALUE(xTaskNotifyGive(thread_stack_task_handle));
}


void otSysEventSignalPending(void)
{
    static BaseType_t xHigherPriorityTaskWoken;
    TaskStatus_t xTaskDetails;
    if (thread_stack_task_handle == NULL)
    {
        return;
    }

    vTaskNotifyGiveFromISR(thread_stack_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/***************************************************************************************************
 * @section State change handling
 **************************************************************************************************/

 static void thread_state_changed_callback(uint32_t flags, void * p_context)
{
    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n", flags, otThreadGetDeviceRole(p_context));
}

/***************************************************************************************************
 * @section Buttons
 **************************************************************************************************/

static void subscribe(void)
{
    uint8_t  topic_name_len = strlen(m_topic_name);
    uint32_t err_code       = NRF_SUCCESS;

    if (m_subscribed)
    {
        err_code = mqttsn_client_unsubscribe(&m_client, m_topic.p_topic_name, topic_name_len, &m_msg_id);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("UNSUBSCRIBE message could not be sent.\r\n");
        }
        else
        {
            m_subscribed = false;
        }
    }
    else
    {
        err_code = mqttsn_client_subscribe(&m_client, m_topic.p_topic_name, topic_name_len, &m_msg_id);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("SUBSCRIBE message could not be sent.\r\n");
        }
        else
        {
            m_subscribed = true;
        }
    }
}


// Manually search for gateway and connect/disconnect
static void bsp_event_handler(bsp_event_t event)
{
    if (otThreadGetDeviceRole(thread_ot_instance_get()) < OT_DEVICE_ROLE_CHILD )
    {
        (void)event;
        return;
    }

    switch (event)
    {

        case BSP_EVENT_KEY_0:
        {
          uint8_t payload = 1;
          NRF_LOG_INFO("PUBLISH");
          uint32_t err_code = mqttsn_client_publish(&m_client, m_topic.topic_id, &payload, 1, &m_msg_id);
          if (err_code != NRF_SUCCESS)
          {
              NRF_LOG_ERROR("PUBLISH message could not be sent. Error code: 0x%x\r\n", err_code)
          }

          break;
        
        }
        case BSP_EVENT_KEY_1:
        {
            uint32_t err_code = mqttsn_client_search_gateway(&m_client, SEARCH_GATEWAY_TIMEOUT);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("SEARCH GATEWAY message could not be sent. Error: 0x%x\r\n", err_code);
            }
            break;
        }

        case BSP_EVENT_KEY_2:
        {
            uint32_t err_code;

            if (mqttsn_client_state_get(&m_client) == MQTTSN_CLIENT_CONNECTED)
            {
                err_code = mqttsn_client_disconnect(&m_client);
                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_ERROR("DISCONNECT message could not be sent. Error: 0x%x\r\n", err_code);
                }
            }
            else
            {
                err_code = mqttsn_client_connect(&m_client, &m_gateway_addr, m_gateway_id, &m_connect_opt);
                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_ERROR("CONNECT message could not be sent. Error: 0x%x\r\n", err_code);
                }
            }
            break;
        }

        case BSP_EVENT_KEY_3:
        {
            subscribe();
            break;
        }

        default:
            break;
    }
}


/**@brief Function for initializing the Thread Board Support Package
 */
static void thread_bsp_init(void)
{
    uint32_t error_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(error_code);

    error_code = bsp_thread_init(thread_ot_instance_get());
    APP_ERROR_CHECK(error_code);
}


/**@brief Function for initializing the Thread Stack
 */
static void thread_instance_init(void)
{
    thread_configuration_t thread_configuration =
    {
        .radio_mode        = THREAD_RADIO_MODE_RX_ON_WHEN_IDLE,
        .autocommissioning = true,
    };

    thread_init(&thread_configuration);
    thread_cli_init();
    thread_state_changed_callback_set(thread_state_changed_callback);
}

/**@brief Function for initializing the MQTTSN client.
 */
static void mqttsn_init(void)
{
    uint32_t err_code = mqttsn_client_init(&m_client,
                                           MQTTSN_DEFAULT_CLIENT_PORT,
                                           mqttsn_evt_handler,
                                           thread_ot_instance_get());
    APP_ERROR_CHECK(err_code);

    connect_opt_init();
}

/**@brief Function for initializing scheduler module.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

void thread_stack_task(void * arg)
{
    UNUSED_PARAMETER(arg);

    scheduler_init();
    thread_instance_init();

    // Notify MQTT task
    UNUSED_RETURN_VALUE(xTaskNotifyGive(mqttsn_task_handle));

    while (1)
    {
        thread_process();
        app_sched_execute();
        
        UNUSED_RETURN_VALUE(ulTaskNotifyTake(pdTRUE, portMAX_DELAY));
    }
}


void publish(mqttsn_msg_queue_element_t msg) {

  if (mqttsn_outgoing_message_queue != NULL && xQueueSend(mqttsn_outgoing_message_queue, &msg, 0) != pdPASS) {
    
    NRF_LOG_ERROR("Failed to post mqttsn message to outgoing message queue");
  
  }

}

void register_topic(TaskHandle_t task, mqttsn_register_topic_queue_element_t topic) {


  if (mqttsn_register_topic_queue != NULL && xQueueSend(mqttsn_register_topic_queue, &topic, 0) != pdPASS) {

    NRF_LOG_ERROR("Failed to post mqttsn register topic message to queue");
  
  }
  
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}



void mqttsn_task(void *arg) {
  
  UNUSED_PARAMETER(arg);
  
  // Wait for thread stack task to initialize
  UNUSED_RETURN_VALUE(ulTaskNotifyTake(pdTRUE, portMAX_DELAY));

  mqttsn_outgoing_message_queue = xQueueCreate(MQTTSN_PACKET_FIFO_MAX_LENGTH, sizeof(mqttsn_msg_queue_element_t));
  mqttsn_register_topic_queue = xQueueCreate(MQTTSN_PACKET_FIFO_MAX_LENGTH, sizeof(mqttsn_register_topic_queue_element_t));
  //registerTopicSemaphore = xSemaphoreCreateBinary();
  if (mqttsn_outgoing_message_queue == NULL || mqttsn_register_topic_queue == NULL) {
    NRF_LOG_ERROR("Not enough heap memory available for mqttsn task");
  }
  
  thread_bsp_init();
  mqttsn_init();

  TickType_t lastWakeTime;
  const TickType_t delay = MQTTSN_TASK_DELAY_SEC;
  mqttsn_client_state_t state = mqttsn_client_state_get(&m_client);
  uint32_t err_code;

  while(1) {
    /*MQTTSN_CLIENT_UNINITIALIZED = 0,       < Client has not been initialized yet. */
    /*MQTTSN_CLIENT_ASLEEP,                  < Client is in sleep mode. */
    /*MQTTSN_CLIENT_AWAKE,                   < Client is awake. */
    /*MQTTSN_CLIENT_CONNECTED,               < Client is connected. */
    /*MQTTSN_CLIENT_DISCONNECTED,            < Client is disconnected. */
    /*MQTTSN_CLIENT_ESTABLISHING_CONNECTION, < Client is attempting to connect. */
    /*MQTTSN_CLIENT_WAITING_FOR_SLEEP,       < Client is waiting for permission to sleep. */
    /*MQTTSN_CLIENT_WAITING_FOR_DISCONNECT,  < Client is waiting for permission to disconnect. */

    while (state == MQTTSN_CLIENT_DISCONNECTED) {
      uint32_t err_code = mqttsn_client_search_gateway(&m_client, SEARCH_GATEWAY_TIMEOUT);
      if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("SEARCH GATEWAY message could not be sent. Error: 0x%x\r\n", err_code);
      }
      
      // Wait here until callback for gateway search is executed
      if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE && found_active_gateway) {
        // Connect to broker
        err_code = mqttsn_client_connect(&m_client, &m_gateway_addr, m_gateway_id, &m_connect_opt);
        if (err_code != NRF_SUCCESS) {
          NRF_LOG_ERROR("CONNECT message could not be sent. Error: 0x%x\r\n", err_code);
        }
        // Wait here until connect callback is executed
        UNUSED_RETURN_VALUE(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)); 
      }
      state = mqttsn_client_state_get(&m_client);
    }
    
    mqttsn_msg_queue_element_t rx_msg;
    if (mqttsn_outgoing_message_queue != NULL && xQueueReceive(mqttsn_outgoing_message_queue, &rx_msg, 0) == pdPASS) {
      
      // TODO: need to associate a certain topic with the registered topic id given by broker
      char payload[] = "{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 7.0, 8.0, 9.0, 10.0}";
      //float p[13] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0};
      uint8_t p_payload = &payload;
      uint32_t err_code = mqttsn_client_publish(&m_client, m_topic.topic_id/*rx_msg.topic.topic_id*/, &payload/*&rx_msg.payload*/, sizeof(payload)/*rx_msg.payload_size*/, &rx_msg.msg_id);

      if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("PUBLISH message could not be sent. Error code: 0x%x\r\n", err_code)
      }

    }
    
    /*mqttsn_register_topic_queue_element_t rx_register;
    if (mqttsn_register_topic_queue != NULL && xQueueReceive(mqttsn_register_topic_queue, &rx_register, 0) == pdPASS) {

      uint32_t err_code = mqttsn_client_topic_register(&m_client, rx_register.topic.p_topic_name, strlen(rx_register.topic.p_topic_name), &rx_register.msg_id);

      if (err_code != NRF_SUCCESS) {
            NRF_LOG_ERROR("REGISTER message could not be sent. Error code: 0x%x\r\n", err_code);
      }
      

    }*/
    
    
    
    uint32_t timer_value = mqttsn_platform_timer_cnt_get();
    NRF_LOG_INFO("time: %" PRIu32, timer_value);
    lastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*delay);
  
  }

}