#include "thread_mqttsn.h"
#include "mqttsn_client.h"
#include "thread_utils.h"
#include "bsp_thread.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "nrf_log.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "mqttsn_platform.h"
#include <openthread/thread.h>
#include "message_buffer.h"
#include <string.h>
#include <inttypes.h>
#include "robot_config.h"

#define SCHED_QUEUE_SIZE       32                                           /**< Maximum number of events in the scheduler queue. */
#define SCHED_EVENT_DATA_SIZE  APP_TIMER_SCHED_EVENT_DATA_SIZE              /**< Maximum app_scheduler event size. */

#define ROBOT_TOPIC_NAME              "v2/robot/NRF_5/adv"
#define ROBOT_LINE_TOPIC_NAME         "v2/robot/NRF_5/line"
#define ROBOT_CTRL_TOPIC_NAME         "v2/robot/NRF_5/controller"
#define ROBOT_TEST_TOPIC_NAME         "v2/robot/NRF_5/test"
#define SERVER_CMD_TOPIC_NAME         "v2/server/NRF_5/cmd"
#define SERVER_INIT_TOPIC_NAME        "v2/server/NRF_5/init"
#define NUM_TOPICS                    6
#define NUM_SUB_TOPICS                2

#define SEARCH_GATEWAY_TIMEOUT        5                                     /**< MQTT-SN Gateway discovery procedure timeout in [s]. */
#define MQTTSN_TASK_DELAY_S           0.1                                   /**< MQTTSN task delay in milliseconds */ 

extern TaskHandle_t thread_stack_task_handle, mqttsn_task_handle;

SemaphoreHandle_t registerTopicSemaphore;
EventGroupHandle_t connectEventGroup;
uint8_t has_finished_connected_callback = 0;

static mqttsn_client_t      m_client;                                       /**< An MQTT-SN client instance. */
static mqttsn_remote_t      m_gateway_addr;                                 /**< A gateway address. */
static uint8_t              m_gateway_id;                                   /**< A gateway ID. */
static mqttsn_connect_opt_t m_connect_opt;                                  /**< Connect options for the MQTT-SN client. */
static uint16_t             m_msg_id           = 0;                         /**< Message ID thrown with MQTTSN_EVENT_TIMEOUT. */
static char                 m_client_id[]      = ROBOT_NAME;                /**< The MQTT-SN Client's ID. */

static mqttsn_topic_t topic_arr[NUM_TOPICS] = 
{
  {
    .p_topic_name = ROBOT_TOPIC_NAME, 
    .topic_id     = NULL
  },
  {
    .p_topic_name = SERVER_CMD_TOPIC_NAME, 
    .topic_id     = NULL
  }, 
  {
    .p_topic_name = SERVER_INIT_TOPIC_NAME,
    .topic_id     = NULL
  }, 
  {
    .p_topic_name = ROBOT_CTRL_TOPIC_NAME,
    .topic_id     = NULL
  },
  {
    .p_topic_name = ROBOT_LINE_TOPIC_NAME,
    .topic_id     = NULL
  },
  {
    .p_topic_name = ROBOT_TEST_TOPIC_NAME,
    .topic_id     = NULL
  }
};

typedef struct mqttsn_subscribe_topic {
  QueueHandle_t queue;
  uint8_t queue_size;
  mqttsn_topic_t* p_topic;
  uint8_t identifier;
}mqttsn_subscribe_topic_t;


static mqttsn_subscribe_topic_t sub_topic_arr[NUM_SUB_TOPICS] = {
  {
    .queue = NULL,
    .queue_size = 1,
    .p_topic = &topic_arr[1], // pointer to v2/server/<ROBOT_NAME>/cmd topic
    .identifier = TARGET_IDENTIFIER
  }, 
  {
    .queue = NULL,
    .queue_size = 1,
    .p_topic = &topic_arr[2], // pointer to v2/server/ROBOT_NAME>/init topic
    .identifier = INIT_IDENTIFIER
  }
};

QueueHandle_t mqttsn_outgoing_message_queue, mqttsn_outgoing_line_message_queue;
SemaphoreHandle_t publish_semaphore;

QueueHandle_t get_queue_handle(char* topic_name) {
  for (uint8_t i=0; i<NUM_SUB_TOPICS; i++) {
    if (!strcmp(sub_topic_arr[i].p_topic->p_topic_name, topic_name)) {
      return sub_topic_arr[i].queue;
    }
  }
}

/***************************************************************************************************
 * @section MQTT-SN
 **************************************************************************************************/


 /**@brief Get topic id for given topic name. 
 * @param[in]    p_topic_name  Pointer to topic name.
 */
uint16_t get_topic_id(char* p_topic_name) {
  for (uint8_t i=0; i<NUM_TOPICS; i++) {
    if (!(strcmp(p_topic_name, topic_arr[i].p_topic_name))) {
      return topic_arr[i].topic_id;
    }
  }
  NRF_LOG_WARNING("Given topic name is not a registered topic");
  return NULL;
}


/**@brief Initializes MQTT-SN client's connection options. */
static void connect_opt_init(void)
{
    m_connect_opt.alive_duration = MQTTSN_DEFAULT_ALIVE_DURATION,
    m_connect_opt.clean_session  = MQTTSN_DEFAULT_CLEAN_SESSION_FLAG,
    m_connect_opt.will_flag      = MQTTSN_DEFAULT_WILL_FLAG,
    m_connect_opt.client_id_len  = strlen(m_client_id),

    memcpy(m_connect_opt.p_client_id,  (unsigned char *)m_client_id,  m_connect_opt.client_id_len);
}

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
}


/**@brief Processes CONNACK message from a gateway.
 *
 * @details This function launches the topic registration procedure if necessary.
 */
static void connected_callback(void)
{
    
    // Register topics, i.e. bind long topic name to id specified by broker
    for (uint8_t i=0; i<NUM_TOPICS; i++) {
      mqttsn_topic_t topic = topic_arr[i];
      NRF_LOG_INFO("Request to register topic %s", NRF_LOG_PUSH(topic.p_topic_name));
      uint32_t err_code = mqttsn_client_topic_register(&m_client,
                                                       topic.p_topic_name,
                                                       strlen(topic.p_topic_name),
                                                       &m_msg_id);
      if (err_code != NRF_SUCCESS)
      {
          NRF_LOG_ERROR("REGISTER message could not be sent. Error code: 0x%x\r\n", err_code);
      }
    }

    // Subscribe to topics
    for (uint8_t i=0; i<NUM_SUB_TOPICS; i++) {
      mqttsn_topic_t* p_topic = sub_topic_arr[i].p_topic;
      uint32_t err_code = mqttsn_client_subscribe(&m_client, p_topic->p_topic_name, strlen(p_topic->p_topic_name), &m_msg_id);
      if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("SUBSCRIBE message could not be sent.\r\n");
      }
    }

    has_finished_connected_callback = 1;

}


/**@brief Processes DISCONNECT message from a gateway. */
static void disconnected_callback(void)
{
    NRF_LOG_INFO("MQTT-SN event: Disconnected");
}

uint8_t all_topics_registered() {
  for (uint8_t i=0; i<NUM_TOPICS; i++) {
    if (topic_arr[i].topic_id == NULL) {
      // Not all topics have been registered
      return 0;
    }
  }
  // All topics have been registered
  return 1;

}


/**@brief Processes REGACK message from a gateway.
 *
 * @param[in] p_event Pointer to MQTT-SN event.
 */
static void regack_callback(mqttsn_event_t * p_event)
{
    mqttsn_topic_t registered_topic = p_event->event_data.registered.packet.topic;
    for (int8_t i=0; i<NUM_TOPICS; i++) {
      if (!strcmp(registered_topic.p_topic_name, topic_arr[i].p_topic_name)) {
        // Set topic id corresponding to registered topic name
        mqttsn_topic_t* p_topic = &topic_arr[i];
        p_topic->topic_id = registered_topic.topic_id;
        break;
      }
    }
    NRF_LOG_INFO("MQTT-SN event: Topic %s has been registered with ID: %d.\r\n", 
                  NRF_LOG_PUSH(p_event->event_data.registered.packet.topic.p_topic_name),
                  p_event->event_data.registered.packet.topic.topic_id); 
    if (all_topics_registered()) {

      // Start main loop of mqttsn task
      UNUSED_RETURN_VALUE(xTaskNotifyGive(mqttsn_task_handle));
    
    }

}


/**@brief Processes data published by a broker. */
static void received_callback(mqttsn_event_t * p_event)
{
    for (uint8_t i=0; i<NUM_SUB_TOPICS; i++) {
      mqttsn_subscribe_topic_t sub_topic = sub_topic_arr[i];
      
      // Find a match for topic id
      if (p_event->event_data.published.packet.topic.topic_id == sub_topic.p_topic->topic_id) {

        // The value which p_payload points to must be copied, since the pointer value 
        // may be overwritten with new data before the receiving task has read it
        uint8_t* payload = p_event->event_data.published.p_payload;

        switch(sub_topic.identifier) {
          case INIT_IDENTIFIER: 
            {
              mqttsn_init_msg_t rx_init_msg;
              memcpy(&rx_init_msg, payload, sizeof(mqttsn_init_msg_t));
              if (xQueueSend(sub_topic.queue, (void*)&rx_init_msg, 0) != pdPASS) {
                NRF_LOG_ERROR("Failed to post received payload from topic %s to queue", NRF_LOG_PUSH(sub_topic.p_topic->p_topic_name));
              }
              break;
            }
          case TARGET_IDENTIFIER:
          {
            mqttsn_target_msg_t rx_target_msg;
            memcpy(&rx_target_msg, payload, sizeof(mqttsn_target_msg_t));
            if (xQueueSend(sub_topic.queue, (void*)&rx_target_msg, 0) != pdPASS) {
              NRF_LOG_ERROR("Failed to post received payload from topic %s to queue", NRF_LOG_PUSH(sub_topic.p_topic->p_topic_name));
            }
            break;
          }
        
        }
        break; // exit for-loop
      }
    }
}


/**@brief Processes retransmission limit reached event. */
static void timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_INFO("MQTT-SN event: Timed-out message: %d. Message ID: %d.\r\n",
                  p_event->event_data.error.msg_type,
                  p_event->event_data.error.msg_id);

   switch(p_event->event_data.error.msg_type) {
    case MQTTSN_PACKET_CONNACK:
      NRF_LOG_WARNING("CONNACK not received, retrying connect...");
      uint32_t err_code = mqttsn_client_connect(&m_client, &m_gateway_addr, m_gateway_id, &m_connect_opt);
      if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("CONNECT message could not be sent. Error: 0x%x\r\n", err_code);
      }
      break;
    case MQTTSN_PACKET_PUBACK:
      switch(p_event->event_data.error.error) {
        case MQTTSN_ERROR_REJECTED_CONGESTION:
          NRF_LOG_WARNING("PUBACK not received due to network congestion");
          // TODO
          break;
        case MQTTSN_ERROR_TIMEOUT:
          NRF_LOG_WARNING("PUBACK not received due to timeout");
          // TODO
          break;
      }
      break;
    case MQTTSN_PACKET_PINGREQ:
      NRF_LOG_ERROR("PINGREQ not received, trying to reconnect...");
      mqttsn_client_disconnect(&m_client); // main loop of mqttsn task will try to reconnect
      break;
   }
   

}


/**@brief Processes results of gateway discovery procedure. */
static void searchgw_timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_INFO("MQTT-SN event: Gateway discovery result: 0x%x.\r\n", p_event->event_data.discovery);
    // Notify mqttsn task
    //UNUSED_RETURN_VALUE(xTaskNotifyGive(mqttsn_task_handle));
    uint32_t err_code;

    switch(p_event->event_data.discovery) {
      
      case MQTTSN_SEARCH_GATEWAY_FINISHED:
        // TODO
        break;
      
      case MQTTSN_SEARCH_GATEWAY_NO_GATEWAY_FOUND:
        NRF_LOG_INFO("Retrying gateway search...\n\r");
        err_code = mqttsn_client_search_gateway(&m_client, SEARCH_GATEWAY_TIMEOUT);
        if (err_code != NRF_SUCCESS) {
          NRF_LOG_ERROR("SEARCH GATEWAY message could not be sent. Error: 0x%x\r\n", err_code);
        }
        break;

      case MQTTSN_SEARCH_GATEWAY_PLATFORM_FAILED:
        NRF_LOG_ERROR("Search gateway platform failed\n\r");
        // TODO
        break;

      case MQTTSN_SEARCH_GATEWAY_TRANSPORT_FAILED:
        NRF_LOG_ERROR("Search gateway transport failed");
        // TODO
        break;
    }
}


/**@brief Function for handling MQTT-SN events. */
void mqttsn_evt_handler(mqttsn_client_t * p_client, mqttsn_event_t * p_event)
{
    switch(p_event->event_id)
    {
        case MQTTSN_EVENT_GATEWAY_FOUND:
            NRF_LOG_INFO("MQTT-SN event: Client has found an active gateway.\r\n");
            gateway_info_callback(p_event);
            uint32_t err_code = mqttsn_client_connect(&m_client, &m_gateway_addr, m_gateway_id, &m_connect_opt);
            if (err_code != NRF_SUCCESS) {
              NRF_LOG_ERROR("CONNECT message could not be sent. Error: 0x%x\r\n", err_code);
            }
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
          uint8_t payload = 0x01;
          uint32_t err_code = publish(ROBOT_TOPIC_NAME, &payload, sizeof(uint8_t), 0, 0);
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
            NRF_LOG_INFO("BSP EVENT KEY 3");
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

    scheduler_init(); // Is the scheduler needed?
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

uint32_t publish_line(char* topic_name, mqttsn_line_msg_t payload, uint8_t payload_size, uint8_t qos, uint16_t msg_id) {
  if (!mqttsn_client_is_connected()) {
    return NRF_ERROR_BUSY;
  }
  
  mqttsn_line_msg_queue_element_t queue_element;
  queue_element.msg_id = msg_id;
  queue_element.qos = qos;
  queue_element.payload_size = payload_size;
  queue_element.payload = payload;
  queue_element.topic_id = get_topic_id(topic_name);
  if (queue_element.topic_id == NULL) {
    return NRF_ERROR_NULL;
  }

  if (mqttsn_outgoing_line_message_queue != NULL && xQueueSend(mqttsn_outgoing_line_message_queue, &queue_element, 10) != pdPASS) {
    
    NRF_LOG_ERROR("Failed to post mqttsn message to outgoing message queue");
  
  }
  return NRF_SUCCESS;

}

uint32_t publish(char* topic_name, void* p_payload, uint8_t payload_size, uint8_t qos, uint16_t msg_id) {

  if (!mqttsn_client_is_connected()) {
    return NRF_ERROR_BUSY;
  }
  mqttsn_msg_queue_element_t msg;
  
  // find topic_id from topic_name
  msg.topic_id = get_topic_id(topic_name);
  if (msg.topic_id == NULL) {
    return NRF_ERROR_NULL;
  }
  msg.payload = p_payload;
  msg.payload_size = payload_size;
  msg.qos = qos;
  msg.msg_id = msg_id;

  if (mqttsn_outgoing_message_queue != NULL && xQueueSend(mqttsn_outgoing_message_queue, &msg, 10) != pdPASS) {
    
    NRF_LOG_ERROR("Failed to post mqttsn message to outgoing message queue");
  
  }
  return NRF_SUCCESS;
}

uint8_t mqttsn_client_is_connected() {
  mqttsn_client_state_t state = mqttsn_client_state_get(&m_client);
  if (state == MQTTSN_CLIENT_CONNECTED && has_finished_connected_callback) {
    return 1;
  }
  return 0;
}


void mqttsn_task(void *arg) {
  
  UNUSED_PARAMETER(arg);
  
  // Wait for thread stack task to initialize
  UNUSED_RETURN_VALUE(ulTaskNotifyTake(pdTRUE, portMAX_DELAY));

  mqttsn_outgoing_message_queue = xQueueCreate(MQTTSN_PACKET_FIFO_MAX_LENGTH, sizeof(mqttsn_msg_queue_element_t));
  mqttsn_outgoing_line_message_queue = xQueueCreate(MQTTSN_PACKET_FIFO_MAX_LENGTH, sizeof(mqttsn_line_msg_queue_element_t));

  if (mqttsn_outgoing_message_queue == NULL || mqttsn_outgoing_line_message_queue == NULL) {
    NRF_LOG_ERROR("Not enough heap memory available for mqttsn task");
    while(1) {
      // do nothing
    }
  }

  for (uint8_t i=0; i<NUM_SUB_TOPICS; i++) {
    mqttsn_subscribe_topic_t* p_sub_topic = &sub_topic_arr[i];

    if (p_sub_topic->identifier == INIT_IDENTIFIER) {
      p_sub_topic->queue = xQueueCreate(p_sub_topic->queue_size, sizeof(mqttsn_init_msg_t));
    } 
    else if (p_sub_topic->identifier == TARGET_IDENTIFIER) {
      p_sub_topic->queue = xQueueCreate(p_sub_topic->queue_size, sizeof(mqttsn_target_msg_t));
    }

    if (p_sub_topic->queue == NULL) {
       NRF_LOG_ERROR("Not enough heap memory available for mqttsn task");
       while(1) {
        // do nothing
       }
    } else {
      vQueueAddToRegistry(p_sub_topic->queue, p_sub_topic->p_topic->p_topic_name);
    }

  }
  
  thread_bsp_init();
  mqttsn_init();

  TickType_t lastWakeTime;
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

    mqttsn_client_state_t state = mqttsn_client_state_get(&m_client);

    while (state == MQTTSN_CLIENT_DISCONNECTED) {

      uint32_t err_code = mqttsn_client_search_gateway(&m_client, SEARCH_GATEWAY_TIMEOUT);
      if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("SEARCH GATEWAY message could not be sent. Error: 0x%x\r\n", err_code);
      }

      // Wait here until connect callback is executed
      UNUSED_RETURN_VALUE(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)); 

      state = mqttsn_client_state_get(&m_client);
    }
    mqttsn_msg_queue_element_t rx_msg;
    mqttsn_line_msg_queue_element_t rx_line_msg;
    /*if (mqttsn_outgoing_message_queue != NULL && xQueueReceive(mqttsn_outgoing_message_queue, &rx_msg, 0) == pdPASS) {
      
      uint32_t err_code = mqttsn_client_publish(&m_client, rx_msg.topic_id, rx_msg.payload, rx_msg.payload_size, &rx_msg.msg_id);

      if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("PUBLISH message could not be sent. Error code: 0x%x\r\n", err_code)
      }

    } else {
      taskYIELD();
    }*/
    
    while (mqttsn_outgoing_message_queue != NULL && xQueueReceive(mqttsn_outgoing_message_queue, &rx_msg, 0) == pdPASS) {

      uint32_t err_code = mqttsn_client_publish(&m_client, rx_msg.topic_id, rx_msg.payload, rx_msg.payload_size, rx_msg.qos, &rx_msg.msg_id);
    
      if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("PUBLISH message could not be sent. Error code: 0x%x\r\n", err_code)
      }

    }

    while (mqttsn_outgoing_line_message_queue != NULL && xQueueReceive(mqttsn_outgoing_line_message_queue, &rx_line_msg, 0) == pdPASS) {

      uint32_t err_code = mqttsn_client_publish(&m_client, rx_line_msg.topic_id, &rx_line_msg.payload, rx_line_msg.payload_size, rx_line_msg.qos, &rx_line_msg.msg_id);

      if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("PUBLISH message could not be sent. Error code: 0x%x\r\n", err_code)
      }
    }
    
    //lastWakeTime = xTaskGetTickCount();
    //vTaskDelayUntil(&lastWakeTime, configTICK_RATE_HZ*1);
    taskYIELD();
  
  }

}