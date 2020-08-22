/******************************************************************************
* File Name: mqtt_operation.c
*
* Description: This file contains threads, functions, and other resources related
* to MQTT operations.
*
******************************************************************************
* Copyright (2019), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*****************************************​**************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "aws_demo.h"
#include "cJSON.h"
#include "mqtt_operation.h"

/* Set up logging for this demo. */
#include "iot_demo_logging.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define MAX_DUTY_CYCLE              (100)

/* Subtracting from MAX_DUTY_CYCLE since the LED is connected in active low configuration */
#define GET_DUTY_CYCLE(x)           (MAX_DUTY_CYCLE - x)

/* PWM frequency used to drive LED */
#define PWM_LED_FREQ_HZ             (1000000u)

/* Thread delays */
#define MQTT_THREAD_LOOP_DELAY_MS   (100)

/* MQTT Publish Topics */
//#define TOPIC_SHADOW_UPDATE         "$aws/things/capsense/shadow/update"
//#define TOPIC_SHADOW_GET            "$aws/things/capsense/shadow/get"
#define TOPIC_SHADOW_UPDATE         "set"
#define TOPIC_SHADOW_GET            "get"

/*******************************************************************************
 * Global variable
 ******************************************************************************/
QueueHandle_t bowl_command_data_q;
cyhal_pwm_t pwm_led;
bool led_on = false;

/*******************************************************************************
 * Forward Declaration
 ******************************************************************************/
void updateLedBrightness(uint32_t brightness);
void updateLedState(const char* command);

/*************** MQTT Publish Thread ***************/
/*
 * Summary: This task receives command from CapSense Task and publishes the state of
 * buttons/slider to an AWS thing shadow
 *
 *  @param[in] arg argument for the thread
 *
 */
void task_mqtt(void* param)
{
    /* Suppress warning for unused parameter */
    (void)param;

    mqtt_data_t mqtt_cmd_data;  /* Queue to receive command from CapSense task */
    char json[MAX_JSON_MESSAGE_LENGTH]; /* JSON message to send */
    char topic[MAX_TOPIC_LENGTH];       /* Buffer for topic name */
    uint16_t topicLength = 0;   /* Length of topic */
    uint16_t messageLength = 0; /* Length of JSON message */

    /* Configure the TCPWM for driving led */
    cyhal_pwm_init(&pwm_led, CYBSP_USER_LED, NULL);

    /* Send command to get status of LED from the device shadow */
    mqtt_cmd_data.command = GET_BOWL_STATE;
    xQueueOverwrite(bowl_command_data_q, &mqtt_cmd_data);

    while (1)
    {
        /* Block until a command has been received over queue */
        xQueueReceive(bowl_command_data_q, &mqtt_cmd_data, portMAX_DELAY);

        topicLength = snprintf(topic, sizeof(topic), TOPIC_SHADOW_UPDATE);

        switch(mqtt_cmd_data.command)
        {
            /* Turn on the servo and stepper. */
            case BOWL_TURN_ON:
            {
                messageLength = snprintf(json, sizeof(json), "{\"pos\":1 \"rpm\":1}");
                break;
            }
            /* Turn off servo and stepper */
            case BOWL_TURN_OFF:
            {
            	messageLength = snprintf(json, sizeof(json), "{\"pos\":0 \"rpm\":0}");
                break;
            }
            /* Update stepper speed */
            case BOWL_UPDATE_SPEED:
            {
                messageLength = snprintf(json, sizeof(json), "{\"speed\": %d}", (int)mqtt_cmd_data.speed);
                break;
            }
            /* Send empty message and set topic to get shadow status */
            case GET_BOWL_STATE:
            {
                messageLength = snprintf(json, sizeof(json), "{}");
                topicLength = snprintf(topic, sizeof(topic), TOPIC_SHADOW_GET);
                break;
            }
            /* Invalid command */
            default:
            {
                /* Handle invalid command here */
                break;
            }
        }

        /* PUBLISH (and wait) for all messages. */
        PublishMessage( mqtt_connection,
                        topic,
                        topicLength,
                        json,
                        messageLength);

        vTaskDelay(pdMS_TO_TICKS(MQTT_THREAD_LOOP_DELAY_MS));
    }
}

/*************** Initialize MQTT library***************/
/* @return `EXIT_SUCCESS` if all libraries were successfully initialized; EXIT_FAILURE` otherwise. */
int InitializeMqtt( void )
{
    int status = EXIT_SUCCESS;
    IotMqttError_t mqttInitStatus = IOT_MQTT_SUCCESS;

    mqttInitStatus = IotMqtt_Init();

    if( mqttInitStatus != IOT_MQTT_SUCCESS )
    {
        /* Failed to initialize MQTT library. */
        configPRINTF(("Failed to initialize MQTT library\r\n"));
        status = EXIT_FAILURE;
    }
    return status;
}

/*************** Establish a new connection to the MQTT server ***************/
/*
 * Summary: Establish a new connection to the MQTT server.
 *
 * @param[in] awsIotMqttMode Specify if this demo is running with the AWS IoT
 * MQTT server. Set this to `false` if using another MQTT server.
 * @param[in] pIdentifier NULL-terminated MQTT client identifier.
 * @param[in] pNetworkServerInfo Passed to the MQTT connect function when
 * establishing the MQTT connection.
 * @param[in] pNetworkCredentialInfo Passed to the MQTT connect function when
 * establishing the MQTT connection.
 * @param[in] pNetworkInterface The network interface to use for this demo.
 * @param[out] pmqtt_connection Set to the handle to the new MQTT connection.
 *
 * @return `EXIT_SUCCESS` if the connection is successfully established; `EXIT_FAILURE`
 * otherwise.
 */
int EstablishMqttConnection( bool awsIotMqttMode,
                             const char * pIdentifier,
                             void * pNetworkServerInfo,
                             void * pNetworkCredentialInfo,
                             const IotNetworkInterface_t * pNetworkInterface,
                             IotMqttConnection_t * pmqtt_connection )
{
    int status = EXIT_SUCCESS;
    IotMqttError_t connectStatus = IOT_MQTT_STATUS_PENDING;
    IotMqttNetworkInfo_t networkInfo = IOT_MQTT_NETWORK_INFO_INITIALIZER;
    IotMqttConnectInfo_t connectInfo = IOT_MQTT_CONNECT_INFO_INITIALIZER;

    /*
     * Set the members of the network info not set by the initializer. This
     * struct provided information on the transport layer to the MQTT connection.
     */
    networkInfo.createNetworkConnection = true;
    networkInfo.u.setup.pNetworkServerInfo = pNetworkServerInfo;
    networkInfo.u.setup.pNetworkCredentialInfo = pNetworkCredentialInfo;
    networkInfo.pNetworkInterface = pNetworkInterface;

    #if ( IOT_MQTT_ENABLE_SERIALIZER_OVERRIDES == 1 ) && defined( IOT_DEMO_MQTT_SERIALIZER )
        networkInfo.pMqttSerializer = IOT_DEMO_MQTT_SERIALIZER;
    #endif

    /* Set the members of the connection info not set by the initializer. */
    connectInfo.awsIotMqttMode = awsIotMqttMode;
    connectInfo.cleanSession = true;
    connectInfo.keepAliveSeconds = KEEP_ALIVE_SECONDS;
    connectInfo.pWillInfo = NULL;

    /* Use the parameter client identifier provided. */
    if( pIdentifier != NULL )
    {
        connectInfo.pClientIdentifier = pIdentifier;
        connectInfo.clientIdentifierLength = ( uint16_t ) strlen( pIdentifier );
    }

    connectStatus = IotMqtt_Connect( &networkInfo,
                                     &connectInfo,
                                     MQTT_TIMEOUT_MS,
                                     pmqtt_connection );

    if( connectStatus != IOT_MQTT_SUCCESS )
    {
        status = EXIT_FAILURE;
    }

    return status;
}

/*************** Modify MQTT Subscription ***************/
/*
 * Summary: Add or remove subscriptions by either subscribing or unsubscribing.
 *
 * @param[in] mqtt_connection The MQTT connection to use for subscriptions.
 * @param[in] operation Either #IOT_MQTT_SUBSCRIBE or #IOT_MQTT_UNSUBSCRIBE.
 * @param[in] pTopicFilters Array of topic filters for subscriptions.
 * @param[in] pTopicFilterLength Array of length of topic filters.
 * @param[in] filterCount Number of topic filters.
 * @param[in] pCallbackParameter The parameter to pass to the subscription
 * callback.
 *
 * @return `EXIT_SUCCESS` if the subscription operation succeeded; `EXIT_FAILURE`
 * otherwise.
 */
int ModifySubscriptions( IotMqttConnection_t mqtt_connection,
                         IotMqttOperationType_t operation,
                         const char ** pTopicFilters,
                         const uint16_t *pTopicFilterLength,
                         const uint32_t filterCount,
                         void * pCallbackParameter )
{
    int status = EXIT_SUCCESS;
    uint32_t i = 0;
    IotMqttError_t subscriptionStatus = IOT_MQTT_STATUS_PENDING;
    IotMqttSubscription_t pSubscriptions[ filterCount ];

    /* Set the members of the subscription list. */
    for( i = 0; i < filterCount; i++ )
    {
        pSubscriptions[ i ].qos = IOT_MQTT_QOS_1;
        pSubscriptions[ i ].pTopicFilter = pTopicFilters[ i ];
        pSubscriptions[ i ].topicFilterLength = pTopicFilterLength[ i ];
        pSubscriptions[ i ].callback.pCallbackContext = pCallbackParameter;
        pSubscriptions[ i ].callback.function = MqttSubscriptionCallback;
    }

    /* Modify subscriptions by either subscribing or unsubscribing. */
    if( operation == IOT_MQTT_SUBSCRIBE )
    {
        subscriptionStatus = IotMqtt_TimedSubscribe( mqtt_connection,
                                                     pSubscriptions,
                                                     filterCount,
                                                     0,
                                                     MQTT_TIMEOUT_MS );

        /* Check the status of SUBSCRIBE. */
        switch( subscriptionStatus )
        {
            case IOT_MQTT_SUCCESS:
                IotLogInfo( "All demo topic filter subscriptions accepted.\r\n");
                break;

            case IOT_MQTT_SERVER_REFUSED:

                /* Check which subscriptions were rejected before exiting the demo. */
                for( i = 0; i < filterCount; i++ )
                {
                    if( IotMqtt_IsSubscribed( mqtt_connection,
                                              pSubscriptions[ i ].pTopicFilter,
                                              pSubscriptions[ i ].topicFilterLength,
                                              NULL ) == true )
                    {
                        IotLogInfo("Topic filter %.*s was accepted.\r\n",
                                    pSubscriptions[ i ].topicFilterLength,
                                    pSubscriptions[ i ].pTopicFilter);
                    }
                    else
                    {
                        IotLogInfo("Topic filter %.*s was rejected.\r\n",
                                    pSubscriptions[ i ].topicFilterLength,
                                    pSubscriptions[ i ].pTopicFilter);
                    }
                }

                status = EXIT_FAILURE;
                break;

            default:

                status = EXIT_FAILURE;
                break;
        }
    }
    else if( operation == IOT_MQTT_UNSUBSCRIBE )
    {
        subscriptionStatus = IotMqtt_TimedUnsubscribe( mqtt_connection,
                                                       pSubscriptions,
                                                       filterCount,
                                                       0,
                                                       MQTT_TIMEOUT_MS );

        /* Check the status of UNSUBSCRIBE. */
        if( subscriptionStatus != IOT_MQTT_SUCCESS )
        {
            status = EXIT_FAILURE;
        }
    }
    else
    {
        /* Only SUBSCRIBE and UNSUBSCRIBE are valid for modifying subscriptions. */
        IotLogInfo("MQTT operation %s is not valid for modifying subscriptions.\r\n",
                    IotMqtt_OperationType( operation ) );

        status = EXIT_FAILURE;
    }

    return status;
}

/*************** Publish MQTT Message ***************/
/*
 * Summary: Publish MQTT Message
 *
 * @param[in] mqtt_connection The MQTT connection to use for publishing.
 * @param[in] Topic name for publishing.
 * @param[in] Topic name length.
 * @param[in] Message for publishing.
 * @param[in] Message length.
 *
 * @return `EXIT_SUCCESS` if all messages are published and received; `EXIT_FAILURE`
 * otherwise.
 */
int PublishMessage( IotMqttConnection_t mqtt_connection,
                    char* topic,
                    uint16_t topicLength,
                    char* mqttMessage,
                    uint16_t messageLength)
{
    int status = EXIT_SUCCESS;
    IotMqttError_t publishStatus = IOT_MQTT_STATUS_PENDING;
    IotMqttPublishInfo_t publishInfo = IOT_MQTT_PUBLISH_INFO_INITIALIZER;
    IotMqttCallbackInfo_t publishComplete = IOT_MQTT_CALLBACK_INFO_INITIALIZER;

    /* The MQTT library should invoke this callback when a PUBLISH message
     * is successfully transmitted. Assign NULL to not attach a callback */
    publishComplete.function = NULL;

    /* Pass the PUBLISH number to the operation complete callback.
       Assigned NULL since callback is not attached */
    publishComplete.pCallbackContext = NULL;

    /* Set the common members of the publish info. */
    publishInfo.qos = IOT_MQTT_QOS_1;
    publishInfo.topicNameLength = topicLength;
    publishInfo.pPayload = mqttMessage;
    publishInfo.retryMs = PUBLISH_RETRY_MS;
    publishInfo.retryLimit = PUBLISH_RETRY_LIMIT;
    publishInfo.payloadLength = ( size_t ) messageLength;
    publishInfo.pTopicName = topic;

    /* PUBLISH a message */
    publishStatus = IotMqtt_Publish( mqtt_connection,
                                     &publishInfo,
                                     0,
                                     &publishComplete,
                                     NULL );

    if( publishStatus != IOT_MQTT_STATUS_PENDING )
    {
        status = EXIT_FAILURE;
    }
    return status;
}

/***************  MQTT Subscription callback ***************/
/*
 * Summary: Called by the MQTT library when an incoming PUBLISH message is received.
 *
 * The demo uses this callback to handle incoming PUBLISH messages.
 * @param[in] param1 Counts the total number of received PUBLISH messages. This
 * callback will increment this counter.
 * @param[in] pPublish Information about the incoming PUBLISH message passed by
 * the MQTT library.
 */
void MqttSubscriptionCallback( void * param1,
                               IotMqttCallbackParam_t * const pPublish )
{
    ( void )param1;             /* Suppress compiler warning */
    char topicStr[MAX_TOPIC_LENGTH] = {0};    /* String to copy the topic into */
    char pubType[20] =  {0};    /* String to compare to the publish type */

    /* Copy the message to a null terminated string */
    memcpy(topicStr, pPublish->u.message.info.pTopicName, pPublish->u.message.info.topicNameLength);
    topicStr[pPublish->u.message.info.topicNameLength+1] = 0; /* Add termination */

    /* Copy the message to a null terminated string */
    char *pPayload = (char*)pvPortMalloc(pPublish->u.message.info.payloadLength + 1);
    memcpy(pPayload, pPublish->u.message.info.pPayload, pPublish->u.message.info.payloadLength);
    topicStr[pPublish->u.message.info.topicNameLength] = 0; /* Add termination */


    /* Scan the topic to extract publish type */
    sscanf(topicStr, "%19s", pubType);

    /* Check to see if it is an initial get of the values of other things */
    if(strcmp(pubType, TOPIC_SHADOW_UPDATE) == 0)
    {
        /* Parse JSON message */
        cJSON *root = cJSON_Parse(pPayload);
        cJSON *state = cJSON_GetObjectItem(root,"state");
        cJSON *reported = cJSON_GetObjectItem(state,"reported");

        if(reported == NULL)
        {
            /* If there is no previous shadow state, set LED to max brightness and start PWM */
            updateLedState("LED ON");
            updateLedBrightness(MAX_DUTY_CYCLE);
        }
        else
        {
            cJSON *command = cJSON_GetObjectItem(reported,"Command");
            cJSON *brightness = cJSON_GetObjectItem(reported,"Brightness");

            if(command != NULL)
            {
                updateLedState(command->valuestring);
            }
            if(brightness != NULL)
            {
                updateLedBrightness(brightness->valueint);
            }
        }
        cJSON_Delete(root);
    }

    if(strcmp(pubType,"update/documents") == 0)
    {
        /* Parse JSON message */
        cJSON *root = cJSON_Parse(pPayload);
        cJSON *current = cJSON_GetObjectItem(root,"current");
        cJSON *state = cJSON_GetObjectItem(current,"state");
        cJSON *reported = cJSON_GetObjectItem(state,"reported");
        cJSON *command = cJSON_GetObjectItem(reported,"Command");
        cJSON *brightness = cJSON_GetObjectItem(reported,"Brightness");

        if(command != NULL)
        {
            updateLedState(command->valuestring);
        }
        if(brightness != NULL)
        {
            updateLedBrightness(brightness->valueint);
        }
        cJSON_Delete(root);
    }
    vPortFree(pPayload);
}

/***************  Turn ON/OFF LED  ***************/
void updateLedState(const char* command)
{
    if(0 == strcmp(command, "LED ON"))
    {
        if (!led_on)
        {
            /* Start PWM to turn the LED on */
            cyhal_pwm_start(&pwm_led);
            led_on = true;
        }
    }
    else if(0 == strcmp(command, "LED OFF"))
    {
        if(led_on)
        {
            /* Stop PWM to turn the LED off */
            cyhal_pwm_stop(&pwm_led);
            led_on = false;
        }
    }
}

/***************  Change LED Brightness  ***************/
void updateLedBrightness(uint32_t brightness)
{
    cyhal_pwm_set_duty_cycle(&pwm_led, GET_DUTY_CYCLE(brightness), PWM_LED_FREQ_HZ);
}
