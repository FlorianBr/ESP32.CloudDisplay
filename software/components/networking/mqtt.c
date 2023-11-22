/**
 ******************************************************************************
 *  file           : mqtt.c
 *  brief          : MQTT Connector
 ******************************************************************************
 *  Copyright (C) 2021 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "mqtt_client.h"


/* Private includes ----------------------------------------------------------*/
#include "../display/include/lcd.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define MQTT_BROKER "mqtt://192.168.178.22:8880" // the AWS endpoint

#define MQTT_TOPIC "IOTDISPLAY"
#define MQTT_CLIENTID "IOTDISPLAY"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char *TAG = "MQTT";
static bool isconnected = false;
static esp_mqtt_client_handle_t client = NULL;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

static esp_err_t event_handler_cb(esp_mqtt_event_handle_t event) {
    int msg_id;
    switch (event->event_id) {
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
            break;
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, MQTT_TOPIC "/cmds/#", 0);
            ESP_LOGI(TAG, "Subscrive successful, msg_id=%d", msg_id);
            isconnected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            isconnected = false;
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            // ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            // ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            // printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            // printf("DATA=%.*s\r\n", event->data_len, event->data);
            if (strncmp(event->topic, MQTT_TOPIC "/cmds/lcdraw", event->topic_len)==0) {
                LCD_SetRawData((uint8_t*)event->data, event->current_data_offset, event->data_len);
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

/**
 * @brief  The MQTT Event handler
 */
static void event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    event_handler_cb(event_data);
}

/**
 * @brief Connect to MQTT
 * 
 * @return esp_err_t 
 */
esp_err_t MQTT_Init(void) {
    isconnected = false;

    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri                = MQTT_BROKER,
        .client_id          = MQTT_CLIENTID,
        .buffer_size        = 8096,
        .out_buffer_size    = 1024,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, event_handler, client);
    esp_mqtt_client_start(client);

    return (ESP_OK);
}

esp_err_t MQTT_SendTime(char *cBuffer) {
    if (!isconnected) return (ESP_FAIL);
    esp_mqtt_client_publish(client, MQTT_TOPIC "/time", cBuffer, 0, 1, 0);
    return (ESP_OK);
}
