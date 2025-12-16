#include "Service_MQTT.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include <string.h>
#include <stdlib.h>
#include "BGT60TR13C_API_HeartRate.h"

static const char* TAG = "MQTT_Service";
static const char* topic_pub = "heart_rate/status";
static const char* topic_sub = "heart_rate/cmd";

static esp_mqtt_client_handle_t client = NULL;
static void MQTT_Parse_Message(const char* json){
    cJSON *root = cJSON_Parse(json);
    if(!root){
        ESP_LOGE(TAG, "JSON ERROR!");
        return;
    }
    char* cmd = cJSON_GetObjectItem(root, "Status")->valuestring;
    if(cmd){
        if(strcmp(cmd, "Start") == 0) Start_Measuring();
        else if(strcmp(cmd, "Stop") == 0) Stop_Measuring();
        else ESP_LOGI(TAG, "Message MQTT Error");
    }
    cJSON_Delete(root);
}

void MQTT_Publish(const char* message, float hear_rate){
    if (!client)
        return;

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "hr", hear_rate);
    cJSON_AddStringToObject(root, "status", message);

    char* json = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(client, topic_pub, json, 0, 1, 0);
    ESP_LOGI(TAG, "Publish [%s]: %s", topic_pub, json);
    free(json);
    cJSON_Delete(root);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT Connected");
        esp_mqtt_client_subscribe(client, topic_sub, 1);
        break;

    case MQTT_EVENT_DATA:
    {
        int len = event->data_len;
        if (len > 500)
            len = 500;

        char data[512] = {0};
        memcpy(data, event->data, len);

        ESP_LOGI(TAG, "Received: %s", data);
        MQTT_Parse_Message(data);
    }
    break;

    default:
        break;
    }
}

void MQTT_Init(void){
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = "mqtt://broker.hivemq.com:1883",
        .credentials.client_id = "ESP32_Client",
    };
    client = esp_mqtt_client_init(&cfg);

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);

    esp_mqtt_client_start(client);
    ESP_LOGI(TAG, "MQTT Started");
}