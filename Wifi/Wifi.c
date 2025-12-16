#include "Wifi.h"
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_smartconfig.h"
#include "nvs_flash.h"

#define TAG "WIFI"

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0
#define SMARTCONFIG_DONE_BIT BIT1

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "Disconnected, retrying...");
            esp_wifi_connect();
            break;
        }
    }

    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }

    else if (event_base == SC_EVENT) {
        switch (event_id) {
        case SC_EVENT_SCAN_DONE:
            ESP_LOGI(TAG, "SmartConfig: Scan done");
            break;

        case SC_EVENT_FOUND_CHANNEL:
            ESP_LOGI(TAG, "SmartConfig: Channel found");
            break;

        case SC_EVENT_GOT_SSID_PSWD: {
            smartconfig_event_got_ssid_pswd_t *evt = event_data;
            wifi_config_t wifi_config = {0};

            memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
            memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));

            ESP_LOGI(TAG, "SmartConfig SSID:%s", wifi_config.sta.ssid);

            esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
            esp_wifi_connect();
            break;
        }

        case SC_EVENT_SEND_ACK_DONE:
            ESP_LOGI(TAG, "SmartConfig completed");
            xEventGroupSetBits(wifi_event_group, SMARTCONFIG_DONE_BIT);
            break;
        }
    }
}

static void start_smartconfig(void)
{
    ESP_LOGI(TAG, "Starting SmartConfig...");

    xEventGroupClearBits(wifi_event_group, SMARTCONFIG_DONE_BIT);

    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    esp_smartconfig_start(&cfg);

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT | SMARTCONFIG_DONE_BIT,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi connected");
        }

        if (bits & SMARTCONFIG_DONE_BIT) {
            ESP_LOGI(TAG, "SmartConfig done. Stopping...");
            esp_smartconfig_stop();
            break;
        }
    }
}

void wifi_init(void)
{
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    wifi_event_group = xEventGroupCreate();

    // Network + event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Init WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    // Load existing WiFi config
    wifi_config_t wifi_cfg = {0};
    esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);

    bool has_saved_wifi = strlen((char *)wifi_cfg.sta.ssid) > 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    if (has_saved_wifi) {
        ESP_LOGI(TAG, "Connecting to saved WiFi: %s", wifi_cfg.sta.ssid);
        esp_wifi_connect();
    } else {
        start_smartconfig();
    }
}

bool wifi_is_connected(void){
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return true;
    }
    return false;
}
