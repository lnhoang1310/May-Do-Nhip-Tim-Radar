#include "driver/uart.h"
#include "esp_log.h"
#include "Service_MQTT.h"
#include "BGT60TR13C_API_HeartRate.h"
#include "Wifi.h"

const char* TAG = "Main";

void app_main(void) {
    uart_set_baudrate(UART_NUM_0, 115200);
    wifi_init();
    while(!wifi_is_connected()){
        ESP_LOGI(TAG, "Waiting for wifi ...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "Wifi Connected");
    MQTT_Init();
    BGT60TR13C_Init(SPI2_HOST);
}