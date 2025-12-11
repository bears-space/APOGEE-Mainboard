#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_event.h"
#include "protocol_examples_common.h"

#include "http_server.h"

static const char *TAG = "vigilant";


void vigilant_init() {
    ESP_LOGI(TAG, "Init NVS");
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_LOGI(TAG, "Init netif + event loop");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "Connecting (WiFi/Ethernet)...");
    ESP_ERROR_CHECK(example_connect());

    ESP_LOGI(TAG, "Registering HTTP server event handlers");
    ESP_ERROR_CHECK(http_server_register_event_handlers());

    ESP_LOGI(TAG, "Starting HTTP server");
    ESP_ERROR_CHECK(http_server_start());
}