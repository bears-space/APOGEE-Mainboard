idf_component_register(
    SRCS "recovery_app.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_http_server app_update esp_partition
)
