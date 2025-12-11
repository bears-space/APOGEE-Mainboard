// ota_http.c

#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "ota_http.h"

static const char *TAG_OTA = "ota_http";

#define OTA_RECV_BUF_SIZE 1024

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    esp_err_t err;
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    if (update_partition == NULL) {
        ESP_LOGE(TAG_OTA, "No OTA partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_OTA, "Writing to partition subtype %d at offset 0x%lx",
             update_partition->subtype, (unsigned long)update_partition->address);

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_OTA, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return err;
    }

    int remaining = req->content_len;
    ESP_LOGI(TAG_OTA, "OTA upload: content length = %d bytes", remaining);

    uint8_t buf[OTA_RECV_BUF_SIZE];

    while (remaining > 0) {
        int to_read = remaining > OTA_RECV_BUF_SIZE ? OTA_RECV_BUF_SIZE : remaining;
        int received = httpd_req_recv(req, (char *)buf, to_read);

        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                ESP_LOGW(TAG_OTA, "Socket timeout, retrying...");
                continue; // No decrement of 'remaining', nochmal versuchen
            }
            ESP_LOGE(TAG_OTA, "httpd_req_recv failed (%d)", received);
            esp_ota_end(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
            return ESP_FAIL;
        }

        err = esp_ota_write(ota_handle, buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_OTA, "esp_ota_write failed (%s)", esp_err_to_name(err));
            esp_ota_end(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return err;
        }

        remaining -= received;
    }

    ESP_LOGI(TAG_OTA, "OTA upload finished, calling esp_ota_end");

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_OTA, "esp_ota_end failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_OTA, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return err;
    }

    ESP_LOGI(TAG_OTA, "OTA successful, will reboot");

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK, rebooting");

    // Kurz warten, damit Antwort rausgeht
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

esp_err_t ota_http_register_handlers(httpd_handle_t server)
{
    static const httpd_uri_t ota_update_uri = {
        .uri       = "/update",
        .method    = HTTP_POST,
        .handler   = ota_post_handler,
        .user_ctx  = NULL,
    };

    esp_err_t err = httpd_register_uri_handler(server, &ota_update_uri);
    if (err == ESP_OK) {
        ESP_LOGI(TAG_OTA, "Registered OTA HTTP handler at /update");
    } else {
        ESP_LOGE(TAG_OTA, "Failed to register OTA handler (%s)", esp_err_to_name(err));
    }
    return err;
}
