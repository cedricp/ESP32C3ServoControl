#include "esp_http_server.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

static httpd_handle_t g_server = NULL;

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char *htmlheader =
        "<!DOCTYPE html>"
        "<html>"
        "<head><title>ESP32-C3</title></head>"
        "<meta http-equiv='refresh' content='1'>"
        "<body>";

    const char *html_footer =
        "<h1>Hello ESP32-C3</h1>"
        "<p>Serveur Web OK</p>"
        "</body>"
        "</html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, htmlheader, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, html_footer, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));

    strcpy((char*)wifi_config.ap.ssid, "FLIGHT_COM_AP");
    wifi_config.ap.password[0] = 0;

    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.ssid_len = 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void start_webserver(void)
{
    wifi_init_softap();

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {

        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL
        };

        httpd_register_uri_handler(server, &root);
    }
    g_server = server;
}