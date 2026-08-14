#include "server.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "cJSON.h"
#include <string.h>
#include <esp_log.h>
#include "captdns.h"
#include "gyro_task.h"
#include "pid.h"
#include "types.h"
#include "pid.html.h"
#include "output.html.h"

static httpd_handle_t g_server = NULL;
static dns_server_handle_t dns_server_handle = NULL;

#define DNS_PORT 53
#define WIFI_SSID "FLIGHT_CON"

extern PID_Config_t* get_pid_roll(void);
extern PID_Config_t* get_pid_pitch(void);
extern PID_Config_t* get_pid_yaw(void);
extern int g_master_gain_channel;
extern int g_ouput_mapping[NUM_SERVOS];
extern uint32_t g_failsafe_us[NUM_SERVOS];
extern bool g_invert_channel[NUM_SERVOS];

static void dhcp_set_captiveportal_url(void) {
    // get the IP of the access point to redirect to
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    char ip_addr[16];
    inet_ntoa_r(ip_info.ip.addr, ip_addr, 16);

    // turn the IP into a URI
    char* captiveportal_uri = (char*) malloc(32 * sizeof(char));
    assert(captiveportal_uri && "Failed to allocate captiveportal_uri");
    strcpy(captiveportal_uri, "http://");
    strcat(captiveportal_uri, ip_addr);

    // get a handle to configure DHCP with
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    // set the DHCP option 114
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(netif));
    ESP_ERROR_CHECK(esp_netif_dhcps_option(netif, ESP_NETIF_OP_SET, ESP_NETIF_CAPTIVEPORTAL_URI, captiveportal_uri, strlen(captiveportal_uri)));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_start(netif));
}

// Handler de réception du POST /api/config
esp_err_t config_post_handler(httpd_req_t *req) {
    char buf[512];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Payload trop grand");
        return ESP_FAIL;
    }

    if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
        return ESP_FAIL;
    }
    buf[ret] = '\0'; // Fin de chaîne JSON

    // Parsing du JSON avec cJSON
    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON Invalide");
        return ESP_FAIL;
    }

    PID_Config_t* g_pidroll_config  = get_pid_roll();
    PID_Config_t* g_pidpitch_config = get_pid_pitch();
    PID_Config_t* g_pidyaw_config   = get_pid_yaw();
    
    // // Roll
    cJSON *item = cJSON_GetObjectItem(json, "roll_kp");
    if (item) g_pidroll_config->Kp = item->valuedouble;
    item = cJSON_GetObjectItem(json, "roll_kd");
    if (item) g_pidroll_config->Kd = item->valuedouble / 1000.0;
    item = cJSON_GetObjectItem(json, "roll_rate");
    if (item) g_pidroll_config->maxRateDegs = item->valuedouble;
    item = cJSON_GetObjectItem(json, "roll_invert");
    if (cJSON_IsBool(item)) {
        g_pidroll_config->invert = cJSON_IsTrue(item); // Renvoie 1 (true) ou 0 (false)
    }

    // Pitch
    item = cJSON_GetObjectItem(json, "pitch_kp");
    if (item) g_pidpitch_config->Kp = item->valuedouble;
    item = cJSON_GetObjectItem(json, "pitch_kd");
    if (item) g_pidpitch_config->Kd = item->valuedouble / 1000.0;
    item = cJSON_GetObjectItem(json, "pitch_rate");
    if (item) g_pidpitch_config->maxRateDegs = item->valuedouble;
    item = cJSON_GetObjectItem(json, "pitch_invert");
    if (cJSON_IsBool(item)) {
        g_pidpitch_config->invert = cJSON_IsTrue(item); // Renvoie 1 (true) ou 0 (false)
    }

    // Yaw
    item = cJSON_GetObjectItem(json, "yaw_kp");
    if (item) g_pidyaw_config->Kp = item->valuedouble;
    item = cJSON_GetObjectItem(json, "yaw_kd");
    if (item) g_pidyaw_config->Kd = item->valuedouble / 1000.0;
    item = cJSON_GetObjectItem(json, "yaw_rate");
    if (item) g_pidyaw_config->maxRateDegs = item->valuedouble;
    item = cJSON_GetObjectItem(json, "yaw_invert");
    if (cJSON_IsBool(item)) {
        g_pidyaw_config->invert = cJSON_IsTrue(item); // Renvoie 1 (true) ou 0 (false)
    }

    item = cJSON_GetObjectItem(json, "master_gain");
    if (item) g_master_gain_channel = item->valueint;

    cJSON_Delete(json);

    // Optionnel : Sauvegarder dans la NVS Flash ici
    // save_config_to_nvs();

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

esp_err_t config_postpwm_handler(httpd_req_t *req) {
    char buf[512];
    int ret, remaining = req->content_len;

    if (remaining >= sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Payload trop grand");
        return ESP_FAIL;
    }

    if ((ret = httpd_req_recv(req, buf, remaining)) <= 0) {
        return ESP_FAIL;
    }
    buf[ret] = '\0'; // Fin de chaîne JSON

    // Parsing du JSON avec cJSON
    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON Invalide");
        return ESP_FAIL;
    }

    cJSON *item = cJSON_GetObjectItem(json, "channel0");
    if (item) g_ouput_mapping[0] = item->valueint;
    item = cJSON_GetObjectItem(json, "channel1");
    if (item) g_ouput_mapping[1] = item->valueint;
    item = cJSON_GetObjectItem(json, "channel2");
    if (item) g_ouput_mapping[2] = item->valueint;
    item = cJSON_GetObjectItem(json, "channel3");
    if (item) g_ouput_mapping[3] = item->valueint;
    item = cJSON_GetObjectItem(json, "channel4");
    if (item) g_ouput_mapping[4] = item->valueint;
    item = cJSON_GetObjectItem(json, "channel5");
    if (item) g_ouput_mapping[5] = item->valueint;

    item = cJSON_GetObjectItem(json, "invert0");
    if (item) g_invert_channel[0] = item->valueint;
    item = cJSON_GetObjectItem(json, "invert1");
    if (item) g_invert_channel[1] = item->valueint;
    item = cJSON_GetObjectItem(json, "invert2");
    if (item) g_invert_channel[2] = item->valueint;
    item = cJSON_GetObjectItem(json, "invert3");
    if (item) g_invert_channel[3] = item->valueint;
    item = cJSON_GetObjectItem(json, "invert4");
    if (item) g_invert_channel[4] = item->valueint;
    item = cJSON_GetObjectItem(json, "invert5");
    if (item) g_invert_channel[5] = item->valueint;

    item = cJSON_GetObjectItem(json, "failsafe0");
    if (item) g_failsafe_us[0] = item->valueint;
    item = cJSON_GetObjectItem(json, "failsafe1");
    if (item) g_failsafe_us[1] = item->valueint;
    item = cJSON_GetObjectItem(json, "failsafe2");
    if (item) g_failsafe_us[2] = item->valueint;
    item = cJSON_GetObjectItem(json, "failsafe3");
    if (item) g_failsafe_us[3] = item->valueint;
    item = cJSON_GetObjectItem(json, "failsafe4");
    if (item) g_failsafe_us[4] = item->valueint;
    item = cJSON_GetObjectItem(json, "failsafe5");
    if (item) g_failsafe_us[5] = item->valueint;

    cJSON_Delete(json);

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

esp_err_t config_get_handler(httpd_req_t *req) {
    // 1. Création de l'objet JSON racine
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Échec allocation cJSON");
        return ESP_FAIL;
    }

    PID_Config_t* g_pidroll_config  = get_pid_roll();
    PID_Config_t* g_pidpitch_config = get_pid_pitch();
    PID_Config_t* g_pidyaw_config   = get_pid_yaw();

    // Axis: Roll
    cJSON_AddNumberToObject(json, "roll_kp",   g_pidroll_config->Kp);
    cJSON_AddNumberToObject(json, "roll_kd",   g_pidroll_config->Kd * 1000.0f);
    cJSON_AddNumberToObject(json, "roll_rate", g_pidroll_config->maxRateDegs);
    cJSON_AddBoolToObject(json, "roll_invert", g_pidroll_config->invert);

    // Axis: Pitch
    cJSON_AddNumberToObject(json, "pitch_kp",   g_pidpitch_config->Kp);
    cJSON_AddNumberToObject(json, "pitch_kd",   g_pidpitch_config->Kd * 1000.0f);
    cJSON_AddNumberToObject(json, "pitch_rate", g_pidpitch_config->maxRateDegs);
    cJSON_AddBoolToObject(json, "pitch_invert", g_pidpitch_config->invert);

    // Axis: Yaw
    cJSON_AddNumberToObject(json, "yaw_kp",   g_pidyaw_config->Kp);
    cJSON_AddNumberToObject(json, "yaw_kd",   g_pidyaw_config->Kd * 1000.0f);
    cJSON_AddNumberToObject(json, "yaw_rate", g_pidyaw_config->maxRateDegs);
    cJSON_AddBoolToObject(json, "yaw_invert", g_pidyaw_config->invert);

    cJSON_AddNumberToObject(json, "master_gain", g_master_gain_channel);

    cJSON_AddNumberToObject(json, "channel0", g_ouput_mapping[0]);
    cJSON_AddNumberToObject(json, "channel1", g_ouput_mapping[1]);
    cJSON_AddNumberToObject(json, "channel2", g_ouput_mapping[2]);
    cJSON_AddNumberToObject(json, "channel3", g_ouput_mapping[3]);
    cJSON_AddNumberToObject(json, "channel4", g_ouput_mapping[4]);
    cJSON_AddNumberToObject(json, "channel5", g_ouput_mapping[5]);

    cJSON_AddNumberToObject(json, "failsafe0", g_failsafe_us[0]);
    cJSON_AddNumberToObject(json, "failsafe1", g_failsafe_us[1]);
    cJSON_AddNumberToObject(json, "failsafe2", g_failsafe_us[2]);
    cJSON_AddNumberToObject(json, "failsafe3", g_failsafe_us[3]);
    cJSON_AddNumberToObject(json, "failsafe4", g_failsafe_us[4]);
    cJSON_AddNumberToObject(json, "failsafe5", g_failsafe_us[5]);

    cJSON_AddBoolToObject(json, "invert_channel0", g_invert_channel[0]);
    cJSON_AddBoolToObject(json, "invert_channel1", g_invert_channel[1]);
    cJSON_AddBoolToObject(json, "invert_channel2", g_invert_channel[2]);
    cJSON_AddBoolToObject(json, "invert_channel3", g_invert_channel[3]);
    cJSON_AddBoolToObject(json, "invert_channel4", g_invert_channel[4]);
    cJSON_AddBoolToObject(json, "invert_channel5", g_invert_channel[5]);

    // 3. Conversion de l'objet JSON en chaîne de caractères (non formatée = plus compacte)
    char *json_str = cJSON_PrintUnformatted(json);
    if (json_str == NULL) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Échec génération JSON");
        return ESP_FAIL;
    }

    // 4. Configuration des en-têtes HTTP et envoi
    httpd_resp_set_type(req, "application/json");
    // Optionnel: Évite que le navigateur garde en cache les anciens réglages
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    
    esp_err_t res = httpd_resp_sendstr(req, json_str);

    // 5. Libération obligatoire de la mémoire allouée par cJSON
    cJSON_Delete(json);
    free(json_str);

    return res;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_send(req, (const char *)pid_html_gz, pid_html_gz_len);

    return ESP_OK;
}

static esp_err_t output_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_send(req, (const char *)output_html_gz, output_html_gz_len);

    return ESP_OK;
}

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    // Set status
    httpd_resp_set_status(req, "303 See Other");
    // Redirect to the "/" root directory
    httpd_resp_set_hdr(req, "Location", "/");
    // iOS requires content in the response to detect a captive portal, simply redirecting is not sufficient.
    httpd_resp_send(req, "Redirect to the captive portal", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static void wifi_init_softap(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = 1,
            .authmode = WIFI_AUTH_OPEN,
            .max_connection = 4,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

int server_is_running(void)
{
    return g_server != NULL;
}

void stop_webserver(void)
{
    if (g_server) {
        httpd_stop(g_server);
        g_server = NULL;
        esp_wifi_stop();
        esp_wifi_deinit();
    }
    
    stop_dns_server(dns_server_handle);
    dns_server_handle = NULL;
}

void start_webserver(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_softap();

    dhcp_set_captiveportal_url();

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 13;
    config.lru_purge_enable = true;

    if (httpd_start(&g_server, &config) == ESP_OK) {
        // Handler pour l'adresse racine
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_get_handler,
            .user_ctx = NULL
        };

        httpd_uri_t output = {
            .uri = "/output",
            .method = HTTP_GET,
            .handler = output_get_handler,
            .user_ctx = NULL
        };
        
        static const httpd_uri_t uri_config_post = {
            .uri       = "/api/config",
            .method    = HTTP_POST,
            .handler   = config_post_handler, // Votre fonction
            .user_ctx  = NULL
        };

            static const httpd_uri_t uri_configpwm_post = {
            .uri       = "/api/configpwm",
            .method    = HTTP_POST,
            .handler   = config_postpwm_handler, // Votre fonction
            .user_ctx  = NULL
        };

        static const httpd_uri_t uri_config_get = {
            .uri       = "/api/config",
            .method    = HTTP_GET,
            .handler   = config_get_handler, // À implémenter pour renvoyer le JSON actuel
            .user_ctx  = NULL
        };
        
        httpd_register_err_handler(g_server, HTTPD_404_NOT_FOUND, http_404_error_handler);
        httpd_register_uri_handler(g_server, &root);
        httpd_register_uri_handler(g_server, &output);
        httpd_register_uri_handler(g_server, &uri_config_get);
        httpd_register_uri_handler(g_server, &uri_config_post);
        httpd_register_uri_handler(g_server, &uri_configpwm_post);
    }

    dns_server_config_t dns_config = DNS_SERVER_CONFIG_SINGLE("*" /* all A queries */, "WIFI_AP_DEF" /* softAP netif ID */);
    dns_server_handle = start_dns_server(&dns_config);
}