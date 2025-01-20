#include "weather_client.h"
#include "esp_http_client.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_tls.h"

#define THINGSBOARD_HOST "srv-iot.diatel.upm.es"
#define THINGSBOARD_PORT 443 // HTTPS port
#define WEATHER_DEVICE_ID "a9501f70-cd21-11ef-bab7-d10af52420c3"
#define WEATHER_ACCESS_TOKEN "igxnbStVswAQD2dwgsAk"
#define WEATHER_ENDPOINT "/api/v1/%s/telemetry"

static const char *TAG = "WEATHER_CLIENT";
static esp_http_client_handle_t http_client = NULL;
static bool client_initialized = false;

// Weather data storage
static weather_data_t current_weather = {
    .pressure = 0.0,
    .wind_speed = 0.0,
    .rain_probability = 0.0};

// Parse weather data from ThingsBoard response
static void parse_weather_data(const char *data)
{
    cJSON *root = cJSON_Parse(data);
    if (root)
    {
        // Get the latest values array - ThingsBoard stores historical data
        cJSON *values = cJSON_GetObjectItem(root, "pressure");
        if (values && cJSON_IsArray(values) && cJSON_GetArraySize(values) > 0)
        {
            cJSON *latest = cJSON_GetArrayItem(values, 0);
            if (latest)
            {
                cJSON *value = cJSON_GetObjectItem(latest, "value");
                if (value)
                {
                    current_weather.pressure = value->valuedouble;
                }
            }
        }

        // Same for wind speed
        values = cJSON_GetObjectItem(root, "wind_speed");
        if (values && cJSON_IsArray(values) && cJSON_GetArraySize(values) > 0)
        {
            cJSON *latest = cJSON_GetArrayItem(values, 0);
            if (latest)
            {
                cJSON *value = cJSON_GetObjectItem(latest, "value");
                if (value)
                {
                    current_weather.wind_speed = value->valuedouble;
                }
            }
        }

        // And rain probability
        values = cJSON_GetObjectItem(root, "rain_probability");
        if (values && cJSON_IsArray(values) && cJSON_GetArraySize(values) > 0)
        {
            cJSON *latest = cJSON_GetArrayItem(values, 0);
            if (latest)
            {
                cJSON *value = cJSON_GetObjectItem(latest, "value");
                if (value)
                {
                    current_weather.rain_probability = value->valuedouble;
                }
            }
        }

        ESP_LOGI(TAG, "Weather update - P:%.1f W:%.1f R:%.1f%%",
                 current_weather.pressure, current_weather.wind_speed,
                 current_weather.rain_probability);

        cJSON_Delete(root);
    }
}

static void fetch_weather_data(void *pvParameters)
{
    char url[256];
    snprintf(url, sizeof(url), "https://%s/api/v1/%s/telemetry",
             THINGSBOARD_HOST, WEATHER_ACCESS_TOKEN);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET, 
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .timeout_ms = 5000,
        .crt_bundle_attach = esp_crt_bundle_attach,  // Use the certificate bundle
        .use_global_ca_store = false,
        .skip_cert_common_name_check = true          // Skip hostname verification temporarily
    };

    http_client = esp_http_client_init(&config);
    if (http_client == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return;
    }

    ESP_LOGI(TAG, "Weather fetch task started with URL: %s", url);

    while (1)
    {
        esp_err_t err = esp_http_client_perform(http_client);

        if (err == ESP_OK)
        {
            int status_code = esp_http_client_get_status_code(http_client);
            if (status_code == 200)
            {
                int content_length = esp_http_client_get_content_length(http_client);
                if (content_length > 0)
                {
                    char *response = malloc(content_length + 1);
                    if (response)
                    {
                        int read_len = esp_http_client_read(http_client, response, content_length);
                        if (read_len == content_length)
                        {
                            response[content_length] = 0;
                            parse_weather_data(response);
                        }
                        else
                        {
                            ESP_LOGE(TAG, "Failed to read complete response");
                        }
                        free(response);
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Empty response received");
                }
            }
            else
            {
                ESP_LOGE(TAG, "HTTP GET failed with status code: %d", status_code);
            }
        }
        else
        {
            ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(10000)); // Update every 10 seconds
    }
}

// WiFi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "Connecting to WiFi...");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG, "Reconnecting to WiFi...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void wifi_init_sta(const char *ssid, const char *password)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };

    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialization complete");
}

void weather_client_init(const char *wifi_ssid, const char *wifi_password)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta(wifi_ssid, wifi_password);

    xTaskCreate(fetch_weather_data, "weather_fetch", 8192, NULL, 5, NULL);
    client_initialized = true;
}

weather_data_t weather_client_get_data(void)
{
    return current_weather;
}

bool weather_client_is_connected(void)
{
    return client_initialized;
}