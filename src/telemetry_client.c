#include "telemetry_client.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"

#define THINGSBOARD_HOST "srv-iot.diatel.upm.es"
#define THINGSBOARD_PORT 443 // HTTPS port
#define NODE_DEVICE_ID "e380e2b0-d433-11ef-bab7-d10af52420c3"
#define NODE_ACCESS_TOKEN "8lpRi4s20USJgwDYKX0A"
#define TELEMETRY_ENDPOINT "/api/v1/%s/telemetry"

static const char *TAG = "TELEMETRY_CLIENT";
static esp_http_client_handle_t http_client = NULL;
static bool client_initialized = false;

void telemetry_client_init(void)
{
    if (client_initialized)
    {
        ESP_LOGW(TAG, "Client already initialized");
        return;
    }

    ESP_LOGI(TAG, "Initializing telemetry client");

    char url[256];
    snprintf(url, sizeof(url), "https://%s/api/v1/%s/telemetry",
             THINGSBOARD_HOST, NODE_ACCESS_TOKEN);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .timeout_ms = 5000,
        .crt_bundle_attach = esp_crt_bundle_attach,  // Use the built-in certificate bundle
        .use_global_ca_store = false,                // Don't use global CA store
        .skip_cert_common_name_check = false         // Verify the host name
    };

    http_client = esp_http_client_init(&config);
    if (http_client == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return;
    }

    esp_http_client_set_header(http_client, "Content-Type", "application/json");
    client_initialized = true;
    ESP_LOGI(TAG, "Telemetry client initialized successfully with URL: %s", url);
}

bool telemetry_client_publish(const char *data)
{
    if (!client_initialized || http_client == NULL)
    {
        ESP_LOGE(TAG, "Client not initialized");
        return false;
    }
    
    // Parse input JSON
    cJSON *root = cJSON_Parse(data);
    if (!root)
    {
        ESP_LOGE(TAG, "Failed to parse input JSON");
        return false;
    }

    // Convert to string
    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    if (!json_string)
    {
        ESP_LOGE(TAG, "Failed to create JSON string");
        return false;
    }

    // Set POST data
    esp_err_t err = esp_http_client_set_post_field(http_client, json_string, strlen(json_string));
    free(json_string);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set POST data: %s", esp_err_to_name(err));
        return false;
    }

    // Perform HTTP POST
    err = esp_http_client_perform(http_client);
    if (err == ESP_OK)
    {
        int status_code = esp_http_client_get_status_code(http_client);
        if (status_code == 200)
        {
            ESP_LOGI(TAG, "Successfully published telemetry");
            return true;
        }
        ESP_LOGE(TAG, "HTTP POST failed with status code: %d", status_code);
    }
    else
    {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    return false;
}

bool telemetry_client_is_connected(void)
{
    return client_initialized;
}