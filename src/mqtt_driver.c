#include "mqtt_driver.h"
#include "esp_log.h"

#define THINGSBOARD_HOST "srv-iot.diatel.upm.es"
#define THINGSBOARD_PORT 1883

static const char *TAG = "MQTT_DRIVER";
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool client_initialized = false;
static mqtt_event_callback_t user_event_callback = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    
    // Log standard events
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected to ThingsBoard MQTT");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Disconnected from ThingsBoard MQTT");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT Error occurred");
            break;
    }

    // Forward event to user callback if registered
    if (user_event_callback) {
        user_event_callback(event);
    }
}

esp_err_t mqtt_driver_init(const mqtt_driver_config_t *config)
{
    if (client_initialized) {
        ESP_LOGW(TAG, "MQTT driver already initialized");
        return ESP_OK;
    }

    if (!config || !config->device_id || !config->access_token) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }

    char uri[128];
    snprintf(uri, sizeof(uri), "mqtt://%s:%d", THINGSBOARD_HOST, THINGSBOARD_PORT);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
        .broker.address.port = THINGSBOARD_PORT,
        .credentials.username = config->access_token,
        .credentials.client_id = config->device_id
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    // Store user callback
    user_event_callback = config->event_callback;

    // Register MQTT event handler
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    // Start MQTT client
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return err;
    }

    client_initialized = true;
    ESP_LOGI(TAG, "MQTT driver initialized successfully with URI: %s", uri);
    return ESP_OK;
}

esp_mqtt_client_handle_t mqtt_driver_get_client(void)
{
    return mqtt_client;
}

bool mqtt_driver_is_connected(void)
{
    return client_initialized && mqtt_client != NULL;
}

void mqtt_driver_deinit(void)
{
    if (!client_initialized) {
        return;
    }

    if (mqtt_client != NULL) {
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
    }

    user_event_callback = NULL;
    client_initialized = false;
    ESP_LOGI(TAG, "MQTT driver deinitialized");
}