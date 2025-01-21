
// telemetry_client.c
#include "telemetry_client.h"
#include "mqtt_driver.h"
#include "esp_log.h"
#include "cJSON.h"

#define NODE_DEVICE_ID "e380e2b0-d433-11ef-bab7-d10af52420c3"
#define NODE_ACCESS_TOKEN "8lpRi4s20USJgwDYKX0A"

static const char *TAG = "TELEMETRY_CLIENT";

static void telemetry_mqtt_callback(esp_mqtt_event_handle_t event)
{
    // Handle specific telemetry events if needed
    // For now, we rely on the default MQTT driver logging
}

esp_err_t telemetry_client_init(void)
{
    ESP_LOGI(TAG, "Initializing telemetry client");
    
    mqtt_driver_config_t config = {
        .device_id = NODE_DEVICE_ID,
        .access_token = NODE_ACCESS_TOKEN,
        .event_callback = telemetry_mqtt_callback
    };

    esp_err_t err = mqtt_driver_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT driver: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Telemetry client initialized successfully");
    return ESP_OK;
}

bool telemetry_client_publish(const char *data)
{
    if (!mqtt_driver_is_connected()) {
        ESP_LOGE(TAG, "Client not initialized");
        return false;
    }

    // Validate JSON format
    cJSON *root = cJSON_Parse(data);
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse input JSON");
        return false;
    }

    // Convert to string
    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    if (!json_string) {
        ESP_LOGE(TAG, "Failed to create JSON string");
        return false;
    }

    // Publish to ThingsBoard telemetry topic
    int msg_id = esp_mqtt_client_publish(mqtt_driver_get_client(),
                                       "v1/devices/me/telemetry",
                                       json_string,
                                       strlen(json_string),
                                       1,  // QoS 1
                                       0); // No retain
    free(json_string);
    
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish telemetry");
        return false;
    }

    ESP_LOGI(TAG, "Successfully published telemetry (msg_id=%d)", msg_id);
    return true;
}

bool telemetry_client_is_connected(void)
{
    return mqtt_driver_is_connected();
}

void telemetry_client_deinit(void)
{
    mqtt_driver_deinit();
    ESP_LOGI(TAG, "Telemetry client deinitialized");
}