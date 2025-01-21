
// weather_client.c
#include "weather_client.h"
#include "mqtt_driver.h"
#include "esp_log.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define WEATHER_DEVICE_ID "a9501f70-cd21-11ef-bab7-d10af52420c3"
#define WEATHER_ACCESS_TOKEN "igxnbStVswAQD2dwgsAk"

static const char *TAG = "WEATHER_CLIENT";
static TaskHandle_t mqtt_task_handle = NULL;

// Weather data storage
static weather_data_t current_weather = {
    .pressure = 0.0,
    .wind_speed = 0.0,
    .rain_probability = 0.0
};

// Parse weather data from ThingsBoard response
static void parse_weather_data(const char *data)
{
    cJSON *root = cJSON_Parse(data);
    if (root) {
        cJSON *pressure = cJSON_GetObjectItem(root, "pressure");
        if (pressure && cJSON_IsNumber(pressure)) {
            current_weather.pressure = pressure->valuedouble;
        }

        cJSON *wind_speed = cJSON_GetObjectItem(root, "wind_speed");
        if (wind_speed && cJSON_IsNumber(wind_speed)) {
            current_weather.wind_speed = wind_speed->valuedouble;
        }

        cJSON *rain_prob = cJSON_GetObjectItem(root, "rain_probability");
        if (rain_prob && cJSON_IsNumber(rain_prob)) {
            current_weather.rain_probability = rain_prob->valuedouble;
        }

        ESP_LOGI(TAG, "Weather update - P:%.1f W:%.1f R:%.1f%%",
                 current_weather.pressure, current_weather.wind_speed,
                 current_weather.rain_probability);

        cJSON_Delete(root);
    }
}

static void weather_mqtt_callback(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            // Subscribe to weather updates
            esp_mqtt_client_subscribe(mqtt_driver_get_client(), 
                                    "v1/devices/me/attributes", 0);
            break;

        case MQTT_EVENT_DATA:
            if (event->data_len > 0) {
                char *data = malloc(event->data_len + 1);
                if (data) {
                    memcpy(data, event->data, event->data_len);
                    data[event->data_len] = 0;
                    parse_weather_data(data);
                    free(data);
                }
            }
            break;
    }
}

static void weather_task(void *pvParameters)
{
    while (1) {
        if (mqtt_driver_is_connected()) {
            // Request latest weather data
            esp_mqtt_client_publish(mqtt_driver_get_client(),
                                  "v1/devices/me/telemetry/request",
                                  "", 0, 1, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // Update every 10 seconds
    }
}

esp_err_t weather_client_init(void)
{
    ESP_LOGI(TAG, "Initializing weather client");

    // Configure and initialize MQTT driver
    mqtt_driver_config_t config = {
        .device_id = WEATHER_DEVICE_ID,
        .access_token = WEATHER_ACCESS_TOKEN,
        .event_callback = weather_mqtt_callback
    };

    esp_err_t err = mqtt_driver_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MQTT driver: %s", esp_err_to_name(err));
        return err;
    }

    // Create weather update task
    BaseType_t xReturned = xTaskCreate(
        weather_task,
        "weather_task",
        8192,
        NULL,
        5,
        &mqtt_task_handle
    );

    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create weather task");
        mqtt_driver_deinit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Weather client initialized successfully");
    return ESP_OK;
}

weather_data_t weather_client_get_data(void)
{
    return current_weather;
}

bool weather_client_is_connected(void)
{
    return mqtt_driver_is_connected();
}

void weather_client_deinit(void)
{
    if (mqtt_task_handle != NULL) {
        vTaskDelete(mqtt_task_handle);
        mqtt_task_handle = NULL;
    }

    mqtt_driver_deinit();
    ESP_LOGI(TAG, "Weather client deinitialized");
}