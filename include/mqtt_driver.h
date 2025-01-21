// mqtt_driver.h
#ifndef MQTT_DRIVER_H
#define MQTT_DRIVER_H

#include "esp_err.h"
#include "mqtt_client.h"
#include "esp_event.h"

typedef void (*mqtt_event_callback_t)(esp_mqtt_event_handle_t event);

typedef struct {
    const char *device_id;
    const char *access_token;
    mqtt_event_callback_t event_callback;
} mqtt_driver_config_t;

/**
 * @brief Initialize the MQTT driver
 * @param config Driver configuration including device credentials
 * @return ESP_OK on success
 */
esp_err_t mqtt_driver_init(const mqtt_driver_config_t *config);

/**
 * @brief Get the MQTT client handle
 * @return MQTT client handle or NULL if not initialized
 */
esp_mqtt_client_handle_t mqtt_driver_get_client(void);

/**
 * @brief Check if MQTT client is initialized
 * @return true if initialized
 */
bool mqtt_driver_is_connected(void);

/**
 * @brief Deinitialize the MQTT driver
 */
void mqtt_driver_deinit(void);

#endif // MQTT_DRIVER_H
