// telemetry_client.h
#ifndef TELEMETRY_CLIENT_H
#define TELEMETRY_CLIENT_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initialize the telemetry client
 * @return ESP_OK on success
 */
esp_err_t telemetry_client_init(void);

/**
 * @brief Publish telemetry data to ThingsBoard
 * @param data JSON-formatted string containing telemetry data
 * @return true if published successfully
 */
bool telemetry_client_publish(const char *data);

/**
 * @brief Check if telemetry client is connected
 * @return true if connected
 */
bool telemetry_client_is_connected(void);

/**
 * @brief Deinitialize the telemetry client
 */
void telemetry_client_deinit(void);

#endif // TELEMETRY_CLIENT_H
