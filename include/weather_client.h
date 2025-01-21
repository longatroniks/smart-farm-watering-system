// weather_client.h
#ifndef WEATHER_CLIENT_H
#define WEATHER_CLIENT_H

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    float pressure;
    float wind_speed;
    float rain_probability;
} weather_data_t;

/**
 * @brief Initialize the weather client
 * @return ESP_OK on success
 */
esp_err_t weather_client_init(void);

/**
 * @brief Get the current weather data
 * @return Current weather data structure
 */
weather_data_t weather_client_get_data(void);

/**
 * @brief Check if weather client is connected
 * @return true if connected
 */
bool weather_client_is_connected(void);

/**
 * @brief Deinitialize the weather client
 */
void weather_client_deinit(void);

#endif // WEATHER_CLIENT_H
