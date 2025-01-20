#ifndef WEATHER_CLIENT_H
#define WEATHER_CLIENT_H

#include <stdbool.h>

// Weather data structure
typedef struct
{
    float pressure;
    float wind_speed;
    float rain_probability;
} weather_data_t;

// Initialize the MQTT client and WiFi
void weather_client_init(const char *wifi_ssid, const char *wifi_password);

// Get the latest weather data
weather_data_t weather_client_get_data(void);

// Check if weather client is connected
bool weather_client_is_connected(void);

#endif // WEATHER_CLIENT_H