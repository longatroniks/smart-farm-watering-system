#ifndef TELEMETRY_CLIENT_H
#define TELEMETRY_CLIENT_H

#include <stdbool.h>

// Initialize telemetry client - must be called after WiFi is connected
void telemetry_client_init(void);

// Publish sensor data to ThingsBoard
bool telemetry_client_publish(const char *data);

// Check if connected to MQTT broker
bool telemetry_client_is_connected(void);

#endif /* TELEMETRY_CLIENT_H */