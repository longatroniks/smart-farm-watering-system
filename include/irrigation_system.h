// irrigation_system.h
#ifndef IRRIGATION_SYSTEM_H
#define IRRIGATION_SYSTEM_H

#include "esp_err.h"

typedef struct {
    int water_tank_level;
    int soil_moisture_percentage;
    float temperature;
    float humidity;
    int light_level;
    const char *pump_state;
} system_status_t;

esp_err_t irrigation_system_init(void);
system_status_t irrigation_system_get_status(void);

#endif // IRRIGATION_SYSTEM_H