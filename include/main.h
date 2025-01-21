// main.h
#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>

// Task configurations
#define STACK_SIZE_SENSOR 4096
#define STACK_SIZE_PUMP 2048
#define STACK_SIZE_JSON 8192
#define TASK_PRIORITY 5

// GPIO configurations
#define PUMP_GPIO_PIN 25
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS 1000

// ADC configurations
#define PHOTO_TRANSISTOR_PIN ADC_CHANNEL_6
#define SOIL_MOISTURE_PIN ADC_CHANNEL_7
#define MOISTURE_AIR_VALUE 2450
#define MOISTURE_WATER_VALUE 1150

#endif // MAIN_H
