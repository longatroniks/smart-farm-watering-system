// si7021.h
#ifndef SI7021_H
#define SI7021_H

#include "driver/i2c_master.h"
#include "esp_err.h"

// Si7021 I2C Address
#define SI7021_ADDR                0x40

// Si7021 Commands
#define SI7021_CMD_READ_TEMP       0xE3  // Command to read temperature
#define SI7021_CMD_READ_HUM        0xE5  // Command to read humidity

typedef struct {
    i2c_master_bus_handle_t bus_handle;      // Handle for the I2C bus
    i2c_master_dev_handle_t device_handle;   // Handle for the Si7021 device
    float last_temperature;                  // Last read temperature value
    float last_humidity;                     // Last read humidity value
} si7021_dev_t;

/**
 * @brief Initialize Si7021 sensor
 * @param dev Pointer to si7021_dev_t structure
 * @param bus_handle Handle to initialized I2C bus
 * @return ESP_OK on success
 */
esp_err_t si7021_init(si7021_dev_t* dev, i2c_master_bus_handle_t bus_handle);

/**
 * @brief Read temperature from Si7021
 * @param dev Pointer to initialized si7021_dev_t structure
 * @param temperature Pointer to store temperature value (in Celsius)
 * @return ESP_OK on success
 */
esp_err_t si7021_read_temperature(si7021_dev_t* dev, float* temperature);

/**
 * @brief Read humidity from Si7021
 * @param dev Pointer to initialized si7021_dev_t structure
 * @param humidity Pointer to store humidity value (in percentage)
 * @return ESP_OK on success
 */
esp_err_t si7021_read_humidity(si7021_dev_t* dev, float* humidity);

/**
 * @brief Check if Si7021 sensor is responding
 * @param dev Pointer to initialized si7021_dev_t structure
 * @return ESP_OK if sensor responds, error otherwise
 */
esp_err_t si7021_probe(si7021_dev_t* dev);

#endif // SI7021_H