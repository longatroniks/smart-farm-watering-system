// si7021.c
#include "si7021.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SI7021";
static const int I2C_MASTER_TIMEOUT_MS = 1000;

// Private function to read raw data from the sensor
static esp_err_t si7021_read_raw(si7021_dev_t* dev, uint8_t cmd, uint16_t* raw_data)
{
    uint8_t data[2];
    esp_err_t ret;

    // Send command to the sensor
    ret = i2c_master_transmit(dev->device_handle, &cmd, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error sending command: %d", ret);
        return ret;
    }

    // Add delay for sensor to process command
    vTaskDelay(pdMS_TO_TICKS(50));

    // Read response from sensor
    ret = i2c_master_receive(dev->device_handle, data, 2, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading response: %d", ret);
        return ret;
    }

    *raw_data = (data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t si7021_init(si7021_dev_t* dev, i2c_master_bus_handle_t bus_handle)
{
    if (dev == NULL || bus_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    dev->bus_handle = bus_handle;
    dev->last_temperature = 0.0f;
    dev->last_humidity = 0.0f;

    // Configure the Si7021 device settings
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SI7021_ADDR,
        .scl_speed_hz = 100000,  // 100 kHz
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_config, &dev->device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus");
        return ret;
    }

    return ESP_OK;
}

esp_err_t si7021_read_temperature(si7021_dev_t* dev, float* temperature)
{
    if (dev == NULL || temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_temp;
    esp_err_t ret = si7021_read_raw(dev, SI7021_CMD_READ_TEMP, &raw_temp);

    if (ret == ESP_OK) {
        // Convert raw temperature data to Celsius
        *temperature = ((175.72f * raw_temp) / 65536.0f) - 46.85f;
        dev->last_temperature = *temperature;
        ESP_LOGD(TAG, "Temperature: %.2f°C", *temperature);
    }

    return ret;
}

esp_err_t si7021_read_humidity(si7021_dev_t* dev, float* humidity)
{
    if (dev == NULL || humidity == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_hum;
    esp_err_t ret = si7021_read_raw(dev, SI7021_CMD_READ_HUM, &raw_hum);

    if (ret == ESP_OK) {
        // Convert raw humidity data to percentage
        *humidity = ((125.0f * raw_hum) / 65536.0f) - 6.0f;
        
        // Constrain values
        if (*humidity > 100.0f) *humidity = 100.0f;
        if (*humidity < 0.0f) *humidity = 0.0f;
        
        dev->last_humidity = *humidity;
        ESP_LOGD(TAG, "Humidity: %.2f%%", *humidity);
    }

    return ret;
}

esp_err_t si7021_probe(si7021_dev_t* dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_probe(dev->bus_handle, SI7021_ADDR, I2C_MASTER_TIMEOUT_MS);
}