#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define I2C_MASTER_SCL_IO          22    
#define I2C_MASTER_SDA_IO          21    
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS      1000   

#define PHOTO_TRANSISTOR_PIN       ADC_CHANNEL_6  // GPIO34
#define SOIL_MOISTURE_PIN          ADC_CHANNEL_7  // GPIO35

#define SI7021_ADDR                0x40
#define SI7021_CMD_READ_TEMP       0xE3
#define SI7021_CMD_READ_HUM        0xE5

// Calibration values for Capacitive Soil Moisture Sensor v1.2
#define MOISTURE_AIR_VALUE         2450    // Value in air (dry)
#define MOISTURE_WATER_VALUE       1150    // Value in water (wet)

static const char *TAG = "SENSORS";
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t si7021_handle;
static adc_oneshot_unit_handle_t adc1_handle;

// Function for initializing ADC
static void adc_init(void)
{
    // ADC1 Init
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC1 Config - Using ADC_ATTEN_DB_11 for full range measurement
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,  // Changed to 11dB for better range
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PHOTO_TRANSISTOR_PIN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SOIL_MOISTURE_PIN, &config));
}

// Function for initializing I2C bus
static void i2c_master_init_bus(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
}

// Function for initializing Si7021 device handle
static void si7021_init_handle(void)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SI7021_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &si7021_handle));
}

// Function to read raw data from Si7021 sensor
static esp_err_t read_si7021_raw(uint8_t cmd, uint16_t *raw_data)
{
    uint8_t data[2];
    esp_err_t ret;

    // Write command to sensor
    ret = i2c_master_transmit(si7021_handle, &cmd, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error sending command: %d", ret);
        return ret;
    }

    // Add small delay for measurement
    vTaskDelay(pdMS_TO_TICKS(50));

    // Read response
    ret = i2c_master_receive(si7021_handle, data, 2, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error reading response: %d", ret);
        return ret;
    }

    *raw_data = (data[0] << 8) | data[1];
    return ESP_OK;
}

void read_temperature(void)
{
    uint16_t raw_temp;
    esp_err_t ret = read_si7021_raw(SI7021_CMD_READ_TEMP, &raw_temp);
    
    if (ret == ESP_OK) {
        float temp_c = ((175.72f * raw_temp) / 65536.0f) - 46.85f;
        ESP_LOGI(TAG, "Temperature: %.2f°C", temp_c);
    } else {
        ESP_LOGE(TAG, "Failed to read temperature");
    }
}

void read_humidity(void)
{
    uint16_t raw_hum;
    esp_err_t ret = read_si7021_raw(SI7021_CMD_READ_HUM, &raw_hum);
    
    if (ret == ESP_OK) {
        float humidity = ((125.0f * raw_hum) / 65536.0f) - 6.0f;
        if (humidity > 100.0f) humidity = 100.0f;
        if (humidity < 0.0f) humidity = 0.0f;
        ESP_LOGI(TAG, "Humidity: %.2f%%", humidity);
    } else {
        ESP_LOGE(TAG, "Failed to read humidity");
    }
}

void read_photo_transistor(void) 
{
    int adc_raw;
    esp_err_t ret = adc_oneshot_read(adc1_handle, PHOTO_TRANSISTOR_PIN, &adc_raw);
    if (ret == ESP_OK) {
        // Calculate the percentage of light (0-4095 range)
        int light_percentage = (adc_raw * 100) / 4095;
        ESP_LOGI(TAG, "Light level: %d%% (ADC: %d)", light_percentage, adc_raw);
    } else {
        ESP_LOGE(TAG, "Error reading photo transistor");
    }
}

// Helper function to map values from one range to another
static int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void read_soil_moisture(void)
{
    int adc_raw;
    esp_err_t ret = adc_oneshot_read(adc1_handle, SOIL_MOISTURE_PIN, &adc_raw);
    if (ret == ESP_OK) {
        // Calculate moisture percentage (inverted because sensor gives higher values for drier soil)
        // Constrain ADC value first to avoid overflow
        if (adc_raw > MOISTURE_AIR_VALUE) adc_raw = MOISTURE_AIR_VALUE;
        if (adc_raw < MOISTURE_WATER_VALUE) adc_raw = MOISTURE_WATER_VALUE;
        
        int moisture_percentage = map(adc_raw, MOISTURE_AIR_VALUE, MOISTURE_WATER_VALUE, 0, 100);

        // Determine moisture level with updated thresholds
        const char* moisture_level;
        if (moisture_percentage < 15) {
            moisture_level = "Very Dry";
        } else if (moisture_percentage < 30) {
            moisture_level = "Dry";
        } else if (moisture_percentage < 60) {
            moisture_level = "Moderate";
        } else if (moisture_percentage < 85) {
            moisture_level = "Wet";
        } else {
            moisture_level = "Very Wet";
        }

        ESP_LOGI(TAG, "Soil Moisture: %d%% (%s) [ADC: %d]", 
                 moisture_percentage, moisture_level, adc_raw);
    } else {
        ESP_LOGE(TAG, "Error reading soil moisture sensor");
    }
}

void sensor_task(void *pvParameters)
{
    // Add a short delay before starting readings to allow sensors to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    while (1) {
        read_temperature();
        read_humidity();
        read_photo_transistor();
        read_soil_moisture();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing sensors...");
    
    // Initialize I2C bus
    i2c_master_init_bus();
    
    // Initialize Si7021 device handle
    si7021_init_handle();
    
    // Initialize ADC
    adc_init();
    
    // Verify Si7021 is responding
    esp_err_t err = i2c_master_probe(bus_handle, SI7021_ADDR, I2C_MASTER_TIMEOUT_MS);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Si7021 found at address 0x%02X", SI7021_ADDR);
        // Create sensor reading task
        xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "Could not find Si7021 at address 0x%02X", SI7021_ADDR);
    }
}