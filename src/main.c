#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include <stdlib.h>  // For rand() and srand()
#include <time.h>    // For time() to seed random numbers
#include "cJSON.h"

// GPIO for the pump
#define PUMP_GPIO_PIN 25 // GPIO pin for controlling the pump

// I2C Configuration
#define I2C_MASTER_SCL_IO          22    // GPIO pin for I2C clock
#define I2C_MASTER_SDA_IO          21    // GPIO pin for I2C data
#define I2C_MASTER_NUM             I2C_NUM_0 // I2C port number
#define I2C_MASTER_TIMEOUT_MS      1000  // Timeout for I2C operations

// ADC Pins for Sensors
#define PHOTO_TRANSISTOR_PIN       ADC_CHANNEL_6  // GPIO34 for light sensor
#define SOIL_MOISTURE_PIN          ADC_CHANNEL_7  // GPIO35 for soil moisture sensor

// Si7021 Sensor Configuration
#define SI7021_ADDR                0x40  // I2C address of Si7021 sensor
#define SI7021_CMD_READ_TEMP       0xE3  // Command to read temperature
#define SI7021_CMD_READ_HUM        0xE5  // Command to read humidity

// Calibration values for Capacitive Soil Moisture Sensor v1.2
#define MOISTURE_AIR_VALUE         2450  // ADC value for dry soil
#define MOISTURE_WATER_VALUE       1150  // ADC value for saturated soil

// Logging Tags
static const char *TAG_SENSORS = "SENSORS";
static const char *TAG_PUMP = "PUMP";

// Global variables
static int water_tank_level = 0; // Global variable for water tank level (starts randomly between 60% and 40%)
static int soil_moisture_percentage = 0;  // Global variable to store the soil moisture percentage
static float temperature = 0.0;  // Temperature in Celsius
static float humidity = 0.0;     // Humidity in percentage
static int light_level = 0;      // Light level (e.g., ADC value or percentage)
static const char* pump_state = "OFF";  // Global pump state variable, Default state is OFF

// Handles for peripherals
static i2c_master_bus_handle_t bus_handle; // Handle for the I2C bus
static i2c_master_dev_handle_t si7021_handle; // Handle for the Si7021 device
static adc_oneshot_unit_handle_t adc1_handle; // Handle for ADC unit 1

// Function for initializing ADC (Analog-to-Digital Converter)
static void adc_init(void)
{
    // Configure ADC Unit 1
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,          // Specify ADC unit 1
        .ulp_mode = ADC_ULP_MODE_DISABLE, // Disable ULP mode (not used here)
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Configure ADC channels (light sensor and soil moisture sensor)
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,   // Set resolution to 12 bits
        .atten = ADC_ATTEN_DB_12,      // Set attenuation for full range measurement
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PHOTO_TRANSISTOR_PIN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SOIL_MOISTURE_PIN, &config));
}

// Function for initializing the I2C bus
static void i2c_master_init_bus(void)
{
    // Configure I2C bus settings
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,        // Specify I2C port
        .sda_io_num = I2C_MASTER_SDA_IO,  // GPIO pin for SDA
        .scl_io_num = I2C_MASTER_SCL_IO,  // GPIO pin for SCL
        .clk_source = I2C_CLK_SRC_DEFAULT, // Default clock source
        .glitch_ignore_cnt = 7,           // Glitch filter setting
        .flags.enable_internal_pullup = true, // Enable internal pull-ups for SDA/SCL
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
}

// Function for initializing the Si7021 sensor handle
static void si7021_init_handle(void)
{
    // Configure the Si7021 device settings
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, // 7-bit I2C address
        .device_address = SI7021_ADDR,        // Address of the Si7021 sensor
        .scl_speed_hz = 100000,               // I2C clock speed (100 kHz)
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &si7021_handle));
}

// Function to read raw data from the Si7021 sensor
static esp_err_t read_si7021_raw(uint8_t cmd, uint16_t *raw_data)
{
    uint8_t data[2];
    esp_err_t ret;

    // Send the command to the sensor
    ret = i2c_master_transmit(si7021_handle, &cmd, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SENSORS, "Error sending command: %d", ret);
        return ret;
    }

    // Add a small delay for the sensor to process the command
    vTaskDelay(pdMS_TO_TICKS(50));

    // Read the response from the sensor
    ret = i2c_master_receive(si7021_handle, data, 2, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SENSORS, "Error reading response: %d", ret);
        return ret;
    }

    *raw_data = (data[0] << 8) | data[1]; // Combine the two bytes into a 16-bit value
    return ESP_OK;
}

// Function to read temperature from the Si7021 sensor
void read_temperature(void)
{
    uint16_t raw_temp;
    esp_err_t ret = read_si7021_raw(SI7021_CMD_READ_TEMP, &raw_temp);

    if (ret == ESP_OK) {
        // Convert raw temperature data to Celsius
        temperature  = ((175.72f * raw_temp) / 65536.0f) - 46.85f;
        ESP_LOGI(TAG_SENSORS, "Temperature: %.2f°C", temperature);
    } else {
        ESP_LOGE(TAG_SENSORS, "Failed to read temperature");
    }
}

// Function to read humidity from the Si7021 sensor
void read_humidity(void)
{
    uint16_t raw_hum;
    esp_err_t ret = read_si7021_raw(SI7021_CMD_READ_HUM, &raw_hum);

    if (ret == ESP_OK) {
        // Convert raw humidity data to percentage
        humidity = ((125.0f * raw_hum) / 65536.0f) - 6.0f;
        if (humidity > 100.0f) humidity = 100.0f; // Constrain values
        if (humidity < 0.0f) humidity = 0.0f;
        ESP_LOGI(TAG_SENSORS, "Humidity: %.2f%%", humidity);
    } else {
        ESP_LOGE(TAG_SENSORS, "Failed to read humidity");
    }
}

// Function to read light level from the phototransistor
void read_photo_transistor(void) 
{
    int adc_raw;
    esp_err_t ret = adc_oneshot_read(adc1_handle, PHOTO_TRANSISTOR_PIN, &adc_raw);
    if (ret == ESP_OK) {
        // Calculate light percentage based on ADC value (0-4095 range)
        light_level = (adc_raw * 100) / 4095;
        //ESP_LOGI(TAG, "Light level: %d%% (ADC: %d)", light_percentage, adc_raw);  //Print the value and the ADC raw
        ESP_LOGI(TAG_SENSORS, "Light level: %d%%", light_level);
    } else {
        ESP_LOGE(TAG_SENSORS, "Error reading photo transistor");
    }
}

// Helper function to map a value from one range to another
static int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to read soil moisture level
void read_soil_moisture(void)
{
    int adc_raw;
    esp_err_t ret = adc_oneshot_read(adc1_handle, SOIL_MOISTURE_PIN, &adc_raw);
    if (ret == ESP_OK) {
        // Constrain ADC values to avoid overflow
        if (adc_raw > MOISTURE_AIR_VALUE) adc_raw = MOISTURE_AIR_VALUE;
        if (adc_raw < MOISTURE_WATER_VALUE) adc_raw = MOISTURE_WATER_VALUE;

        // Map ADC value to moisture percentage
        soil_moisture_percentage = map(adc_raw, MOISTURE_AIR_VALUE, MOISTURE_WATER_VALUE, 0, 100);        
        
        // Print Soil Moisture: 0%
        ESP_LOGI(TAG_SENSORS, "Soil Moisture: %d%%", soil_moisture_percentage);

    } else {
        ESP_LOGE(TAG_SENSORS, "Error reading soil moisture sensor");
    }
}

// Task for reading all sensor data periodically
void sensor_task(void *pvParameters)
{
    // Delay before starting sensor readings to allow initialization
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        read_temperature();       // Read temperature
        read_humidity();          // Read humidity
        read_photo_transistor(); // Read light level
        read_soil_moisture();    // Read soil moisture level
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay between readings (1 second)
    }
}

// Function to initialize GPIO for the pump
static void pump_gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_GPIO_PIN), // Configure GPIO25
        .mode = GPIO_MODE_OUTPUT,               // Set as output
        .pull_up_en = GPIO_PULLUP_DISABLE,      // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE          // No interrupts
    };
    gpio_config(&io_conf);

    // Set pump to OFF initially
    gpio_set_level(PUMP_GPIO_PIN, 0);
}

// Function to turn the pump ON
void pump_on(void) {
    gpio_set_level(PUMP_GPIO_PIN, 1);  // Set GPIO pin to high, turning the pump on
    //ESP_LOGI(TAG_PUMP, "Pump turned ON.");
}

// Function to turn the pump OFF
void pump_off(void) {
    gpio_set_level(PUMP_GPIO_PIN, 0);  // Set GPIO pin to low, turning the pump off
    //ESP_LOGI(TAG_PUMP, "Pump turned OFF.");
}


// Function to simulate random initial water tank level between 40% and 60%
static void initialize_water_tank_level(void) {
    srand(time(NULL));  // Seed the random number generator
    water_tank_level = (rand() % 21) + 40;  // Random value between 40 and 60
    ESP_LOGI(TAG_PUMP, "Water tank initialized to %d%%", water_tank_level);
}

// Function to update water tank level by 1% each time the pump is on
void update_water_tank_level(void) {
    if (water_tank_level > 0) {
        water_tank_level -= 1;
        ESP_LOGI(TAG_PUMP, "Water tank level: %d%%", water_tank_level); // Log the updated water level
    }
}


void pump_control_task(void *pvParameters) {
    bool pump_is_on = false;

    while (1) {
        // Check if the water tank level is below 20%
        if (water_tank_level <= 20) {
            ESP_LOGI(TAG_PUMP, "Water tank level is below 20%%. Turning pump OFF.");
            pump_off();  // Ensure the pump is off if water level is low
            pump_is_on = false;
            pump_state = "OFF";
            vTaskDelay(pdMS_TO_TICKS(1000));  // Small delay before checking again
            continue;  // Skip the rest of the logic if water tank level is low
        }

        // Check if the water tank level is sufficient (>20%)
        if (water_tank_level > 20) {
            // If all conditions are met, turn the pump on
            if (soil_moisture_percentage < 50 && temperature >= 12.0f && temperature <= 25.0f && humidity < 90.0f && light_level < 60) {
                if (!pump_is_on) {
                    pump_is_on = true;
                    pump_state = "ON";
                    pump_on();  // Turn pump on
                    ESP_LOGI(TAG_PUMP, "Pump is ON. Decreasing water level.");
                }

                // Keep the pump on and decrease the water level every second
                while (pump_is_on && water_tank_level > 0) {
                    update_water_tank_level();  // Decrease the water level by 1%
                    ESP_LOGI(TAG_PUMP, "Water tank level: %d%%", water_tank_level); // Log the updated water level
                    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second before decreasing again

                    // Check conditions to turn off the pump based on sensors
                    if (soil_moisture_percentage >= 60) {
                        ESP_LOGI(TAG_PUMP, "Soil moisture is sufficient. Turning pump off.");
                        pump_is_on = false;
                        pump_state = "OFF";
                        pump_off();
                        break;
                    }
                    if (temperature < 12.0f || temperature > 25.0f) {
                        ESP_LOGI(TAG_PUMP, "Temperature out of range. Turning pump off.");
                        pump_is_on = false;
                        pump_state = "OFF";
                        pump_off();
                        break;
                    }
                    if (humidity > 90.0f) {
                        ESP_LOGI(TAG_PUMP, "Humidity too high. Turning pump off.");
                        pump_is_on = false;
                        pump_state = "OFF";
                        pump_off();
                        break;
                    }
                    if (light_level > 60) {
                        ESP_LOGI(TAG_PUMP, "Light level too high. Turning pump off.");
                        pump_is_on = false;
                        pump_state = "OFF";
                        pump_off();
                        break;
                    }
                    if (water_tank_level <= 0) {
                        ESP_LOGI(TAG_PUMP, "Water tank is empty. Turning pump off.");
                        pump_is_on = false;
                        pump_state = "OFF";
                        pump_off();
                        break;
                    }
                    // Check if water tank level drops below 20% while pump is on
                    if (water_tank_level <= 20) {
                        ESP_LOGI(TAG_PUMP, "Water tank level dropped below 20%%. Turning pump OFF.");
                        pump_is_on = false;
                        pump_state = "OFF";
                        pump_off();
                        break;
                    }
                }
            } else {
                ESP_LOGI(TAG_PUMP, "Water tank level is above 20%% but conditions for turning pump on are not met.");
                vTaskDelay(pdMS_TO_TICKS(1000)); // Small delay before checking again
            }
        }
    }
}

// JSON Logging Task
void json_log_task(void *pvParameters) {
    while (1) {
        // Define a buffer for the formatted string
        char log_string[256];

        // Format the sensor values into a string
        snprintf(log_string, sizeof(log_string),
         "{\"brightness_sensor\": %d, \"humidity_sensor\": %.2f, "
         "\"moisture_sensor\": %d, \"temperature_sensor\": %.2f, "
         "\"pump_state\": \"%s\", \"water_level_sensor\": %d}",
         light_level, humidity, soil_moisture_percentage,
         temperature, pump_state, water_tank_level);


        // Log the string
        ESP_LOGI("JSON_LOG", "%s", log_string);

        // Wait for 5 seconds before logging again
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Main application entry point
void app_main(void) {
    ESP_LOGI(TAG_SENSORS, "Initializing sensors...");
    ESP_LOGI(TAG_PUMP, "Initializing pump...");

    // Initialize water tank level
    initialize_water_tank_level();

    // Initialize I2C bus and Si7021 sensor (existing functions)
    i2c_master_init_bus();
    si7021_init_handle();
    adc_init();

    // Initialize pump GPIO
    pump_gpio_init();

    // Verify if Si7021 sensor is responding
    esp_err_t err = i2c_master_probe(bus_handle, SI7021_ADDR, I2C_MASTER_TIMEOUT_MS);
    if (err == ESP_OK) {
        ESP_LOGI(TAG_SENSORS, "Si7021 found at address 0x%02X", SI7021_ADDR);

        // Create tasks for sensors and pump control
        xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
        xTaskCreate(pump_control_task, "pump_control_task", 2048, NULL, 5, NULL);
        xTaskCreate(json_log_task, "json_log_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG_SENSORS, "Could not find Si7021 at address 0x%02X", SI7021_ADDR);
    }
}