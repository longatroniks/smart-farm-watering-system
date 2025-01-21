
#include "irrigation_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "si7021.h"
#include "weather_client.h"
#include "telemetry_client.h"
#include "main.h"
#include <stdlib.h>
#include <time.h>

static const char *TAG = "IRRIGATION";

// Task handles
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t pump_task_handle = NULL;
static TaskHandle_t telemetry_task_handle = NULL;

// System status
static system_status_t system_status = {
    .water_tank_level = 0,
    .soil_moisture_percentage = 0,
    .temperature = 0.0,
    .humidity = 0.0,
    .light_level = 0,
    .pump_state = "OFF"
};

// Peripheral handles
static i2c_master_bus_handle_t bus_handle;
static adc_oneshot_unit_handle_t adc1_handle;
static si7021_dev_t si7021_dev;

// Hardware initialization functions
static esp_err_t init_i2c(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_config, &bus_handle);
}

static esp_err_t init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PHOTO_TRANSISTOR_PIN, &config));
    return adc_oneshot_config_channel(adc1_handle, SOIL_MOISTURE_PIN, &config);
}

static esp_err_t init_pump(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    return gpio_set_level(PUMP_GPIO_PIN, 0);
}

// Sensor reading functions
static void read_sensors(void)
{
    // Read temperature and humidity
    esp_err_t ret = si7021_read_temperature(&si7021_dev, &system_status.temperature);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Temperature read error: %s", esp_err_to_name(ret));
    }

    ret = si7021_read_humidity(&si7021_dev, &system_status.humidity);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Humidity read error: %s", esp_err_to_name(ret));
    }

    // Read light level
    int adc_raw;
    ret = adc_oneshot_read(adc1_handle, PHOTO_TRANSISTOR_PIN, &adc_raw);
    if (ret == ESP_OK) {
        system_status.light_level = (adc_raw * 100) / 4095;
    }

    // Read soil moisture
    ret = adc_oneshot_read(adc1_handle, SOIL_MOISTURE_PIN, &adc_raw);
    if (ret == ESP_OK) {
        if (adc_raw > MOISTURE_AIR_VALUE) adc_raw = MOISTURE_AIR_VALUE;
        if (adc_raw < MOISTURE_WATER_VALUE) adc_raw = MOISTURE_WATER_VALUE;
        system_status.soil_moisture_percentage = 
            ((MOISTURE_AIR_VALUE - adc_raw) * 100) / (MOISTURE_AIR_VALUE - MOISTURE_WATER_VALUE);
    }
}

// Task definitions
static void sensor_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        read_sensors();
        
        ESP_LOGI(TAG, "Sensors - T:%.1f°C H:%.1f%% L:%d%% M:%d%% W:%d%% P:%s",
                 system_status.temperature, system_status.humidity, 
                 system_status.light_level, system_status.soil_moisture_percentage,
                 system_status.water_tank_level, system_status.pump_state);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2000));
    }
}

static void pump_control_task(void *pvParameters)
{
    bool pump_is_on = false;
    
    while (1) {
        bool should_pump = 
            system_status.water_tank_level > 20 &&
            system_status.soil_moisture_percentage < 50 &&
            system_status.temperature >= 12.0f &&
            system_status.temperature <= 25.0f &&
            system_status.humidity < 90.0f &&
            system_status.light_level < 60;

        if (should_pump && !pump_is_on) {
            gpio_set_level(PUMP_GPIO_PIN, 1);
            pump_is_on = true;
            system_status.pump_state = "ON";
            ESP_LOGI(TAG, "Pump ON - Starting irrigation");
        } else if (!should_pump && pump_is_on) {
            gpio_set_level(PUMP_GPIO_PIN, 0);
            pump_is_on = false;
            system_status.pump_state = "OFF";
            ESP_LOGI(TAG, "Pump OFF - Conditions not met");
        }

        if (pump_is_on) {
            system_status.water_tank_level--;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void telemetry_task(void *pvParameters)
{
    while (1) {
        if (telemetry_client_is_connected()) {
            cJSON *root = cJSON_CreateObject();
            if (root) {
                cJSON_AddNumberToObject(root, "brightness_sensor", system_status.light_level);
                cJSON_AddNumberToObject(root, "humidity_sensor", system_status.humidity);
                cJSON_AddNumberToObject(root, "moisture_sensor", system_status.soil_moisture_percentage);
                cJSON_AddNumberToObject(root, "temperature_sensor", system_status.temperature);
                cJSON_AddStringToObject(root, "pump_state", system_status.pump_state);
                cJSON_AddNumberToObject(root, "water_level_sensor", system_status.water_tank_level);

                char *json_string = cJSON_PrintUnformatted(root);
                if (json_string) {
                    if (telemetry_client_publish(json_string)) {
                        ESP_LOGI(TAG, "Telemetry data sent");
                    }
                    free(json_string);
                }
                cJSON_Delete(root);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

esp_err_t irrigation_system_init(void)
{
    ESP_LOGI(TAG, "Initializing irrigation system");

    // Initialize random water tank level
    srand(time(NULL));
    system_status.water_tank_level = (rand() % 21) + 40;

    // Initialize hardware
    ESP_ERROR_CHECK(init_i2c());
    ESP_ERROR_CHECK(init_adc());
    ESP_ERROR_CHECK(init_pump());
    ESP_ERROR_CHECK(si7021_init(&si7021_dev, bus_handle));
    ESP_ERROR_CHECK(si7021_probe(&si7021_dev));

    // Initialize network clients
    ESP_ERROR_CHECK(wifi_driver_init("Karlo's iPhone", "kkkkkkkk"));
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    ESP_ERROR_CHECK(weather_client_init());
    ESP_ERROR_CHECK(telemetry_client_init());
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Create tasks
    BaseType_t ret = xTaskCreate(sensor_task, "sensor_task",
                                STACK_SIZE_SENSOR, NULL, TASK_PRIORITY,
                                &sensor_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return ESP_FAIL;
    }

    ret = xTaskCreate(pump_control_task, "pump_task",
                     STACK_SIZE_PUMP, NULL, TASK_PRIORITY,
                     &pump_task_handle);
    if (ret != pdPASS) {
        vTaskDelete(sensor_task_handle);
        ESP_LOGE(TAG, "Failed to create pump task");
        return ESP_FAIL;
    }

    ret = xTaskCreate(telemetry_task, "telemetry_task",
                     STACK_SIZE_JSON, NULL, TASK_PRIORITY,
                     &telemetry_task_handle);
    if (ret != pdPASS) {
        vTaskDelete(sensor_task_handle);
        vTaskDelete(pump_task_handle);
        ESP_LOGE(TAG, "Failed to create telemetry task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Irrigation system initialized successfully");
    return ESP_OK;
}

system_status_t irrigation_system_get_status(void)
{
    return system_status;
}
