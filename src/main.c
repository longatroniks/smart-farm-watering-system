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
#include <stdlib.h>
#include <time.h>
#include "cJSON.h"

// Our drivers
#include "si7021.h"
#include "weather_client.h"
#include "telemetry_client.h"

// Task configurations
#define STACK_SIZE_SENSOR    4096
#define STACK_SIZE_PUMP      2048
#define STACK_SIZE_JSON      8192  // Increased for HTTPS operations
#define TASK_PRIORITY        5

// GPIO configurations
#define PUMP_GPIO_PIN        25
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS 1000

// ADC configurations
#define PHOTO_TRANSISTOR_PIN ADC_CHANNEL_6
#define SOIL_MOISTURE_PIN    ADC_CHANNEL_7
#define MOISTURE_AIR_VALUE   2450
#define MOISTURE_WATER_VALUE 1150

// Logging Tags
static const char *TAG_SENSORS = "SENSORS";
static const char *TAG_PUMP = "PUMP";

// Task handles
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t pump_task_handle = NULL;
static TaskHandle_t json_task_handle = NULL;

// Global variables
static int water_tank_level = 0;
static int soil_moisture_percentage = 0;
static float temperature = 0.0;
static float humidity = 0.0;
static int light_level = 0;
static const char *pump_state = "OFF";
static weather_data_t current_weather = {0};

// Peripheral handles
static i2c_master_bus_handle_t bus_handle;
static adc_oneshot_unit_handle_t adc1_handle;
static si7021_dev_t si7021_dev;

static void adc_init(void)
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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SOIL_MOISTURE_PIN, &config));
}

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

void read_photo_transistor(void)
{
    int adc_raw;
    esp_err_t ret = adc_oneshot_read(adc1_handle, PHOTO_TRANSISTOR_PIN, &adc_raw);
    if (ret == ESP_OK)
    {
        light_level = (adc_raw * 100) / 4095;
    }
    else
    {
        ESP_LOGE(TAG_SENSORS, "Error reading photo transistor: %s", esp_err_to_name(ret));
    }
}

static int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void read_soil_moisture(void)
{
    int adc_raw;
    esp_err_t ret = adc_oneshot_read(adc1_handle, SOIL_MOISTURE_PIN, &adc_raw);
    if (ret == ESP_OK)
    {
        if (adc_raw > MOISTURE_AIR_VALUE)
            adc_raw = MOISTURE_AIR_VALUE;
        if (adc_raw < MOISTURE_WATER_VALUE)
            adc_raw = MOISTURE_WATER_VALUE;
        soil_moisture_percentage = map(adc_raw, MOISTURE_AIR_VALUE, MOISTURE_WATER_VALUE, 0, 100);
    }
    else
    {
        ESP_LOGE(TAG_SENSORS, "Error reading soil moisture: %s", esp_err_to_name(ret));
    }
}

void sensor_task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1)
    {
        esp_err_t ret;
        ret = si7021_read_temperature(&si7021_dev, &temperature);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_SENSORS, "Error reading temperature: %s", esp_err_to_name(ret));
        }

        ret = si7021_read_humidity(&si7021_dev, &humidity);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_SENSORS, "Error reading humidity: %s", esp_err_to_name(ret));
        }

        read_photo_transistor();
        read_soil_moisture();

        if (weather_client_is_connected())
        {
            current_weather = weather_client_get_data();
        }

        ESP_LOGI(TAG_SENSORS, "Local Sensors - T:%.1f°C H:%.1f%% L:%d%% M:%d%% W:%d%% P:%s",
                 temperature, humidity, light_level, soil_moisture_percentage,
                 water_tank_level, pump_state);

        ESP_LOGI(TAG_SENSORS, "Weather Station - P:%.1fhPa W:%.1fm/s R:%.1f%%",
                 current_weather.pressure, current_weather.wind_speed,
                 current_weather.rain_probability);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void pump_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(PUMP_GPIO_PIN, 0);
}

void pump_on(void)
{
    ESP_ERROR_CHECK(gpio_set_level(PUMP_GPIO_PIN, 1));
}

void pump_off(void)
{
    ESP_ERROR_CHECK(gpio_set_level(PUMP_GPIO_PIN, 0));
}

static void initialize_water_tank_level(void)
{
    srand(time(NULL));
    water_tank_level = (rand() % 21) + 40;
    ESP_LOGI(TAG_PUMP, "Water tank initialized to %d%%", water_tank_level);
}

void update_water_tank_level(void)
{
    if (water_tank_level > 0)
    {
        water_tank_level -= 1;
        if (water_tank_level % 10 == 0 || water_tank_level <= 20)
        {
            ESP_LOGI(TAG_PUMP, "Water tank level: %d%%", water_tank_level);
        }
    }
}

void pump_control_task(void *pvParameters)
{
    bool pump_is_on = false;

    while (1)
    {
        if (water_tank_level <= 20)
        {
            ESP_LOGW(TAG_PUMP, "⚠️ Low water level (%d%%). Pump OFF", water_tank_level);
            pump_off();
            pump_is_on = false;
            pump_state = "OFF";
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (water_tank_level > 20)
        {
            if (soil_moisture_percentage < 50 && temperature >= 12.0f &&
                temperature <= 25.0f && humidity < 90.0f && light_level < 60)
            {
                if (!pump_is_on)
                {
                    pump_is_on = true;
                    pump_state = "ON";
                    pump_on();
                    ESP_LOGI(TAG_PUMP, "Pump ON - Starting irrigation");
                }

                while (pump_is_on && water_tank_level > 0)
                {
                    update_water_tank_level();
                    vTaskDelay(pdMS_TO_TICKS(1000));

                    if (soil_moisture_percentage >= 60 ||
                        temperature < 12.0f || temperature > 25.0f ||
                        humidity > 90.0f || light_level > 60 ||
                        water_tank_level <= 20)
                    {
                        pump_is_on = false;
                        pump_state = "OFF";
                        pump_off();
                        ESP_LOGI(TAG_PUMP, "Pump OFF - Conditions no longer met");
                        break;
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void json_log_task(void *pvParameters)
{
    while (1) {
        cJSON *root = cJSON_CreateObject();
        if (root == NULL) {
            ESP_LOGE(TAG_SENSORS, "Failed to create JSON object");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        cJSON_AddNumberToObject(root, "brightness_sensor", light_level);
        cJSON_AddNumberToObject(root, "humidity_sensor", humidity);
        cJSON_AddNumberToObject(root, "moisture_sensor", soil_moisture_percentage);
        cJSON_AddNumberToObject(root, "temperature_sensor", temperature);
        cJSON_AddStringToObject(root, "pump_state", pump_state);
        cJSON_AddNumberToObject(root, "water_level_sensor", water_tank_level);

        char *json_string = cJSON_PrintUnformatted(root);
        if (json_string == NULL) {
            ESP_LOGE(TAG_SENSORS, "Failed to create JSON string");
            cJSON_Delete(root);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        if (telemetry_client_is_connected()) {
            if (telemetry_client_publish(json_string)) {
                ESP_LOGI(TAG_SENSORS, "Sent sensor data to ThingsBoard");
            } else {
                ESP_LOGW(TAG_SENSORS, "Failed to send sensor data to ThingsBoard");
            }
        }

        cJSON_Delete(root);
        free(json_string);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG_SENSORS, "Starting irrigation system...");

    // Initialize hardware
    initialize_water_tank_level();
    i2c_master_init_bus();
    adc_init();
    pump_gpio_init();

    // Initialize Si7021 sensor
    esp_err_t ret = si7021_init(&si7021_dev, bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SENSORS, "Si7021 initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    if (si7021_probe(&si7021_dev) != ESP_OK)
    {
        ESP_LOGE(TAG_SENSORS, "Si7021 probe failed");
        return;
    }

    // Initialize networking clients
    weather_client_init("Karlo's iPhone", "kkkkkkkk");
    vTaskDelay(pdMS_TO_TICKS(5000));

    if (!weather_client_is_connected()) {
        ESP_LOGE(TAG_SENSORS, "Weather client failed to initialize");
        return;
    }

    telemetry_client_init();
    vTaskDelay(pdMS_TO_TICKS(3000));

    if (!telemetry_client_is_connected()) {
        ESP_LOGE(TAG_SENSORS, "Telemetry client failed to initialize");
        return;
    }

    // Create tasks
    BaseType_t xReturned;

    xReturned = xTaskCreate(
        sensor_task,
        "sensor_task",
        STACK_SIZE_SENSOR,
        NULL,
        TASK_PRIORITY,
        &sensor_task_handle);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG_SENSORS, "Failed to create sensor task");
        return;
    }

    xReturned = xTaskCreate(
        pump_control_task,
        "pump_control_task",
        STACK_SIZE_PUMP,
        NULL,
        TASK_PRIORITY,
        &pump_task_handle);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG_SENSORS, "Failed to create pump control task");
        vTaskDelete(sensor_task_handle);
        return;
    }

    xReturned = xTaskCreate(
        json_log_task,
        "json_log_task",
        STACK_SIZE_JSON,
        NULL,
        TASK_PRIORITY,
        &json_task_handle);
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG_SENSORS, "Failed to create JSON log task");
        vTaskDelete(sensor_task_handle);
        vTaskDelete(pump_task_handle);
        return;
    }

    ESP_LOGI(TAG_SENSORS, "System initialization complete");
}