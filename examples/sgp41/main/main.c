/**
 * Simple example with humidity sensor and SGP41
 */
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <aht.h>
#include <sgp4x.h>
#include <string.h>
#include <esp_err.h>
#include <esp_log.h>

/* float is used in printf(). you need non-default configuration in
 * sdkconfig for ESP8266, which is enabled by default for this
 * example. see sdkconfig.defaults.esp8266
 */

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_GND
#define ADDR AHT_I2C_ADDRESS_GND
#endif
#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_VCC
#define ADDR AHT_I2C_ADDRESS_VCC
#endif

#ifdef CONFIG_EXAMPLE_TYPE_AHT1x
#define AHT_TYPE AHT_TYPE_AHT1x
#endif

#ifdef CONFIG_EXAMPLE_TYPE_AHT20
#define AHT_TYPE AHT_TYPE_AHT20
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "sgp41-example";

static const char *voc_index_name(int32_t voc_index)
{
    if (voc_index <= 0) return "INVALID VOC INDEX";
    else if (voc_index <= 10) return "unbelievable clean";
    else if (voc_index <= 30) return "extremely clean";
    else if (voc_index <= 50) return "highly clean";
    else if (voc_index <= 70) return "very clean";
    else if (voc_index <= 90) return "clean";
    else if (voc_index <= 120) return "normal";
    else if (voc_index <= 150) return "moderately polluted";
    else if (voc_index <= 200) return "highly polluted";
    else if (voc_index <= 300) return "extremely polluted";

    return "RUN!";
}

static const char *nox_index_name(int32_t nox_index)
{
    if (nox_index <= 0) return "INVALID NOX INDEX";
    else if (nox_index <= 1) return "normal";
    else if (nox_index <= 20) return "slightly polluted";
    else if (nox_index <= 50) return "moderately polluted";
    else if (nox_index <= 100) return "highly polluted";
    else if (nox_index <= 200) return "extremely polluted";

    return "RUN!";
}

void task(void *pvParamters)
{
    aht_t aht_dev = { 0 };
    sgp4x_t sgp_dev = { 0 };

    // Setup AHT20
    aht_dev.mode = AHT_MODE_NORMAL;
    aht_dev.type = AHT_TYPE;

    ESP_ERROR_CHECK(aht_init_desc(&aht_dev, ADDR, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(aht_init(&aht_dev));

    bool calibrated;
    ESP_ERROR_CHECK(aht_get_status(&aht_dev, NULL, &calibrated));
    if (calibrated)
        ESP_LOGI(TAG, "Sensor calibrated");
    else
        ESP_LOGW(TAG, "Sensor not calibrated!");
    ESP_LOGI(TAG, "Humidity sensor initialized");

    // Setup SGP41
    memset(&sgp_dev, 0, sizeof(sgp_dev));
    ESP_ERROR_CHECK(sgp4x_init_desc(&sgp_dev, SGP4x_TYPE_SGP41, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(sgp4x_init(&sgp_dev));
    ESP_LOGI(TAG, "SGP41 initialized. Serial: 0x%04x%04x%04x", sgp_dev.serial[0], sgp_dev.serial[1], sgp_dev.serial[2]);

    // Wait until all set up
    vTaskDelay(pdMS_TO_TICKS(250));

    // Start conditioning the sensor will exit conditioning with the first measurement
    // WARNING: This MUST NOT run longer than 10 seconds to avoid damage to the sensor.
    TickType_t last_wakeup = xTaskGetTickCount();
    float temperature, humidity;
    uint16_t voc_raw;
    ESP_ERROR_CHECK(aht_get_data(&aht_dev, &temperature, &humidity));
    ESP_ERROR_CHECK(sgp4x_start_conditioning(&sgp_dev, humidity, temperature, &voc_raw));
    ESP_LOGI(TAG, "started SGP41 conditioning");
    ESP_LOGI(TAG, "%.2f °C, %.2f %%, VOC raw: %" PRIu16, temperature, humidity, voc_raw);

    // Optionally, it's possible to read the VOC value during conditioning by repeatadly calling start_conditioning
    TickType_t voc_last_wakeup = xTaskGetTickCount();
    for (int i = 0; i < 9; i++)
    {
        ESP_ERROR_CHECK(aht_get_data(&aht_dev, &temperature, &humidity));
        ESP_ERROR_CHECK(sgp4x_start_conditioning(&sgp_dev, humidity, temperature, &voc_raw));
        ESP_LOGI(TAG, "%.2f °C, %.2f %%, VOC raw: %" PRIu16, temperature, humidity, voc_raw);
        vTaskDelayUntil(&voc_last_wakeup, pdMS_TO_TICKS(1000));
    }

    vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(10000));
    ESP_LOGI(TAG, "finished SGP41 conditioning");

    while (1)
    {
        // Get the RHT values
        ESP_ERROR_CHECK(aht_get_data(&aht_dev, &temperature, &humidity));

        // Feed it to SGP41
        int32_t voc_index, nox_index;
        ESP_ERROR_CHECK(sgp4x_measure_index(&sgp_dev, humidity, temperature, &voc_index, &nox_index));
        ESP_LOGI(TAG, "%.2f °C, %.2f %%, VOC index: %" PRIi32 ", NOX index: %" PRIi32 ", Air is: VOC [%s], NOX [%s]", temperature, humidity, voc_index, nox_index, voc_index_name(voc_index),
                 nox_index_name(nox_index));

        // Wait until 1 seconds (VOC / NOx cycle time) are over.
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(1000));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(task, "sgp41", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
