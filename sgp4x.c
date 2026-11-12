/**
 * @file sgp4x.c
 *
 * ESP-IDF driver for SGP40 / SGP41 Indoor Air Quality Sensor for VOC (& NOx) Measurements
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2025 Michael Hegel <mrbotastic@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>
#include "sgp4x.h"
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ets_sys.h>

#define I2C_FREQ_HZ 400000

static const char *TAG = "sgp4x";

#define CMD_SOFT_RESET         0x0006
#define CMD_FEATURESET         0x202f
#define CMD_SGP40_MEASURE_RAW  0x260f
#define CMD_SGP41_MEASURE_RAW  0x2619
#define CMD_SGP41_CONDITIONING 0x2612
#define CMD_SELF_TEST          0x280e
#define CMD_SERIAL             0x3682
#define CMD_HEATER_OFF         0x3615

#define TIME_SOFT_RESET        10
#define TIME_FEATURESET        10
#define TIME_SGP40_MEASURE_RAW 30
#define TIME_SGP41_MEASURE_RAW 50
#define TIME_SGP40_SELF_TEST   250
#define TIME_SGP41_SELF_TEST   320
#define TIME_HEATER_OFF        10
#define TIME_SERIAL            10

#define SGP40_SELF_TEST_OK 0xd400
#define SGP41_SELF_TEST_OK 0x0000

#define CHECK(x)                                                                                                                                                                                       \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        esp_err_t __;                                                                                                                                                                                  \
        if ((__ = x) != ESP_OK)                                                                                                                                                                        \
            return __;                                                                                                                                                                                 \
    }                                                                                                                                                                                                  \
    while (0)
#define CHECK_ARG(ARG)                                                                                                                                                                                 \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if (!(ARG))                                                                                                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                                                                                                \
    }                                                                                                                                                                                                  \
    while (0)

static uint8_t crc8(const uint8_t *data, size_t count)
{
    uint8_t res = 0xff;

    for (size_t i = 0; i < count; ++i)
    {
        res ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (res & 0x80)
                res = (res << 1) ^ 0x31;
            else
                res = (res << 1);
        }
    }
    return res;
}

// The sensor uses big-endian words, the buffer is a byte array: the words are
// composed and decomposed byte by byte to avoid an unaligned access
static inline void put_u16be(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static inline uint16_t get_u16be(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static esp_err_t send_cmd(i2c_dev_t *dev, uint16_t cmd, uint16_t *data, size_t words)
{
    uint8_t buf[2 + words * 3];
    // add command
    put_u16be(buf, cmd);
    if (data && words)
        // add arguments
        for (size_t i = 0; i < words; i++)
        {
            uint8_t *p = buf + 2 + i * 3;
            put_u16be(p, data[i]);
            *(p + 2) = crc8(p, 2);
        }

    ESP_LOGV(TAG, "Sending buffer:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, sizeof(buf), ESP_LOG_VERBOSE);

    return i2c_dev_write(dev, NULL, 0, buf, sizeof(buf));
}

static esp_err_t read_resp(i2c_dev_t *dev, uint16_t *data, size_t words)
{
    uint8_t buf[words * 3];
    CHECK(i2c_dev_read(dev, NULL, 0, buf, sizeof(buf)));

    ESP_LOGV(TAG, "Received buffer:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, sizeof(buf), ESP_LOG_VERBOSE);

    for (size_t i = 0; i < words; i++)
    {
        uint8_t *p = buf + i * 3;
        uint8_t crc = crc8(p, 2);
        if (crc != *(p + 2))
        {
            ESP_LOGE(TAG, "Invalid CRC 0x%02x, expected 0x%02x", crc, *(p + 2));
            return ESP_ERR_INVALID_CRC;
        }
        data[i] = get_u16be(p);
    }
    return ESP_OK;
}

// Sends the command and reads the response. The device mutex must be held by the caller
static esp_err_t execute_cmd_locked(sgp4x_t *dev, uint16_t cmd, uint32_t timeout_ms, uint16_t *out_data, size_t out_words, uint16_t *in_data, size_t in_words)
{
    CHECK(send_cmd(&dev->i2c_dev, cmd, out_data, out_words));
    if (timeout_ms)
    {
        if (timeout_ms > 10)
            vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        else
            ets_delay_us(timeout_ms * 1000);
    }
    if (in_data && in_words)
        CHECK(read_resp(&dev->i2c_dev, in_data, in_words));

    return ESP_OK;
}

static esp_err_t execute_cmd(sgp4x_t *dev, uint16_t cmd, uint32_t timeout_ms, uint16_t *out_data, size_t out_words, uint16_t *in_data, size_t in_words)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    esp_err_t err = execute_cmd_locked(dev, cmd, timeout_ms, out_data, out_words, in_data, in_words);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return err;
}

static esp_err_t convert_temperature_and_humidity(float humidity, float temperature, uint16_t *params)
{
    CHECK_ARG(params);

    if (isnan(humidity) || isnan(temperature))
    {
        params[0] = 0x8000;
        params[1] = 0x6666;
        ESP_LOGW(TAG, "Uncompensated measurement");
        return ESP_OK;
    }

    // See table 10 "Conversion of relative humidity in % and temperature in °C into ticks"
    // of the SGP40 and the SGP41 datasheets
    if (humidity < 0.0 || humidity > 100.0)
    {
        ESP_LOGE(TAG, "Humidity %.2f %% is out of the range 0..100", humidity);
        return ESP_ERR_INVALID_ARG;
    }
    if (temperature < -45.0 || temperature > 130.0)
    {
        ESP_LOGE(TAG, "Temperature %.2f °C is out of the range -45..130", temperature);
        return ESP_ERR_INVALID_ARG;
    }

    params[0] = (uint16_t)(humidity / 100.0 * 65535.0);
    params[1] = (uint16_t)((temperature + 45.0) / 175.0 * 65535.0);

    return ESP_OK;
}

////////////////////////////////////////////////////////////////////////////////

esp_err_t sgp4x_init_desc(sgp4x_t *dev, sgp4x_type_t type, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    CHECK_ARG(type == SGP4x_TYPE_SGP40 || type == SGP4x_TYPE_SGP41);

    dev->type = type;
    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = SGP4X_ADDR;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t sgp4x_free_desc(sgp4x_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t sgp4x_soft_reset(sgp4x_t *dev)
{
    CHECK_ARG(dev);

    // The datasheets call this command a general call, but it is sent to the device
    // address here, so the other devices on the bus stay untouched
    return execute_cmd(dev, CMD_SOFT_RESET, TIME_SOFT_RESET, NULL, 0, NULL, 0);
}

esp_err_t sgp4x_init(sgp4x_t *dev)
{
    CHECK_ARG(dev);
    if (dev->type != SGP4x_TYPE_SGP40 && dev->type != SGP4x_TYPE_SGP41)
    {
        ESP_LOGE(TAG, "Unknown device type 0x%0X", dev->type);
        return ESP_ERR_NOT_SUPPORTED;
    }

    CHECK(execute_cmd(dev, CMD_SERIAL, TIME_SERIAL, NULL, 0, dev->serial, 3));
    CHECK(execute_cmd(dev, CMD_FEATURESET, TIME_FEATURESET, NULL, 0, &dev->featureset, 1));

    GasIndexAlgorithm_init(&dev->voc, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    if (dev->type == SGP4x_TYPE_SGP41)
        GasIndexAlgorithm_init(&dev->nox, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);

    ESP_LOGD(TAG, "Device found. S/N: 0x%04x%04x%04x, featureset 0x%04x", dev->serial[0], dev->serial[1], dev->serial[2], dev->featureset);

    return ESP_OK;
}

esp_err_t sgp4x_self_test(sgp4x_t *dev)
{
    CHECK_ARG(dev);

    uint16_t res;
    if (dev->type != SGP4x_TYPE_SGP40 && dev->type != SGP4x_TYPE_SGP41)
        return ESP_ERR_NOT_SUPPORTED;

    CHECK(execute_cmd(dev, CMD_SELF_TEST, dev->type == SGP4x_TYPE_SGP40 ? TIME_SGP40_SELF_TEST : TIME_SGP41_SELF_TEST, NULL, 0, &res, 1));

    if (dev->type == SGP4x_TYPE_SGP40)
        return res == SGP40_SELF_TEST_OK ? ESP_OK : ESP_FAIL;

    res &= 0x0003;
    return res == SGP41_SELF_TEST_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t sgp4x_heater_off(sgp4x_t *dev)
{
    CHECK_ARG(dev);

    return execute_cmd(dev, CMD_HEATER_OFF, TIME_HEATER_OFF, NULL, 0, NULL, 0);
}

esp_err_t sgp4x_start_conditioning(sgp4x_t *dev, float humidity, float temperature, uint16_t *raw_voc)
{
    CHECK_ARG(dev && raw_voc);
    if (dev->type != SGP4x_TYPE_SGP41)
    {
        ESP_LOGE(TAG, "Conditioning is only supported for SGP41");
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint16_t params[2];
    CHECK(convert_temperature_and_humidity(humidity, temperature, params));

    return execute_cmd(dev, CMD_SGP41_CONDITIONING, TIME_SGP41_MEASURE_RAW, params, 2, raw_voc, 1);
}

// Measures the raw values. The device mutex must be held by the caller
static esp_err_t measure_raw_locked(sgp4x_t *dev, float humidity, float temperature, uint16_t *raw_voc, uint16_t *raw_nox)
{
    uint16_t params[2];
    CHECK(convert_temperature_and_humidity(humidity, temperature, params));

    if (dev->type == SGP4x_TYPE_SGP40)
        return execute_cmd_locked(dev, CMD_SGP40_MEASURE_RAW, TIME_SGP40_MEASURE_RAW, params, 2, raw_voc, 1);

    if (dev->type != SGP4x_TYPE_SGP41)
        return ESP_ERR_NOT_SUPPORTED;

    uint16_t results[2];
    CHECK(execute_cmd_locked(dev, CMD_SGP41_MEASURE_RAW, TIME_SGP41_MEASURE_RAW, params, 2, results, 2));
    *raw_voc = results[0];
    *raw_nox = results[1];

    return ESP_OK;
}

esp_err_t sgp4x_measure_raw(sgp4x_t *dev, float humidity, float temperature, uint16_t *raw_voc, uint16_t *raw_nox)
{
    CHECK_ARG(dev && raw_voc);
    if (dev->type == SGP4x_TYPE_SGP41)
        CHECK_ARG(raw_nox);
    if (dev->type != SGP4x_TYPE_SGP40 && dev->type != SGP4x_TYPE_SGP41)
        return ESP_ERR_NOT_SUPPORTED;

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    esp_err_t err = measure_raw_locked(dev, humidity, temperature, raw_voc, raw_nox);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return err;
}

esp_err_t sgp4x_measure_index(sgp4x_t *dev, float humidity, float temperature, int32_t *voc_index, int32_t *nox_index)
{
    CHECK_ARG(dev && voc_index);
    if (dev->type == SGP4x_TYPE_SGP41)
        CHECK_ARG(nox_index);
    if (dev->type != SGP4x_TYPE_SGP40 && dev->type != SGP4x_TYPE_SGP41)
        return ESP_ERR_NOT_SUPPORTED;

    // The measurement and the algorithm run under the same lock: the algorithm keeps
    // its state in the descriptor, so two tasks must not use it at the same time
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint16_t raw_voc, raw_nox;
    esp_err_t err = measure_raw_locked(dev, humidity, temperature, &raw_voc, &raw_nox);
    if (err == ESP_OK)
    {
        GasIndexAlgorithm_process(&dev->voc, raw_voc, voc_index);
        if (dev->type == SGP4x_TYPE_SGP41)
            GasIndexAlgorithm_process(&dev->nox, raw_nox, nox_index);
    }

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return err;
}
