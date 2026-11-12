/**
 * @file sgp4x.h
 * @defgroup sgp4x sgp4x
 * @{
 *
 * ESP-IDF driver for SGP40 / SGP41 Indoor Air Quality Sensor for VOC (& NOx) Measurements
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2025 Michael Hegel <mrbotastic@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SGP4X_H__
#define __SGP4X_H__

#include <stdbool.h>
#include <i2cdev.h>
#include <esp_err.h>
#include "sensirion_gas_index_algorithm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SGP4X_ADDR 0x59 //!< I2C address

/**
 * Device types
 */
typedef enum
{
    SGP4x_TYPE_SGP40 = 0, //!< SGP40
    SGP4x_TYPE_SGP41,     //!< SGP41
} sgp4x_type_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;           //!< I2C device descriptor
    sgp4x_type_t type;           //!< Device type (SGP40 or SGP41)
    uint16_t serial[3];          //!< Device serial number (3 words)
    uint16_t featureset;         //!< Device featureset version
    GasIndexAlgorithmParams voc; //!< VOC index algorithm parameters
    GasIndexAlgorithmParams nox; //!< NOx index algorithm parameters (SGP41 only)
} sgp4x_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param type Device type
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t sgp4x_init_desc(sgp4x_t *dev, sgp4x_type_t type, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp4x_free_desc(sgp4x_t *dev);

/**
 * @brief Read device information, initialize the VOC algorithm
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp4x_init(sgp4x_t *dev);

/**
 * @brief Perform a self-test. Should not be called during conditioning on SGP41.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp4x_self_test(sgp4x_t *dev);

/**
 * @brief Turn hotplate off, stop measurement and put device to idle mode. On SGP41 conditioning should be done before
 *        resuming measurements.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp4x_heater_off(sgp4x_t *dev);

/**
 * @brief Start conditioning. Must be stopped after 10s by calling one of the measure functions. SGP41 only
 *
 * @param dev Device descriptor
 * @param humidity Relative humidity, percents. Use NaN if
 *                 you want uncompensated measurement
 * @param temperature Temperature, degrees Celsius. Use NaN if
 *                 you want uncompensated measurement
 * @param[out] raw_voc Raw value, proportional to the logarithm
 *                 of the resistance of the sensing element
 * @return `ESP_OK` on success
 */
esp_err_t sgp4x_start_conditioning(sgp4x_t *dev, float humidity, float temperature, uint16_t *raw_voc);

/**
 * @brief Perform a measurement
 *
 * @param dev Device descriptor
 * @param humidity Relative humidity, percents. Use NaN if
 *                 you want uncompensated measurement
 * @param temperature Temperature, degrees Celsius. Use NaN if
 *                    you want uncompensated measurement
 * @param[out] raw_voc Raw value, proportional to the logarithm
 *                 of the resistance of the sensing element
 * @param[out] raw_nox Raw value, proportional to the logarithm
 *                 of the resistance of the sensing element. SGP41 only
 * @return `ESP_OK` on success
 */
esp_err_t sgp4x_measure_raw(sgp4x_t *dev, float humidity, float temperature, uint16_t *raw_voc, uint16_t *raw_nox);

/**
 * @brief Perform a measurement and update VOC index
 *
 * @param dev Device descriptor
 * @param humidity Relative humidity, percents. Use NaN if
 *                 you want uncompensated measurement
 * @param temperature Temperature, degrees Celsius. Use NaN if
 *                    you want uncompensated measurement
 * @param[out] voc_index Calculated VOC index
 * @param[out] nox_index Calculated NOx index. SGP41 only
 * @return `ESP_OK` on success
 */
esp_err_t sgp4x_measure_index(sgp4x_t *dev, float humidity, float temperature, int32_t *voc_index, int32_t *nox_index);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SGP4X_H__ */
