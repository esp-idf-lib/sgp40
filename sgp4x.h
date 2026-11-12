/**
 * @file sgp4x.h
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

#include <stdint.h>
#include <stdbool.h>
#include "sensirion_gas_index_algorithm.h"
#include "i2cdev.h"
#include <esp_err.h>

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
 * @return `ESP_ERR_INVALID_ARG` if the type is neither `SGP4x_TYPE_SGP40` nor
 *         `SGP4x_TYPE_SGP41`, or if the arguments are invalid
 * @return `ESP_ERR_NO_MEM` if the device mutex could not be created
 */
esp_err_t sgp4x_init_desc(sgp4x_t *dev, sgp4x_type_t type, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 * @return `ESP_ERR_INVALID_ARG` if the descriptor is not valid
 */
esp_err_t sgp4x_free_desc(sgp4x_t *dev);

/**
 * @brief Read the device information and initialize the gas index algorithm
 *
 * Reads the serial number and the featureset, then prepares the VOC index
 * algorithm and, for the SGP41, the NOx index algorithm. Must be called before
 * any measurement.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 * @return `ESP_ERR_NOT_SUPPORTED` if the device type is unknown
 * @return `ESP_ERR_TIMEOUT` if the device does not respond
 * @return `ESP_ERR_INVALID_CRC` if the response is corrupted
 */
esp_err_t sgp4x_init(sgp4x_t *dev);

/**
 * @brief Reset the sensor and put it back to idle mode
 *
 * The datasheets describe `0x0006` as an I2C general call that resets every
 * device on the bus. Here the command is sent to the device address, so the
 * other devices are not affected.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 * @return `ESP_ERR_TIMEOUT` if the device does not respond
 */
esp_err_t sgp4x_soft_reset(sgp4x_t *dev);

/**
 * @brief Perform a built-in self-test
 *
 * The test takes 250 ms on the SGP40 and 320 ms on the SGP41. Should not be
 * called during the conditioning on the SGP41.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` if the test passed
 * @return `ESP_FAIL` if the test failed
 * @return `ESP_ERR_NOT_SUPPORTED` if the device type is unknown
 * @return `ESP_ERR_TIMEOUT` if the device does not respond
 */
esp_err_t sgp4x_self_test(sgp4x_t *dev);

/**
 * @brief Turn the hotplate off, stop the measurement and put the device to idle mode
 *
 * On the SGP41 the conditioning must be done before resuming the measurements.
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 * @return `ESP_ERR_TIMEOUT` if the device does not respond
 */
esp_err_t sgp4x_heater_off(sgp4x_t *dev);

/**
 * @brief Start the conditioning. SGP41 only
 *
 * The conditioning is required after each restart of the sensor and after the
 * hotplate has been switched off. According to the datasheet it must run for
 * 10 seconds, and 10 seconds must not be exceeded to avoid damage to the
 * sensing material. The conditioning is stopped by calling one of the measure
 * functions.
 *
 * @param dev Device descriptor
 * @param humidity Relative humidity, percents. Use NaN if
 *                 you want uncompensated measurement
 * @param temperature Temperature, degrees Celsius. Use NaN if
 *                 you want uncompensated measurement
 * @param[out] raw_voc Raw value, proportional to the logarithm
 *                 of the resistance of the sensing element
 * @return `ESP_OK` on success
 * @return `ESP_ERR_NOT_SUPPORTED` if the device is not an SGP41
 * @return `ESP_ERR_INVALID_ARG` if the humidity or the temperature is out of
 *         the range `0..100` % or `-45..130` °C
 * @return `ESP_ERR_TIMEOUT` if the device does not respond
 */
esp_err_t sgp4x_start_conditioning(sgp4x_t *dev, float humidity, float temperature, uint16_t *raw_voc);

/**
 * @brief Perform a measurement
 *
 * Measures the raw signals. On the SGP41 both the VOC and the NOx values are
 * read in a single command.
 *
 * @param dev Device descriptor
 * @param humidity Relative humidity, percents. Use NaN if
 *                 you want uncompensated measurement
 * @param temperature Temperature, degrees Celsius. Use NaN if
 *                 you want uncompensated measurement
 * @param[out] raw_voc Raw value, proportional to the logarithm
 *                 of the resistance of the sensing element
 * @param[out] raw_nox Raw value, proportional to the logarithm
 *                 of the resistance of the sensing element. SGP41 only,
 *                 must not be NULL on the SGP41
 * @return `ESP_OK` on success
 * @return `ESP_ERR_INVALID_ARG` if the humidity or the temperature is out of
 *         the range `0..100` % or `-45..130` °C, or if a required pointer is NULL
 * @return `ESP_ERR_NOT_SUPPORTED` if the device type is unknown
 * @return `ESP_ERR_TIMEOUT` if the device does not respond
 * @return `ESP_ERR_INVALID_CRC` if the response is corrupted
 */
esp_err_t sgp4x_measure_raw(sgp4x_t *dev, float humidity, float temperature, uint16_t *raw_voc, uint16_t *raw_nox);

/**
 * @brief Perform a measurement and update the gas indices
 *
 * Measures the raw signals and feeds them to the gas index algorithm. The
 * measurement and the algorithm update are protected by the device mutex, so
 * the descriptor can be used by several tasks. The calls should be made once
 * per second: the algorithm expects a constant sampling interval.
 *
 * @param dev Device descriptor
 * @param humidity Relative humidity, percents. Use NaN if
 *                 you want uncompensated measurement
 * @param temperature Temperature, degrees Celsius. Use NaN if
 *                 you want uncompensated measurement
 * @param[out] voc_index Calculated VOC index
 * @param[out] nox_index Calculated NOx index. SGP41 only,
 *             must not be NULL on the SGP41
 * @return `ESP_OK` on success
 * @return `ESP_ERR_INVALID_ARG` if the humidity or the temperature is out of
 *         the range `0..100` % or `-45..130` °C, or if a required pointer is NULL
 * @return `ESP_ERR_NOT_SUPPORTED` if the device type is unknown
 * @return `ESP_ERR_TIMEOUT` if the device does not respond
 * @return `ESP_ERR_INVALID_CRC` if the response is corrupted
 */
esp_err_t sgp4x_measure_index(sgp4x_t *dev, float humidity, float temperature, int32_t *voc_index, int32_t *nox_index);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SGP4X_H__ */
