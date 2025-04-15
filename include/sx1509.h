#ifndef SX1509_H
#define SX1509_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

/**
 * @brief Default I2C address for SX1509 (ADDR0/ADDR1 tied low).
 */
#define SX1509_DEFAULT_ADDRESS 0x3E

/**
 * @brief Enumeration of SX1509 I/O pins.
 *
 * Defines the 16 available I/O pins on the SX1509, numbered from 0 to 15.
 */
typedef enum {
    SX1509_IO_0 = 0,   /**< I/O Pin 0 */
    SX1509_IO_1,       /**< I/O Pin 1 */
    SX1509_IO_2,       /**< I/O Pin 2 */
    SX1509_IO_3,       /**< I/O Pin 3 */
    SX1509_IO_4,       /**< I/O Pin 4 */
    SX1509_IO_5,       /**< I/O Pin 5 */
    SX1509_IO_6,       /**< I/O Pin 6 */
    SX1509_IO_7,       /**< I/O Pin 7 */
    SX1509_IO_8,       /**< I/O Pin 8 */
    SX1509_IO_9,       /**< I/O Pin 9 */
    SX1509_IO_10,      /**< I/O Pin 10 */
    SX1509_IO_11,      /**< I/O Pin 11 */
    SX1509_IO_12,      /**< I/O Pin 12 */
    SX1509_IO_13,      /**< I/O Pin 13 */
    SX1509_IO_14,      /**< I/O Pin 14 */
    SX1509_IO_15,      /**< I/O Pin 15 */
    SX1509_IO_COUNT    /**< Total number of pins (16) */
} sx1509_io_t;

/**
 * @brief SX1509 I/O mode configuration.
 */
typedef enum {
    SX1509_INPUT = 0,           /**< Input mode */
    SX1509_INPUT_PULLUP,        /**< Input with internal pull-up resistor */
    SX1509_OUTPUT,              /**< Output mode (push-pull) */
    SX1509_ANALOG_OUTPUT        /**< PWM output mode */
} sx1509_io_mode_t;

/**
 * @brief SX1509 device configuration structure.
 *
 * Holds the configuration parameters for an SX1509 device, including the I2C port and address.
 */
typedef struct {
    i2c_port_t i2c_port;    /**< I2C port number (e.g., I2C_NUM_0 or I2C_NUM_1) */
    uint8_t i2c_addr;       /**< I2C address of the SX1509 (e.g., 0x3E to 0x71) */
} sx1509_dev_t;

/**
 * @brief Initialize an SX1509 device.
 *
 * Configures the SX1509 for operation on the specified I2C port and address.
 * Assumes the I2C driver is already initialized externally (e.g., by the application).
 * Resets the device to a known state, with all I/O pins set as inputs and interrupts disabled.
 *
 * @param dev Pointer to the SX1509 device configuration structure.
 * @param port I2C port number (e.g., I2C_NUM_0 or I2C_NUM_1).
 * @param addr I2C address of the SX1509 (typically 0x3E to 0x71).
 * @return
 *     - ESP_OK on success.
 *     - ESP_ERR_INVALID_ARG if dev is NULL.
 *     - ESP_FAIL or other errors if I2C communication fails.
 */
esp_err_t sx1509_init(sx1509_dev_t *dev, i2c_port_t port, uint8_t addr);

/**
 * @brief Configure the mode of an SX1509 I/O pin.
 *
 * Sets the specified pin as input, input with pull-up, output, or analog (PWM) output.
 * For inputs, pull-ups can be enabled to simplify button connections.
 *
 * @param dev Pointer to the initialized SX1509 device configuration structure.
 * @param io I/O pin to configure (SX1509_IO_0 to SX1509_IO_15).
 * @param mode Mode to set (e.g., SX1509_INPUT, SX1509_INPUT_PULLUP, SX1509_OUTPUT).
 * @return
 *     - ESP_OK on success.
 *     - ESP_ERR_INVALID_ARG if dev is NULL or io/mode is invalid.
 *     - ESP_FAIL or other errors if I2C communication fails.
 */
esp_err_t sx1509_pin_mode(sx1509_dev_t *dev, sx1509_io_t io, sx1509_io_mode_t mode);

/**
 * @brief Read the digital state of an SX1509 input pin.
 *
 * Reads the current logic level of the specified pin (0 for low, 1 for high).
 * The pin must be configured as an input (SX1509_INPUT or SX1509_INPUT_PULLUP).
 *
 * @param dev Pointer to the initialized SX1509 device configuration structure.
 * @param io I/O pin to read (SX1509_IO_0 to SX1509_IO_15).
 * @param value Pointer to store the read value (0 or 1).
 * @return
 *     - ESP_OK on success.
 *     - ESP_ERR_INVALID_ARG if dev or value is NULL or io is invalid.
 *     - ESP_FAIL or other errors if I2C communication fails.
 */
esp_err_t sx1509_digital_read(sx1509_dev_t *dev, sx1509_io_t io, uint8_t *value);

/**
 * @brief Enable or disable interrupt on an SX1509 input pin.
 *
 * Configures the specified pin to trigger an interrupt on falling edge (high to low).
 * The SX1509's INT pin must be connected to a microcontroller GPIO to detect interrupts.
 *
 * @param dev Pointer to the initialized SX1509 device configuration structure.
 * @param io I/O pin to configure (SX1509_IO_0 to SX1509_IO_15).
 * @param enable True to enable interrupt, false to disable.
 * @return
 *     - ESP_OK on success.
 *     - ESP_ERR_INVALID_ARG if dev is NULL or io is invalid.
 *     - ESP_FAIL or other errors if I2C communication fails.
 */
esp_err_t sx1509_enable_interrupt(sx1509_dev_t *dev, sx1509_io_t io, bool enable);

/**
 * @brief Read and clear the interrupt source.
 *
 * Identifies which pin(s) triggered an interrupt and clears the interrupt flag.
 * Returns a bitmask where each bit corresponds to an I/O pin (1 if interrupted).
 *
 * @param dev Pointer to the initialized SX1509 device configuration structure.
 * @param sources Pointer to store the interrupt source bitmask (16 bits).
 * @return
 *     - ESP_OK on success.
 *     - ESP_ERR_INVALID_ARG if dev or sources is NULL.
 *     - ESP_FAIL or other errors if I2C communication fails.
 */
esp_err_t sx1509_interrupt_source(sx1509_dev_t *dev, uint16_t *sources);

#endif // SX1509_H
