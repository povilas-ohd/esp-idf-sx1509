#pragma once

#include <esp_err.h>
#include <driver/i2c.h>
#include <driver/gpio.h>

/**
 * @file sx1509.h
 * @brief Header file for SX1509 I/O Expander driver for ESP-IDF.
 *
 * This library provides functions to interface with the SX1509 I/O expander
 * over I2C, supporting digital I/O, interrupts, debouncing, and PWM output.
 */

/**
 * @defgroup SX1509_Registers SX1509 Register Definitions
 * @{
 */
#define REG_INPUT_DISABLE_B     0x00 ///< Input buffer disable register (Bank B)
#define REG_INPUT_DISABLE_A     0x01 ///< Input buffer disable register (Bank A)
#define REG_LONG_SLEW_B         0x02 ///< Output slew rate control (Bank B)
#define REG_LONG_SLEW_A         0x03 ///< Output slew rate control (Bank A)
#define REG_LOW_DRIVE_B         0x04 ///< Low drive strength control (Bank B)
#define REG_LOW_DRIVE_A         0x05 ///< Low drive strength control (Bank A)
#define REG_PULLUP_B            0x06 ///< Pull-up resistor enable (Bank B)
#define REG_PULLUP_A            0x07 ///< Pull-up resistor enable (Bank A)
#define REG_PULLDOWN_B          0x08 ///< Pull-down resistor enable (Bank B)
#define REG_PULLDOWN_A          0x09 ///< Pull-down resistor enable (Bank A)
#define REG_OPEN_DRAIN_B        0x0A ///< Open-drain enable (Bank B)
#define REG_OPEN_DRAIN_A        0x0B ///< Open-drain enable (Bank A)
#define REG_POLARITY_B          0x0C ///< Input polarity (Bank B)
#define REG_POLARITY_A          0x0D ///< Input polarity (Bank A)
#define REG_DIR_B               0x0E ///< Direction (Bank B: 1=input, 0=output)
#define REG_DIR_A               0x0F ///< Direction (Bank A: 1=input, 0=output)
#define REG_DATA_B              0x10 ///< Data register (Bank B)
#define REG_DATA_A              0x11 ///< Data register (Bank A)
#define REG_INTERRUPT_MASK_B    0x12 ///< Interrupt mask (Bank B: 0=enabled)
#define REG_INTERRUPT_MASK_A    0x13 ///< Interrupt mask (Bank A: 0=enabled)
#define REG_SENSE_HIGH_B        0x14 ///< Sense high bits (Bank B)
#define REG_SENSE_LOW_B         0x15 ///< Sense low bits (Bank B)
#define REG_SENSE_HIGH_A        0x16 ///< Sense high bits (Bank A)
#define REG_SENSE_LOW_A         0x17 ///< Sense low bits (Bank A)
#define REG_INTERRUPT_SOURCE_B  0x18 ///< Interrupt source (Bank B)
#define REG_INTERRUPT_SOURCE_A  0x19 ///< Interrupt source (Bank A)
#define REG_EVENT_STATUS_B      0x1A ///< Event status (Bank B)
#define REG_EVENT_STATUS_A      0x1B ///< Event status (Bank A)
#define REG_LEVEL_SHIFTER_1     0x1C ///< Level shifter 1 config
#define REG_LEVEL_SHIFTER_2     0x1D ///< Level shifter 2 config
#define REG_CLOCK               0x1E ///< Clock control
#define REG_MISC                0x1F ///< Miscellaneous control
#define REG_LED_DRIVER_ENABLE_B 0x20 ///< LED driver enable (Bank B)
#define REG_LED_DRIVER_ENABLE_A 0x21 ///< LED driver enable (Bank A)
#define REG_DEBOUNCE_CONFIG_A   0x22 ///< Debounce config (Bank A)
#define REG_DEBOUNCE_CONFIG_B   0x23 ///< Debounce config (Bank B)
#define REG_RESET               0x7D ///< Software reset
#define REG_TEST_1              0x7E ///< Test register 1
#define REG_TEST_2              0x7F ///< Test register 2
/** @} */

/**
 * @brief Pin mode configuration options for SX1509 pins.
 */
typedef enum {
    SX1509_INPUT = 0,      ///< Pin configured as input
    SX1509_OUTPUT,         ///< Pin configured as output
    SX1509_INPUT_PULLUP,   ///< Pin configured as input with pull-up
    SX1509_ANALOG_OUTPUT   ///< Pin configured for PWM output
} sx1509_pin_mode_t;

/**
 * @brief Interrupt mode configuration options for SX1509 pins.
 */
typedef enum {
    SX1509_RISING = 0,  ///< Interrupt on rising edge
    SX1509_FALLING = 1, ///< Interrupt on falling edge
    SX1509_CHANGE = 2   ///< Interrupt on both edges
} sx1509_interrupt_mode_t;

/**
 * @brief SX1509 device configuration structure.
 */
typedef struct {
    i2c_port_t i2c_port;       ///< I2C port number
    uint8_t i2c_addr;          ///< I2C device address
    gpio_num_t int_pin;        ///< Interrupt pin (GPIO number)
    uint16_t debounce_time;    ///< Debounce time in milliseconds
    uint16_t led_clock_div;    ///< LED clock divider for PWM
} sx1509_t;

/**
 * @brief Initialize the SX1509 device.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param i2c_port I2C port number.
 * @param i2c_addr I2C address of the SX1509.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_init(sx1509_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr);

/**
 * @brief Configure multiple pins as inputs with pull-ups and interrupts.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param pin_mask Bitmap of pins to configure (1=configure, 0=ignore).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_configure_input_pins(sx1509_t *dev, uint8_t pin_mask);

/**
 * @brief Configure the mode of a specific pin.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param pin Pin number (0-15).
 * @param mode Pin mode (input, output, etc.).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_pin_mode(sx1509_t *dev, uint8_t pin, sx1509_pin_mode_t mode);

/**
 * @brief Write a digital value to a pin.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param pin Pin number (0-15).
 * @param value Value to write (0 or 1).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_digital_write(sx1509_t *dev, uint8_t pin, uint8_t value);

/**
 * @brief Read the digital value of a pin.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param pin Pin number (0-15).
 * @param value Pointer to store the read value (0 or 1).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_digital_read(sx1509_t *dev, uint8_t pin, uint8_t *value);

/**
 * @brief Enable interrupt on a specific pin.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param pin Pin number (0-15).
 * @param mode Interrupt mode (rising, falling, or change).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_enable_interrupt(sx1509_t *dev, uint8_t pin, sx1509_interrupt_mode_t mode);

/**
 * @brief Disable interrupt on a specific pin.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param pin Pin number (0-15).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_disable_interrupt(sx1509_t *dev, uint8_t pin);

/**
 * @brief Clear all pending interrupts.
 *
 * @param dev Pointer to SX1509 device structure.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_clear_interrupt(sx1509_t *dev);

/**
 * @brief Get the interrupt source bitmap.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param source Pointer to store the 16-bit interrupt source bitmap.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_get_interrupt_source(sx1509_t *dev, uint16_t *source);

/**
 * @brief Set the debounce time for input pins.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param time_ms Debounce time in milliseconds (0.5 to 64 ms).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_set_debounce_time(sx1509_t *dev, uint16_t time_ms);

/**
 * @brief Enable debouncing for a specific pin.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param pin Pin number (0-15).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_debounce_enable(sx1509_t *dev, uint8_t pin);

/**
 * @brief Disable debouncing for a specific pin.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param pin Pin number (0-15).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_debounce_disable(sx1509_t *dev, uint8_t pin);

/**
 * @brief Configure PWM output for a pin.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param pin Pin number (0-15).
 * @param freq_div Frequency divider for PWM.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_pwm_config(sx1509_t *dev, uint8_t pin, uint8_t freq_div);

/**
 * @brief Write a PWM value to a pin.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param pin Pin number (0-15).
 * @param value PWM duty cycle (0-255).
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_pwm_write(sx1509_t *dev, uint8_t pin, uint8_t value);

/**
 * @brief Read the state of all pins in Bank A.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param states Pointer to store the 8-bit pin state bitmap.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t sx1509_get_pin_states(sx1509_t *dev, uint8_t *states);