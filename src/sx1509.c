#include "sx1509.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @file sx1509.c
 * @brief Implementation of SX1509 I/O Expander driver for ESP-IDF.
 *
 * This file contains the functions to control the SX1509 I/O expander,
 * including initialization, pin configuration, digital I/O, interrupts,
 * and debouncing.
 */

static const char *TAG = "SX1509"; ///< Logging tag for SX1509 driver

/**
 * @brief Write a single byte to an SX1509 register.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param reg Register address to write to.
 * @param value Value to write.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
static esp_err_t write_register(sx1509_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, write_buf, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Read a single byte from an SX1509 register.
 *
 * @param dev Pointer to SX1509 device structure.
 * @param reg Register address to read from.
 * @param value Pointer to store the read value.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
static esp_err_t read_register(sx1509_t *dev, uint8_t reg, uint8_t *value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t sx1509_init(sx1509_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr)
{
    dev->i2c_port = i2c_port;
    dev->i2c_addr = i2c_addr;
    
    // Software Reset
    esp_err_t ret = write_register(dev, REG_RESET, 0x12);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));
    ret = write_register(dev, REG_RESET, 0x34);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Default configuration
    ret = write_register(dev, REG_CLOCK, 0x40); // Internal 2MHz oscillator
    if (ret != ESP_OK) return ret;
    
    return write_register(dev, REG_MISC, 0x00); // Normal operation
}

esp_err_t sx1509_configure_input_pins(sx1509_t *dev, uint8_t pin_mask)
{
    esp_err_t ret;

    // Set direction (1 = input)
    ret = write_register(dev, REG_DIR_A, pin_mask);
    if (ret != ESP_OK) return ret;

    // Enable pullup
    ret = write_register(dev, REG_PULLUP_A, pin_mask);
    if (ret != ESP_OK) return ret;

    // Configure debounce
    ret = write_register(dev, REG_DEBOUNCE_CONFIG_A, pin_mask);
    if (ret != ESP_OK) return ret;

    // Enable interrupts (clear mask)
    ret = write_register(dev, REG_INTERRUPT_MASK_A, 0x00);
    if (ret != ESP_OK) return ret;

    // Configure for falling edge detection
    ret = write_register(dev, REG_SENSE_HIGH_A, 0x00);
    if (ret != ESP_OK) return ret;
    
    ret = write_register(dev, REG_SENSE_LOW_A, pin_mask);
    if (ret != ESP_OK) return ret;

    // Clear any pending interrupts
    return sx1509_clear_interrupt(dev);
}

esp_err_t sx1509_pin_mode(sx1509_t *dev, uint8_t pin, sx1509_pin_mode_t mode)
{
    ESP_LOGI(TAG, "Setting pin %d to mode %d", pin, mode);
    
    uint8_t reg_dir = (pin < 8) ? REG_DIR_A : REG_DIR_B;
    uint8_t reg_pullup = (pin < 8) ? REG_PULLUP_A : REG_PULLUP_B;
    uint8_t pin_bit = pin % 8;
    uint8_t dir_val, pullup_val;
    
    esp_err_t ret = read_register(dev, reg_dir, &dir_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read direction register");
        return ret;
    }
    ESP_LOGI(TAG, "Current direction register: 0x%02X", dir_val);
    
    ret = read_register(dev, reg_pullup, &pullup_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read pullup register");
        return ret;
    }
    ESP_LOGI(TAG, "Current pullup register: 0x%02X", pullup_val);
    
    switch (mode) {
        case SX1509_INPUT:
            dir_val |= (1 << pin_bit);
            pullup_val &= ~(1 << pin_bit);
            break;
        case SX1509_OUTPUT:
            dir_val &= ~(1 << pin_bit);
            pullup_val &= ~(1 << pin_bit);
            break;
        case SX1509_INPUT_PULLUP:
            dir_val |= (1 << pin_bit);
            pullup_val |= (1 << pin_bit);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Setting direction register to: 0x%02X", dir_val);
    ret = write_register(dev, reg_dir, dir_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write direction register");
        return ret;
    }
    
    ESP_LOGI(TAG, "Setting pullup register to: 0x%02X", pullup_val);
    return write_register(dev, reg_pullup, pullup_val);
}

esp_err_t sx1509_digital_write(sx1509_t *dev, uint8_t pin, uint8_t value)
{
    uint8_t reg_data = (pin < 8) ? REG_DATA_A : REG_DATA_B;
    uint8_t pin_bit = pin % 8;
    uint8_t data_val;

    esp_err_t ret = read_register(dev, reg_data, &data_val);
    if (ret != ESP_OK) return ret;

    if (value) {
        data_val |= (1 << pin_bit);
    } else {
        data_val &= ~(1 << pin_bit);
    }

    return write_register(dev, reg_data, data_val);
}

esp_err_t sx1509_digital_read(sx1509_t *dev, uint8_t pin, uint8_t *value)
{
    uint8_t reg = (pin < 8) ? REG_DATA_A : REG_DATA_B;
    uint8_t pin_bit = pin % 8;
    uint8_t data;
    
    esp_err_t ret = read_register(dev, reg, &data);
    if (ret == ESP_OK) {
        *value = (data & (1 << pin_bit)) ? 1 : 0;
        ESP_LOGD(TAG, "Pin %d state: %d (register value: 0x%02X)", pin, *value, data);
    }
    
    return ret;
}

esp_err_t sx1509_enable_interrupt(sx1509_t *dev, uint8_t pin, sx1509_interrupt_mode_t mode)
{
    ESP_LOGI(TAG, "Enabling interrupt for pin %d, mode %d", pin, mode);
    
    uint8_t reg_sense_high = (pin < 8) ? REG_SENSE_HIGH_A : REG_SENSE_HIGH_B;
    uint8_t reg_sense_low = (pin < 8) ? REG_SENSE_LOW_A : REG_SENSE_LOW_B;
    uint8_t reg_mask = (pin < 8) ? REG_INTERRUPT_MASK_A : REG_INTERRUPT_MASK_B;
    uint8_t pin_bit = pin % 8;
    
    esp_err_t ret = write_register(dev, reg_sense_high, 0x00);
    if (ret != ESP_OK) return ret;
    
    ret = write_register(dev, reg_sense_low, (1 << pin_bit));
    if (ret != ESP_OK) return ret;
    
    uint8_t mask_val;
    ret = read_register(dev, reg_mask, &mask_val);
    if (ret != ESP_OK) return ret;
    
    mask_val &= ~(1 << pin_bit);
    ret = write_register(dev, reg_mask, mask_val);
    if (ret != ESP_OK) return ret;
    
    return sx1509_clear_interrupt(dev);
}

esp_err_t sx1509_disable_interrupt(sx1509_t *dev, uint8_t pin)
{
    uint8_t reg_mask = (pin < 8) ? REG_INTERRUPT_MASK_A : REG_INTERRUPT_MASK_B;
    uint8_t pin_bit = pin % 8;
    uint8_t mask_val;

    esp_err_t ret = read_register(dev, reg_mask, &mask_val);
    if (ret != ESP_OK) return ret;

    mask_val |= (1 << pin_bit);
    return write_register(dev, reg_mask, mask_val);
}

esp_err_t sx1509_clear_interrupt(sx1509_t *dev)
{
    esp_err_t ret = write_register(dev, REG_INTERRUPT_SOURCE_A, 0xFF);
    if (ret != ESP_OK) return ret;
    
    return write_register(dev, REG_INTERRUPT_SOURCE_B, 0xFF);
}

esp_err_t sx1509_get_interrupt_source(sx1509_t *dev, uint16_t *source)
{
    uint8_t src_a, src_b;
    
    esp_err_t ret = read_register(dev, REG_INTERRUPT_SOURCE_A, &src_a);
    if (ret != ESP_OK) return ret;
    
    ret = read_register(dev, REG_INTERRUPT_SOURCE_B, &src_b);
    if (ret != ESP_OK) return ret;
    
    *source = ((uint16_t)src_b << 8) | src_a;
    return ESP_OK;
}

esp_err_t sx1509_set_debounce_time(sx1509_t *dev, uint16_t time_ms)
{
    uint8_t debounce_value;
    
    if (time_ms <= 0.5) debounce_value = 0;
    else if (time_ms <= 1) debounce_value = 1;
    else if (time_ms <= 2) debounce_value = 2;
    else if (time_ms <= 4) debounce_value = 3;
    else if (time_ms <= 8) debounce_value = 4;
    else if (time_ms <= 16) debounce_value = 5;
    else if (time_ms <= 32) debounce_value = 6;
    else debounce_value = 7;
    
    return write_register(dev, REG_CLOCK, 0x40 | (debounce_value << 4));
}

esp_err_t sx1509_debounce_enable(sx1509_t *dev, uint8_t pin)
{
    uint8_t reg = (pin < 8) ? REG_DEBOUNCE_CONFIG_A : REG_DEBOUNCE_CONFIG_B;
    uint8_t pin_bit = pin % 8;
    uint8_t debounce_val;
    
    esp_err_t ret = read_register(dev, reg, &debounce_val);
    if (ret != ESP_OK) return ret;
    
    debounce_val |= (1 << pin_bit);
    
    return write_register(dev, reg, debounce_val);
}

esp_err_t sx1509_debounce_disable(sx1509_t *dev, uint8_t pin)
{
    uint8_t reg = (pin < 8) ? REG_DEBOUNCE_CONFIG_A : REG_DEBOUNCE_CONFIG_B;
    uint8_t pin_bit = pin % 8;
    uint8_t debounce_val;

    esp_err_t ret = read_register(dev, reg, &debounce_val);
    if (ret != ESP_OK) return ret;

    debounce_val &= ~(1 << pin_bit);
    return write_register(dev, reg, debounce_val);
}

esp_err_t sx1509_pwm_config(sx1509_t *dev, uint8_t pin, uint8_t freq_div)
{
    // Not implemented
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t sx1509_pwm_write(sx1509_t *dev, uint8_t pin, uint8_t value)
{
    // Not implemented
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t sx1509_get_pin_states(sx1509_t *dev, uint8_t *states)
{
    return read_register(dev, REG_DATA_A, states);
}