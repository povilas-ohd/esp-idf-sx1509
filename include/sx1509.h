#pragma once

#include <esp_err.h>
#include <driver/i2c.h>
#include <driver/gpio.h>

// Register definitions
#define REG_INPUT_DISABLE_B    0x00
#define REG_INPUT_DISABLE_A    0x01
#define REG_LONG_SLEW_B       0x02
#define REG_LONG_SLEW_A       0x03
#define REG_LOW_DRIVE_B       0x04
#define REG_LOW_DRIVE_A       0x05
#define REG_PULLUP_B          0x06
#define REG_PULLUP_A          0x07
#define REG_PULLDOWN_B        0x08
#define REG_PULLDOWN_A        0x09
#define REG_OPEN_DRAIN_B      0x0A
#define REG_OPEN_DRAIN_A      0x0B
#define REG_POLARITY_B        0x0C
#define REG_POLARITY_A        0x0D
#define REG_DIR_B             0x0E
#define REG_DIR_A             0x0F
#define REG_DATA_B            0x10
#define REG_DATA_A            0x11
#define REG_INTERRUPT_MASK_B  0x12
#define REG_INTERRUPT_MASK_A  0x13
#define REG_SENSE_HIGH_B      0x14
#define REG_SENSE_LOW_B       0x15
#define REG_SENSE_HIGH_A      0x16
#define REG_SENSE_LOW_A       0x17
#define REG_INTERRUPT_SOURCE_B 0x18
#define REG_INTERRUPT_SOURCE_A 0x19
#define REG_EVENT_STATUS_B    0x1A
#define REG_EVENT_STATUS_A    0x1B
#define REG_LEVEL_SHIFTER_1   0x1C
#define REG_LEVEL_SHIFTER_2   0x1D
#define REG_CLOCK             0x1E
#define REG_MISC              0x1F
#define REG_LED_DRIVER_ENABLE_B 0x20
#define REG_LED_DRIVER_ENABLE_A 0x21
#define REG_RESET             0x7D
#define REG_TEST_1            0x7E
#define REG_TEST_2            0x7F

// Pin definitions
#define SX1509_PIN_0  0
#define SX1509_PIN_1  1
#define SX1509_PIN_2  2
#define SX1509_PIN_3  3
#define SX1509_PIN_4  4
#define SX1509_PIN_5  5
#define SX1509_PIN_6  6
#define SX1509_PIN_7  7
#define SX1509_PIN_8  8
#define SX1509_PIN_9  9
#define SX1509_PIN_10 10
#define SX1509_PIN_11 11
#define SX1509_PIN_12 12
#define SX1509_PIN_13 13
#define SX1509_PIN_14 14
#define SX1509_PIN_15 15

typedef enum {
    SX1509_INPUT = 0,
    SX1509_OUTPUT,
    SX1509_INPUT_PULLUP,
    SX1509_ANALOG_OUTPUT
} sx1509_pin_mode_t;

// Additional register definitions
#define REG_DEBOUNCE_CONFIG_A  0x22
#define REG_DEBOUNCE_CONFIG_B  0x23

typedef enum {
    SX1509_RISING = 0,
    SX1509_FALLING = 1,
    SX1509_CHANGE = 2
} sx1509_interrupt_mode_t;


typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    gpio_num_t int_pin;
    uint16_t debounce_time;
    uint16_t led_clock_div;
} sx1509_t;

esp_err_t sx1509_init(sx1509_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr);
esp_err_t sx1509_pin_mode(sx1509_t *dev, uint8_t pin, sx1509_pin_mode_t mode);
esp_err_t sx1509_digital_write(sx1509_t *dev, uint8_t pin, uint8_t value);
esp_err_t sx1509_digital_read(sx1509_t *dev, uint8_t pin, uint8_t *value);
esp_err_t sx1509_enable_interrupt(sx1509_t *dev, uint8_t pin, sx1509_interrupt_mode_t mode);
esp_err_t sx1509_disable_interrupt(sx1509_t *dev, uint8_t pin);
esp_err_t sx1509_clear_interrupt(sx1509_t *dev);
esp_err_t sx1509_get_interrupt_source(sx1509_t *dev, uint16_t *source);
esp_err_t sx1509_set_debounce_time(sx1509_t *dev, uint16_t time_ms);
esp_err_t sx1509_debounce_enable(sx1509_t *dev, uint8_t pin);
esp_err_t sx1509_debounce_disable(sx1509_t *dev, uint8_t pin);
esp_err_t sx1509_pwm_config(sx1509_t *dev, uint8_t pin, uint8_t freq_div);
esp_err_t sx1509_pwm_write(sx1509_t *dev, uint8_t pin, uint8_t value);
