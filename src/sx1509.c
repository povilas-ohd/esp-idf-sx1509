#include "sx1509.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "SX1509";

// Register definitions (from SparkFun SX1509 library)
#define SX1509_REG_DATA_B           0x00 // Data register bank B (pins 15-8)
#define SX1509_REG_DATA_A           0x01 // Data register bank A (pins 7-0)
#define SX1509_REG_DIR_B            0x02 // Direction bank B (0=output, 1=input)
#define SX1509_REG_DIR_A            0x03 // Direction bank A
#define SX1509_REG_PULLUP_B         0x04 // Pull-up bank B (1=enabled)
#define SX1509_REG_PULLUP_A         0x05 // Pull-up bank A
#define SX1509_REG_INT_MASK_B       0x0C // Interrupt mask bank B (0=enabled)
#define SX1509_REG_INT_MASK_A       0x0D // Interrupt mask bank A
#define SX1509_REG_SENSE_B          0x0E // Interrupt sense bank B (edge config)
#define SX1509_REG_SENSE_A          0x0F // Interrupt sense bank A
#define SX1509_REG_INT_SRC_B        0x10 // Interrupt source bank B (1=triggered)
#define SX1509_REG_INT_SRC_A        0x11 // Interrupt source bank A
#define SX1509_REG_RESET            0x7D // Software reset (write 0x12, 0x34)

static esp_err_t write_reg(sx1509_dev_t *dev, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write reg 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t read_reg(sx1509_dev_t *dev, uint8_t reg, uint8_t *value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read reg 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t write_reg16(sx1509_dev_t *dev, uint8_t reg, uint16_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, (value >> 8) & 0xFF, true); // High byte
    i2c_master_write_byte(cmd, value & 0xFF, true);        // Low byte
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write reg 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t read_reg16(sx1509_dev_t *dev, uint8_t reg, uint16_t *value) {
    uint8_t high, low;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &high, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &low, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        *value = (high << 8) | low;
    } else {
        ESP_LOGE(TAG, "Failed to read reg 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t sx1509_init(sx1509_dev_t *dev, i2c_port_t port, uint8_t addr) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Initializing SX1509 at address 0x%02X", addr);
    dev->i2c_port = port;
    dev->i2c_addr = addr;

    // Probe device
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to probe SX1509 at 0x%02X: %s", addr, esp_err_to_name(ret));
        return ret;
    }

    // Software reset
    ret = write_reg(dev, SX1509_REG_RESET, 0x12);
    if (ret != ESP_OK) return ret;
    ret = write_reg(dev, SX1509_REG_RESET, 0x34);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));

    // Set all pins as inputs, disable pull-ups, disable interrupts
    ret = write_reg16(dev, SX1509_REG_DIR_B, 0xFFFF); // 1=input
    if (ret != ESP_OK) return ret;
    ret = write_reg16(dev, SX1509_REG_PULLUP_B, 0x0000); // 0=disabled
    if (ret != ESP_OK) return ret;
    ret = write_reg16(dev, SX1509_REG_INT_MASK_B, 0xFFFF); // 1=masked
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "SX1509 initialized");
    return ESP_OK;
}

esp_err_t sx1509_pin_mode(sx1509_dev_t *dev, sx1509_io_t io, sx1509_io_mode_t mode) {
    if (!dev || io >= SX1509_IO_COUNT) return ESP_ERR_INVALID_ARG;

    uint8_t dir_reg = io < 8 ? SX1509_REG_DIR_A : SX1509_REG_DIR_B;
    uint8_t pullup_reg = io < 8 ? SX1509_REG_PULLUP_A : SX1509_REG_PULLUP_B;
    uint8_t bit = io % 8;
    uint16_t dir_val, pullup_val;

    // Read current register values
    esp_err_t ret = read_reg16(dev, SX1509_REG_DIR_B, &dir_val);
    if (ret != ESP_OK) return ret;
    ret = read_reg16(dev, SX1509_REG_PULLUP_B, &pullup_val);
    if (ret != ESP_OK) return ret;

    // Update direction
    switch (mode) {
        case SX1509_INPUT:
        case SX1509_INPUT_PULLUP:
            dir_val |= (1 << io); // 1=input
            break;
        case SX1509_OUTPUT:
        case SX1509_ANALOG_OUTPUT:
            dir_val &= ~(1 << io); // 0=output
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    // Update pull-up
    if (mode == SX1509_INPUT_PULLUP) {
        pullup_val |= (1 << io); // 1=enabled
    } else {
        pullup_val &= ~(1 << io); // 0=disabled
    }

    // Write updated values
    ret = write_reg16(dev, SX1509_REG_DIR_B, dir_val);
    if (ret != ESP_OK) return ret;
    ret = write_reg16(dev, SX1509_REG_PULLUP_B, pullup_val);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Set IO %d to mode %d", io, mode);
    return ESP_OK;
}

esp_err_t sx1509_digital_read(sx1509_dev_t *dev, sx1509_io_t io, uint8_t *value) {
    if (!dev || io >= SX1509_IO_COUNT || !value) return ESP_ERR_INVALID_ARG;

    uint8_t reg = io < 8 ? SX1509_REG_DATA_A : SX1509_REG_DATA_B;
    uint8_t bit = io % 8;
    uint8_t data;

    esp_err_t ret = read_reg(dev, reg, &data);
    if (ret != ESP_OK) return ret;

    *value = (data >> bit) & 0x01;
    ESP_LOGI(TAG, "Read IO %d: %d", io, *value);
    return ESP_OK;
}

esp_err_t sx1509_enable_interrupt(sx1509_dev_t *dev, sx1509_io_t io, bool enable) {
    if (!dev || io >= SX1509_IO_COUNT) return ESP_ERR_INVALID_ARG;

    uint8_t mask_reg = io < 8 ? SX1509_REG_INT_MASK_A : SX1509_REG_INT_MASK_B;
    uint8_t sense_reg = io < 8 ? SX1509_REG_SENSE_A : SX1509_REG_SENSE_B;
    uint8_t bit = io % 8;
    uint16_t mask_val, sense_val;

    // Read current values
    esp_err_t ret = read_reg16(dev, SX1509_REG_INT_MASK_B, &mask_val);
    if (ret != ESP_OK) return ret;
    ret = read_reg16(dev, SX1509_REG_SENSE_B, &sense_val);
    if (ret != ESP_OK) return ret;

    // Update interrupt mask (0=enabled, 1=masked)
    if (enable) {
        mask_val &= ~(1 << io); // Unmask
    } else {
        mask_val |= (1 << io); // Mask
    }

    // Set falling edge (0b10) for this pin
    uint8_t sense_bit = (io % 8) * 2;
    sense_val &= ~(0x3 << sense_bit); // Clear bits
    if (enable) {
        sense_val |= (0x2 << sense_bit); // Falling edge
    }

    // Write updated values
    ret = write_reg16(dev, SX1509_REG_INT_MASK_B, mask_val);
    if (ret != ESP_OK) return ret;
    ret = write_reg16(dev, SX1509_REG_SENSE_B, sense_val);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "%s interrupt on IO %d", enable ? "Enabled" : "Disabled", io);
    return ESP_OK;
}

esp_err_t sx1509_interrupt_source(sx1509_dev_t *dev, uint16_t *sources) {
    if (!dev || !sources) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = read_reg16(dev, SX1509_REG_INT_SRC_B, sources);
    if (ret != ESP_OK) return ret;

    // Clear interrupt by writing back
    ret = write_reg16(dev, SX1509_REG_INT_SRC_B, *sources);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Interrupt sources: 0x%04X", *sources);
    return ESP_OK;
}
