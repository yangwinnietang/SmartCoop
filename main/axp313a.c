#include "axp313a.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "AXP313A";

// AXP313A I2C address
#define AXP313A_ADDR 0x36

// I2C configuration for Romeo ESP32-S3
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 1
#define I2C_MASTER_SCL_IO 2
#define I2C_MASTER_FREQ_HZ 100000

// AXP313A Register addresses
#define AXP313A_OUTPUT_CTRL 0x10   // Output control register
#define AXP313A_ALDO1_VOLTAGE 0x16 // ALDO1 voltage setting

// Output control bits
#define AXP313A_ALDO1_EN (1 << 0)

static bool i2c_initialized = false;

static esp_err_t i2c_master_init(void) {
  if (i2c_initialized) {
    return ESP_OK;
  }

  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };

  esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
    return err;
  }

  err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    return err;
  }

  i2c_initialized = true;
  ESP_LOGI(TAG, "I2C master initialized (SDA=%d, SCL=%d)", I2C_MASTER_SDA_IO,
           I2C_MASTER_SCL_IO);
  return ESP_OK;
}

static esp_err_t axp313a_write_reg(uint8_t reg, uint8_t value) {
  uint8_t write_buf[2] = {reg, value};
  return i2c_master_write_to_device(I2C_MASTER_NUM, AXP313A_ADDR, write_buf,
                                    sizeof(write_buf), pdMS_TO_TICKS(100));
}

static esp_err_t axp313a_read_reg(uint8_t reg, uint8_t *value) {
  return i2c_master_write_read_device(I2C_MASTER_NUM, AXP313A_ADDR, &reg, 1,
                                      value, 1, pdMS_TO_TICKS(100));
}

esp_err_t axp313a_init(void) {
  esp_err_t err = i2c_master_init();
  if (err != ESP_OK) {
    return err;
  }

  // Verify AXP313A is present by reading a register
  uint8_t reg_val = 0;
  err = axp313a_read_reg(AXP313A_OUTPUT_CTRL, &reg_val);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to communicate with AXP313A: %s",
             esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(TAG, "AXP313A initialized, output control reg: 0x%02X", reg_val);
  return ESP_OK;
}

esp_err_t axp313a_camera_power_on(void) {
  esp_err_t err;
  uint8_t reg_val = 0;

  // Set ALDO1 voltage to 2.8V (camera AVDD)
  // Voltage = 0.5V + (reg_val * 0.1V), so 2.8V = 0x17 (23 * 0.1 + 0.5 = 2.8)
  err = axp313a_write_reg(AXP313A_ALDO1_VOLTAGE, 0x17);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set ALDO1 voltage: %s", esp_err_to_name(err));
    return err;
  }

  // Read current output control register
  err = axp313a_read_reg(AXP313A_OUTPUT_CTRL, &reg_val);
  if (err != ESP_OK) {
    return err;
  }

  // Enable ALDO1 output
  reg_val |= AXP313A_ALDO1_EN;
  err = axp313a_write_reg(AXP313A_OUTPUT_CTRL, reg_val);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable ALDO1: %s", esp_err_to_name(err));
    return err;
  }

  // Wait for power to stabilize
  vTaskDelay(pdMS_TO_TICKS(100));

  ESP_LOGI(TAG, "Camera power enabled (ALDO1 = 2.8V)");
  return ESP_OK;
}

esp_err_t axp313a_camera_power_off(void) {
  uint8_t reg_val = 0;
  esp_err_t err = axp313a_read_reg(AXP313A_OUTPUT_CTRL, &reg_val);
  if (err != ESP_OK) {
    return err;
  }

  // Disable ALDO1 output
  reg_val &= ~AXP313A_ALDO1_EN;
  err = axp313a_write_reg(AXP313A_OUTPUT_CTRL, reg_val);
  if (err != ESP_OK) {
    return err;
  }

  ESP_LOGI(TAG, "Camera power disabled");
  return ESP_OK;
}
