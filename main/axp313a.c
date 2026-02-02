#include "axp313a.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AXP313A";

// AXP313A I2C address
#define AXP313A_ADDR 0x36

// I2C configuration for Romeo ESP32-S3
// Must match the pins used by the camera to avoid conflict,
// but we will init/deinit on demand to share the bus.
#define I2C_MASTER_SDA_IO 1
#define I2C_MASTER_SCL_IO 2
#define I2C_MASTER_FREQ_HZ 100000

// AXP313A Register addresses
#define AXP313A_OUTPUT_CTRL 0x10   // Output control register
#define AXP313A_ALDO1_VOLTAGE 0x16 // ALDO1 voltage setting

// Output control bits
#define AXP313A_ALDO1_EN (1 << 0)

// Helper to perform transient I2C operation
// We init bus, perfom op, then deinit bus to avoid conflict with Camera driver
static esp_err_t axp313a_perform_op(uint8_t reg, uint8_t *val, bool write) {
  esp_err_t ret;

  // 1. Create I2C Master Bus
  i2c_master_bus_config_t i2c_bus_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = -1, // Auto select
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  i2c_master_bus_handle_t bus_handle;
  ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
    return ret;
  }

  // 2. Add AXP313A Device
  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = AXP313A_ADDR,
      .scl_speed_hz = I2C_MASTER_FREQ_HZ,
  };

  i2c_master_dev_handle_t dev_handle;
  ret = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
  if (ret != ESP_OK) {
    i2c_del_master_bus(bus_handle);
    return ret;
  }

  // 3. Perform Operation
  if (write) {
    uint8_t write_buf[2] = {reg, *val};
    ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf),
                              pdMS_TO_TICKS(100));
  } else {
    // Read
    ret = i2c_master_transmit_receive(dev_handle, &reg, 1, val, 1,
                                      pdMS_TO_TICKS(100));
  }

  // 4. Teardown
  i2c_master_bus_rm_device(dev_handle);
  i2c_del_master_bus(bus_handle);

  return ret;
}

static esp_err_t axp313a_write_reg(uint8_t reg, uint8_t value) {
  return axp313a_perform_op(reg, &value, true);
}

static esp_err_t axp313a_read_reg(uint8_t reg, uint8_t *value) {
  return axp313a_perform_op(reg, value, false);
}

esp_err_t axp313a_init(void) {
  // Verify AXP313A is present by reading a register
  uint8_t reg_val = 0;
  esp_err_t err = axp313a_read_reg(AXP313A_OUTPUT_CTRL, &reg_val);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to communicate with AXP313A: %s",
             esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(TAG, "AXP313A initialized (transient), output control reg: 0x%02X",
           reg_val);
  return ESP_OK;
}

esp_err_t axp313a_camera_power_on(void) {
  esp_err_t err;

  // Set ALDO1 voltage to 2.8V (camera AVDD)
  // Voltage = 0.5V + (reg_val * 0.1V), so 2.8V = 0x17 (23 * 0.1 + 0.5 = 2.8)
  err = axp313a_write_reg(AXP313A_ALDO1_VOLTAGE, 0x17);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set ALDO1 voltage: %s", esp_err_to_name(err));
    return err;
  }

  // Enable ALDO1 output
  // We need read-modify-write, but to be safe and simple with transient bus,
  // we can just read then write in separate transactions.
  uint8_t reg_val = 0;
  err = axp313a_read_reg(AXP313A_OUTPUT_CTRL, &reg_val);
  if (err != ESP_OK)
    return err;

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
