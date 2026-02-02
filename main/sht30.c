#include "sht30.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SHT30";

// SHT30 I2C configuration
#define SHT30_I2C_ADDR 0x44
#define SHT30_SDA_IO 16
#define SHT30_SCL_IO 17
#define SHT30_I2C_FREQ_HZ 100000

// SHT30 Commands (Single Shot, High Repeatability, Clock Stretching Disabled)
#define SHT30_CMD_MEASURE_HIGH_REP_MSB 0x24
#define SHT30_CMD_MEASURE_HIGH_REP_LSB 0x00

static i2c_master_bus_handle_t s_bus_handle = NULL;
static i2c_master_dev_handle_t s_dev_handle = NULL;

/**
 * @brief CRC-8 calculation for SHT30
 */
static uint8_t sht30_crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

esp_err_t sht30_init(void) {
  esp_err_t ret;

  // Create I2C Master Bus
  i2c_master_bus_config_t bus_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = -1, // Auto select
      .scl_io_num = SHT30_SCL_IO,
      .sda_io_num = SHT30_SDA_IO,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ret = i2c_new_master_bus(&bus_config, &s_bus_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
    return ret;
  }

  // Add SHT30 device
  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = SHT30_I2C_ADDR,
      .scl_speed_hz = SHT30_I2C_FREQ_HZ,
  };

  ret = i2c_master_bus_add_device(s_bus_handle, &dev_config, &s_dev_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SHT30 device: %s", esp_err_to_name(ret));
    i2c_del_master_bus(s_bus_handle);
    s_bus_handle = NULL;
    return ret;
  }

  // Test read to verify sensor is present
  float temp, hum;
  ret = sht30_read(&temp, &hum);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SHT30 not responding");
    i2c_master_bus_rm_device(s_dev_handle);
    i2c_del_master_bus(s_bus_handle);
    s_dev_handle = NULL;
    s_bus_handle = NULL;
    return ret;
  }

  ESP_LOGI(TAG, "SHT30 initialized (SDA=IO%d, SCL=IO%d)", SHT30_SDA_IO,
           SHT30_SCL_IO);
  ESP_LOGI(TAG, "Initial reading: %.1f°C, %.1f%%", temp, hum);
  return ESP_OK;
}

esp_err_t sht30_read(float *temperature, float *humidity) {
  if (s_dev_handle == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret;
  uint8_t cmd[2] = {SHT30_CMD_MEASURE_HIGH_REP_MSB,
                    SHT30_CMD_MEASURE_HIGH_REP_LSB};
  uint8_t data[6];

  // Send measurement command
  ret = i2c_master_transmit(s_dev_handle, cmd, sizeof(cmd), pdMS_TO_TICKS(100));
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to send measure command: %s", esp_err_to_name(ret));
    return ret;
  }

  // Wait for measurement (max 15ms for high repeatability)
  vTaskDelay(pdMS_TO_TICKS(20));

  // Read 6 bytes: 2 temp + 1 crc + 2 hum + 1 crc
  ret =
      i2c_master_receive(s_dev_handle, data, sizeof(data), pdMS_TO_TICKS(100));
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to read data: %s", esp_err_to_name(ret));
    return ret;
  }

  // Verify CRC for temperature
  if (sht30_crc8(&data[0], 2) != data[2]) {
    ESP_LOGW(TAG, "Temperature CRC mismatch");
    return ESP_ERR_INVALID_CRC;
  }

  // Verify CRC for humidity
  if (sht30_crc8(&data[3], 2) != data[5]) {
    ESP_LOGW(TAG, "Humidity CRC mismatch");
    return ESP_ERR_INVALID_CRC;
  }

  // Calculate temperature: T = -45 + 175 * (raw / 65535)
  uint16_t raw_temp = (data[0] << 8) | data[1];
  *temperature = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);

  // Calculate humidity: RH = 100 * (raw / 65535)
  uint16_t raw_hum = (data[3] << 8) | data[4];
  *humidity = 100.0f * ((float)raw_hum / 65535.0f);

  return ESP_OK;
}

esp_err_t sht30_deinit(void) {
  if (s_dev_handle != NULL) {
    i2c_master_bus_rm_device(s_dev_handle);
    s_dev_handle = NULL;
  }
  if (s_bus_handle != NULL) {
    i2c_del_master_bus(s_bus_handle);
    s_bus_handle = NULL;
  }
  ESP_LOGI(TAG, "SHT30 deinitialized");
  return ESP_OK;
}
