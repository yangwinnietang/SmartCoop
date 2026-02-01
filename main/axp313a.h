#ifndef AXP313A_H
#define AXP313A_H

#include "esp_err.h"

/**
 * @brief Initialize AXP313A power management IC
 *
 * This configures the I2C bus and enables power outputs
 * required for the camera module on DFRobot Romeo/FireBeetle ESP32-S3.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t axp313a_init(void);

/**
 * @brief Enable camera power via AXP313A ALDO1 output
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t axp313a_camera_power_on(void);

/**
 * @brief Disable camera power
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t axp313a_camera_power_off(void);

#endif // AXP313A_H
