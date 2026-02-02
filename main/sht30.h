#ifndef SHT30_H
#define SHT30_H

#include "esp_err.h"

/**
 * @brief SHT30 温湿度传感器驱动
 *
 * I2C 接口: SDA=IO16, SCL=IO17
 * I2C 地址: 0x44 (默认)
 */

/**
 * @brief 初始化 SHT30 传感器
 *
 * 初始化 I2C 总线并验证传感器是否存在
 *
 * @return ESP_OK 成功, 其他值表示错误
 */
esp_err_t sht30_init(void);

/**
 * @brief 读取温湿度数据
 *
 * @param temperature 输出温度值 (摄氏度)
 * @param humidity 输出相对湿度值 (%)
 * @return ESP_OK 成功, 其他值表示错误
 */
esp_err_t sht30_read(float *temperature, float *humidity);

/**
 * @brief 释放 SHT30 资源
 *
 * @return ESP_OK 成功
 */
esp_err_t sht30_deinit(void);

#endif // SHT30_H
