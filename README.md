# 🐔 SmartCoop Monitor (智能鸡舍监控系统)

基于 ESP32-S3 的智能鸡舍环境监控系统，集成了实时视频流传输与氨气浓度监测功能。

## 📋 项目简介

本项目运行于 **DFRobot Romeo ESP32-S3** 开发板，主要功能包括：
1. **实时视频监控**: 通过 OV3660 摄像头提供 MJPEG 视频流。
2. **环境监测**: 集成 MQ-137 氨气传感器与 **SHT30** 温湿度传感器。
3. **Web 控制台**: 现代化 Web UI，支持实时数据刷新 (1-2s 频率) 与摄像头开关控制。
4. **电源管理**: 集成 AXP313A 电源管理驱动 (解决 I2C 冲突)。

## 🛠️ 硬件配置

| 组件 | 型号/备注 | 连接引脚 |
| :--- | :--- | :--- |
| **主控板** | DFRobot Romeo ESP32-S3 | - |
| **摄像头** | OV3660 (或 OV2640) | DVP 接口 (板载接口) |
| **氨气传感器** | MQ-137 模块 | 模拟输出 -> **GPIO 3** (ADC1_CH2) |
| **温湿度传感器** | SHT30 模块 | I2C (SDA: **IO16**, SCL: **IO17**) |
| **电源管理** | AXP313A (板载) | I2C (SDA: IO1, SCL: IO2) |

> ⚠️ **注意**: MQ-137 传感器需接 **GPIO 3**。原计划使用的 GPIO 4 与摄像头数据线冲突，不可使用。

## 🚀 快速开始

### 1. 环境准备
- 安装 [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/get-started/index.html) (推荐 v5.4)
- 配置环境变量 (`export.bat` / `. ./export.sh`)

### 2. 编译与烧录

```powershell
# 进入项目目录
cd SmartCoop

# 编译
idf.py build

# 烧录并打开监视器 (替换 COMx 为实际端口)
idf.py -p COMx flash monitor
```

### 3. 使用说明
1. 设备启动后会自动连接 WiFi (SSID: `wlwdswifi`, PWD: `12345678`)。
2. 查看串口日志获取设备 IP 地址 (例如 `192.168.1.100`)。
3. 浏览器访问 `http://<设备IP>/` 进入控制台。
4. **摄像头默认为关闭状态**，点击页面上的"开启摄像头"按钮即可查看画面。

## ⚙️ 技术细节

### I2C 驱动冲突解决
由于 AXP313A 电源芯片与摄像头 (SCCB) 共用 I2C 引脚 (IO1, IO2)，且 ESP32-Camera 驱动使用了新版 I2C 驱动，导致初始化冲突。
**解决方案**:
- 重写了 `axp313a.c` 驱动。
- 采用 **Transient Bus (瞬态总线)** 模式：每次读写寄存器时创建 I2C 总线，操作完立即释放。
- 确保在摄像头初始化时，I2C 引脚处于空闲状态。

### ADC 校准
- 使用 `esp_adc/adc_cali_scheme.h` 中的曲线拟合 (Curve Fitting) 方案。
- 针对 ESP32-S3 ADC1 进行校准，提供准确的电压读数。

## 📂 目录结构

```
SmartCoop/
├── main/
│   ├── main.c           # 主程序 (Web服务器, 传感器任务, 摄像头控制)
│   ├── axp313a.c        # 电源管理驱动 (解决 I2C 冲突)
│   ├── axp313a.h        # 电源管理头文件
│   └── idf_component.yml # 组件依赖 (esp32-camera)
├── CMakeLists.txt       # 构建脚本
└── README.md            # 项目说明文档
```

## 📝 许可证
MIT License
