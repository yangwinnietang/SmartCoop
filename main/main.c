#include "axp313a.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_camera.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sht30.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "SmartCoop";

// Store IP address for display
static char s_ip_addr[16] = "0.0.0.0";

// ==========================================
// MQ-137 Ammonia Sensor Configuration (GPIO 3)
// ==========================================
#define MQ137_ADC_CHANNEL ADC_CHANNEL_2 // GPIO 3 -> ADC1_CH2
#define MQ137_ADC_ATTEN ADC_ATTEN_DB_12 // 0-3.3V range

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static volatile int g_ammonia_raw = 0;
static volatile int g_ammonia_voltage_mv = 0;

// ==========================================
// SHT30 Temperature & Humidity Data
// ==========================================
static volatile float g_temperature = 0.0f;
static volatile float g_humidity = 0.0f;

// ==========================================
// Camera State Control
// ==========================================
static volatile bool g_camera_enabled = false;
static volatile bool g_camera_initialized = false;

// ==========================================
// WiFi Configuration
// ==========================================
#define WIFI_SSID "wlwdswifi"
#define WIFI_PASSWORD "12345678"
#define WIFI_MAXIMUM_RETRY 10

// ==========================================
// DFRobot Romeo ESP32-S3 Camera Pin Definition
// Using original reference code pin mapping
// ==========================================
#define CAM_PIN_PWDN -1  // Not used / internal pull-down
#define CAM_PIN_RESET -1 // Not used / internal pull-up
#define CAM_PIN_XCLK 45  // External clock
#define CAM_PIN_SIOD 1   // I2C SDA (SCCB)
#define CAM_PIN_SIOC 2   // I2C SCL (SCCB)

#define CAM_PIN_D7 48 // Data bit 7
#define CAM_PIN_D6 46 // Data bit 6
#define CAM_PIN_D5 8  // Data bit 5
#define CAM_PIN_D4 7  // Data bit 4
#define CAM_PIN_D3 4  // Data bit 3
#define CAM_PIN_D2 41 // Data bit 2
#define CAM_PIN_D1 40 // Data bit 1
#define CAM_PIN_D0 39 // Data bit 0

#define CAM_PIN_VSYNC 6 // Vertical sync
#define CAM_PIN_HREF 42 // Horizontal reference
#define CAM_PIN_PCLK 5  // Pixel clock

// ==========================================
// WiFi Event Handling
// ==========================================
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_event_sta_disconnected_t *disconn =
        (wifi_event_sta_disconnected_t *)event_data;
    ESP_LOGW(TAG, "WiFi disconnected, reason: %d", disconn->reason);
    if (s_retry_num < WIFI_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "Retrying WiFi connection... (%d/%d)", s_retry_num,
               WIFI_MAXIMUM_RETRY);
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
      ESP_LOGE(TAG, "WiFi connection failed after %d retries",
               WIFI_MAXIMUM_RETRY);
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    snprintf(s_ip_addr, sizeof(s_ip_addr), IPSTR, IP2STR(&event->ip_info.ip));
    ESP_LOGI(TAG, "Got IP address: %s", s_ip_addr);
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static esp_err_t wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = WIFI_SSID,
              .password = WIFI_PASSWORD,
              .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "WiFi STA initialized, connecting to %s...", WIFI_SSID);

  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Connected to WiFi SSID: %s", WIFI_SSID);
    return ESP_OK;
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGE(TAG, "Failed to connect to SSID: %s", WIFI_SSID);
    return ESP_FAIL;
  }

  return ESP_FAIL;
}

// ==========================================
// MQ-137 ADC Initialization
// ==========================================
static esp_err_t init_mq137_adc(void) {
  // ADC1 Unit Init
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

  // ADC1 Channel Config
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = MQ137_ADC_ATTEN,
  };
  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, MQ137_ADC_CHANNEL, &config));

  // ADC Calibration
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .atten = MQ137_ADC_ATTEN,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  esp_err_t ret =
      adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "ADC calibration scheme not supported, using raw values");
    adc_cali_handle = NULL;
  }

  ESP_LOGI(TAG, "MQ-137 ADC initialized on GPIO 3 (ADC1_CH2)");
  return ESP_OK;
}

// ==========================================
// MQ-137 Data Reading Task
// ==========================================
static void mq137_task(void *arg) {
  int raw_value = 0;
  int voltage = 0;

  while (true) {
    esp_err_t ret =
        adc_oneshot_read(adc1_handle, MQ137_ADC_CHANNEL, &raw_value);
    if (ret == ESP_OK) {
      g_ammonia_raw = raw_value;

      if (adc_cali_handle) {
        adc_cali_raw_to_voltage(adc_cali_handle, raw_value, &voltage);
        g_ammonia_voltage_mv = voltage;
      } else {
        // Approximate conversion without calibration (12-bit, 3.3V)
        g_ammonia_voltage_mv = (raw_value * 3300) / 4095;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500)); // Read every 500ms
  }
}

// ==========================================
// SHT30 Data Reading Task
// ==========================================
static void sht30_task(void *arg) {
  float temp, hum;

  while (true) {
    if (sht30_read(&temp, &hum) == ESP_OK) {
      g_temperature = temp;
      g_humidity = hum;
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); // Read every 2 seconds
  }
}

// ==========================================
// Camera Initialization
// ==========================================
static esp_err_t init_camera(void) {
  if (g_camera_initialized) {
    ESP_LOGI(TAG, "Camera already initialized");
    return ESP_OK;
  }

  camera_config_t config = {
      .ledc_channel = LEDC_CHANNEL_0,
      .ledc_timer = LEDC_TIMER_0,
      .pin_d0 = CAM_PIN_D0,
      .pin_d1 = CAM_PIN_D1,
      .pin_d2 = CAM_PIN_D2,
      .pin_d3 = CAM_PIN_D3,
      .pin_d4 = CAM_PIN_D4,
      .pin_d5 = CAM_PIN_D5,
      .pin_d6 = CAM_PIN_D6,
      .pin_d7 = CAM_PIN_D7,
      .pin_xclk = CAM_PIN_XCLK,
      .pin_pclk = CAM_PIN_PCLK,
      .pin_vsync = CAM_PIN_VSYNC,
      .pin_href = CAM_PIN_HREF,
      .pin_sccb_sda = CAM_PIN_SIOD,
      .pin_sccb_scl = CAM_PIN_SIOC,
      .pin_pwdn = CAM_PIN_PWDN,
      .pin_reset = CAM_PIN_RESET,

      .xclk_freq_hz = 20000000,       // 20MHz XCLK
      .pixel_format = PIXFORMAT_JPEG, // JPEG for streaming
      .frame_size = FRAMESIZE_VGA,    // 640x480 (stable for OV3660)
      .jpeg_quality = 12,             // Good quality (0-63)
      .fb_count = 2,                  // Double buffer
      .fb_location = CAMERA_FB_IN_PSRAM,
      .grab_mode = CAMERA_GRAB_LATEST, // Always get latest frame
  };

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
    return err;
  }

  // Get sensor and apply OV3660-specific settings
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    ESP_LOGI(TAG, "Camera sensor PID: 0x%02X", s->id.PID);

    // OV3660 specific optimizations
    if (s->id.PID == 0x3660) {
      ESP_LOGI(TAG, "Applying OV3660 optimizations...");
      s->set_brightness(s, 1); // Slight brightness boost
      s->set_saturation(s, 0); // Default saturation
      s->set_contrast(s, 0);   // Default contrast
    }
  }

  // Warm-up: capture and discard several frames to stabilize JPEG encoding
  ESP_LOGI(TAG, "Camera warm-up: discarding initial frames...");
  for (int i = 0; i < 10; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  g_camera_initialized = true;
  g_camera_enabled = true;
  ESP_LOGI(TAG, "Camera initialized successfully!");
  return ESP_OK;
}

// ==========================================
// Camera Deinitialization
// ==========================================
static esp_err_t deinit_camera(void) {
  if (!g_camera_initialized) {
    return ESP_OK;
  }

  esp_err_t err = esp_camera_deinit();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera deinit failed with error 0x%x", err);
    return err;
  }

  g_camera_initialized = false;
  g_camera_enabled = false;
  ESP_LOGI(TAG, "Camera deinitialized");
  return ESP_OK;
}

// ==========================================
// HTTP Stream Handler (MJPEG)
// ==========================================
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  if (!g_camera_enabled || !g_camera_initialized) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_send(req, "Camera is off", 13);
    return ESP_OK;
  }

  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];
  int error_count = 0;

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "10"); // Hint to browser

  ESP_LOGI(TAG, "Stream started");

  while (g_camera_enabled && g_camera_initialized) {
    fb = esp_camera_fb_get();
    if (!fb) {
      ESP_LOGW(TAG, "Camera capture failed, retrying...");
      error_count++;
      if (error_count > 5) {
        ESP_LOGE(TAG, "Too many capture errors, stopping stream");
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue; // Skip this frame, try again
    }
    error_count = 0; // Reset on success

    // Skip frames without valid JPEG data
    if (fb->len < 100 || fb->buf[0] != 0xFF || fb->buf[1] != 0xD8) {
      ESP_LOGW(TAG, "Invalid JPEG frame, skipping");
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);

    if (res != ESP_OK) {
      break;
    }

    // Control frame rate (~10 fps for stable streaming)
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ESP_LOGI(TAG, "Stream ended");
  return res;
}

// ==========================================
// Ammonia API Handler
// ==========================================
static esp_err_t ammonia_handler(httpd_req_t *req) {
  char response[128];
  snprintf(response, sizeof(response), "{\"raw\":%d,\"voltage_mv\":%d}",
           g_ammonia_raw, g_ammonia_voltage_mv);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, response, strlen(response));
}

// ==========================================
// SHT30 API Handler
// ==========================================
static esp_err_t sht30_handler(httpd_req_t *req) {
  char response[128];
  snprintf(response, sizeof(response),
           "{\"temperature\":%.1f,\"humidity\":%.1f}", g_temperature,
           g_humidity);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, response, strlen(response));
}

// ==========================================
// Camera Control Handlers
// ==========================================
static esp_err_t camera_on_handler(httpd_req_t *req) {
  esp_err_t ret = init_camera();
  const char *response =
      ret == ESP_OK ? "{\"status\":\"on\"}" : "{\"status\":\"error\"}";

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, response, strlen(response));
}

static esp_err_t camera_off_handler(httpd_req_t *req) {
  g_camera_enabled = false;
  vTaskDelay(pdMS_TO_TICKS(200)); // Allow stream to stop

  esp_err_t ret = deinit_camera();
  const char *response =
      ret == ESP_OK ? "{\"status\":\"off\"}" : "{\"status\":\"error\"}";

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, response, strlen(response));
}

static esp_err_t camera_status_handler(httpd_req_t *req) {
  char response[64];
  snprintf(response, sizeof(response), "{\"enabled\":%s,\"initialized\":%s}",
           g_camera_enabled ? "true" : "false",
           g_camera_initialized ? "true" : "false");

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, response, strlen(response));
}

// ==========================================
// Root Handler - Web UI
// ==========================================
static esp_err_t index_handler(httpd_req_t *req) {
  const char *html =
      "<!DOCTYPE html>"
      "<html><head><title>SmartCoop Monitor</title>"
      "<meta name='viewport' content='width=device-width, initial-scale=1'>"
      "<meta charset='UTF-8'>"
      "<style>"
      "*{box-sizing:border-box;margin:0;padding:0;}"
      "body{font-family:'Segoe UI',Arial,sans-serif;background:linear-gradient("
      "135deg,#1a1a2e 0%,#16213e "
      "100%);color:#fff;min-height:100vh;padding:20px;}"
      ".container{max-width:800px;margin:0 auto;}"
      "h1{text-align:center;color:#00d4ff;margin-bottom:20px;font-size:1.8em;"
      "text-shadow:0 0 20px rgba(0,212,255,0.5);}"
      ".card{background:rgba(255,255,255,0.1);border-radius:16px;padding:20px;"
      "margin-bottom:20px;backdrop-filter:blur(10px);border:1px solid "
      "rgba(255,255,255,0.1);}"
      ".card-title{font-size:1.2em;color:#00d4ff;margin-bottom:15px;display:"
      "flex;"
      "align-items:center;gap:10px;}"
      ".card-title::before{content:'';width:4px;height:20px;background:#00d4ff;"
      "border-radius:2px;}"
      ".sensor-data{display:flex;justify-content:space-around;text-align:"
      "center;}"
      ".sensor-item{padding:15px;}"
      ".sensor-value{font-size:2.5em;font-weight:bold;color:#4caf50;}"
      ".sensor-label{color:#aaa;font-size:0.9em;margin-top:5px;}"
      ".camera-container{text-align:center;}"
      "#stream{max-width:100%;border-radius:12px;background:#000;display:none;}"
      ".camera-placeholder{background:rgba(0,0,0,0.3);border-radius:12px;"
      "padding:60px;"
      "color:#666;font-size:1.2em;}"
      ".btn{padding:12px "
      "30px;font-size:1em;border:none;border-radius:8px;cursor:pointer;"
      "transition:all 0.3s;margin:10px 5px;font-weight:bold;}"
      ".btn-on{background:linear-gradient(135deg,#4caf50,#45a049);color:#fff;}"
      ".btn-off{background:linear-gradient(135deg,#f44336,#d32f2f);color:#fff;}"
      ".btn:hover{transform:translateY(-2px);box-shadow:0 5px 20px "
      "rgba(0,0,0,0.3);}"
      ".btn:disabled{opacity:0.5;cursor:not-allowed;transform:none;}"
      ".status{display:inline-block;padding:5px "
      "12px;border-radius:20px;font-size:0.85em;}"
      ".status-on{background:#4caf50;}"
      ".status-off{background:#666;}"
      "</style></head><body>"
      "<div class='container'>"
      "<h1>\ud83d\udc14 SmartCoop Monitor</h1>"
      "<div class='card'>"
      "<div class='card-title'>\u6c28\u6c14\u4f20\u611f\u5668 (MQ-137)</div>"
      "<div class='sensor-data'>"
      "<div class='sensor-item'>"
      "<div class='sensor-value' id='voltage'>--</div>"
      "<div class='sensor-label'>\u7535\u538b (mV)</div>"
      "</div>"
      "<div class='sensor-item'>"
      "<div class='sensor-value' id='raw'>--</div>"
      "<div class='sensor-label'>ADC \u539f\u59cb\u503c</div>"
      "</div>"
      "</div></div>"
      "<div class='card'>"
      "<div class='card-title'>\u6e29\u6e7f\u5ea6\u4f20\u611f\u5668 (SHT30)</div>"
      "<div class='sensor-data'>"
      "<div class='sensor-item'>"
      "<div class='sensor-value' id='temp'>--</div>"
      "<div class='sensor-label'>\u6e29\u5ea6 (\u00b0C)</div>"
      "</div>"
      "<div class='sensor-item'>"
      "<div class='sensor-value' id='hum'>--</div>"
      "<div class='sensor-label'>\u6e7f\u5ea6 (%)</div>"
      "</div>"
      "</div></div>"
      "<div class='card'>"
      "<div class='card-title'>\u6444\u50cf\u5934\u76d1\u63a7 "
      "<span class='status status-off' id='cam-status'>\u5173\u95ed</span></div>"
      "<div class='camera-container'>"
      "<div class='camera-placeholder' id='placeholder'>\ud83d\udcf7 \u6444\u50cf\u5934\u5df2\u5173\u95ed</div>"
      "<img id='stream' src='' alt='Camera Stream'>"
      "<div style='margin-top:15px;'>"
      "<button class='btn btn-on' id='btn-on' "
      "onclick='cameraOn()'>\u5f00\u542f\u6444\u50cf\u5934</button>"
      "<button class='btn btn-off' id='btn-off' onclick='cameraOff()' "
      "disabled>\u5173\u95ed\u6444\u50cf\u5934</button>"
      "</div></div></div>"
      "</div>"
      "<script>"
      "function updateAmmonia(){"
      "fetch('/api/ammonia').then(r=>r.json()).then(d=>{"
      "document.getElementById('voltage').textContent=d.voltage_mv;"
      "document.getElementById('raw').textContent=d.raw;"
      "}).catch(e=>console.log('Ammonia fetch error'));"
      "}"
      "function updateSHT30(){"
      "fetch('/api/sht30').then(r=>r.json()).then(d=>{"
      "document.getElementById('temp').textContent=d.temperature.toFixed(1);"
      "document.getElementById('hum').textContent=d.humidity.toFixed(1);"
      "}).catch(e=>console.log('SHT30 fetch error'));"
      "}"
      "function updateCameraStatus(){"
      "fetch('/api/camera/status').then(r=>r.json()).then(d=>{"
      "var st=document.getElementById('cam-status');"
      "var img=document.getElementById('stream');"
      "var ph=document.getElementById('placeholder');"
      "var btnOn=document.getElementById('btn-on');"
      "var btnOff=document.getElementById('btn-off');"
      "if(d.enabled&&d.initialized){"
      "st.textContent='\u8fd0\u884c\u4e2d';st.className='status status-on';"
      "img.style.display='block';ph.style.display='none';"
      "if(!img.src.includes('/stream'))img.src='/stream?t='+Date.now();"
      "btnOn.disabled=true;btnOff.disabled=false;"
      "}else{"
      "st.textContent='\u5173\u95ed';st.className='status status-off';"
      "img.style.display='none';ph.style.display='block';img.src='';"
      "btnOn.disabled=false;btnOff.disabled=true;"
      "}"
      "}).catch(e=>console.log('Status fetch error'));"
      "}"
      "function cameraOn(){"
      "var btnOn=document.getElementById('btn-on');"
      "btnOn.disabled=true;btnOn.textContent='\u6b63\u5728\u5f00\u542f...';"
      "fetch('/api/camera/on').then(r=>r.json()).then(d=>{"
      "btnOn.textContent='\u5f00\u542f\u6444\u50cf\u5934';"
      "updateCameraStatus();"
      "}).catch(e=>{alert('\u5f00\u542f\u5931\u8d25');btnOn.textContent='\u5f00\u542f\u6444\u50cf\u5934';btnOn.disabled=false;});"
      "}"
      "function cameraOff(){"
      "fetch('/api/camera/off').then(r=>r.json()).then(d=>{"
      "updateCameraStatus();"
      "}).catch(e=>alert('\u5173\u95ed\u5931\u8d25'));"
      "}"
      "setInterval(updateAmmonia,1000);"
      "setInterval(updateSHT30,2000);"
      "setInterval(updateCameraStatus,3000);"
      "updateAmmonia();updateSHT30();updateCameraStatus();"
      "</script></body></html>";

  return httpd_resp_send(req, html, strlen(html));
}

// ==========================================
// Server Initialization
// ==========================================
static httpd_handle_t start_webserver(void) {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.lru_purge_enable = true;

  ESP_LOGI(TAG, "Starting web server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) {
    // URI Handlers
    httpd_uri_t index_uri = {
        .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL};
    httpd_register_uri_handler(server, &index_uri);

    httpd_uri_t stream_uri = {
        .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL};
    httpd_register_uri_handler(server, &stream_uri);

    httpd_uri_t ammonia_uri = {
        .uri = "/api/ammonia", .method = HTTP_GET, .handler = ammonia_handler, .user_ctx = NULL};
    httpd_register_uri_handler(server, &ammonia_uri);

    httpd_uri_t sht30_uri = {
        .uri = "/api/sht30", .method = HTTP_GET, .handler = sht30_handler, .user_ctx = NULL};
    httpd_register_uri_handler(server, &sht30_uri);

    httpd_uri_t cam_on_uri = {
        .uri = "/api/camera/on", .method = HTTP_GET, .handler = camera_on_handler, .user_ctx = NULL};
    httpd_register_uri_handler(server, &cam_on_uri);

    httpd_uri_t cam_off_uri = {
        .uri = "/api/camera/off", .method = HTTP_GET, .handler = camera_off_handler, .user_ctx = NULL};
    httpd_register_uri_handler(server, &cam_off_uri);

    httpd_uri_t cam_status_uri = {
        .uri = "/api/camera/status", .method = HTTP_GET, .handler = camera_status_handler, .user_ctx = NULL};
    httpd_register_uri_handler(server, &cam_status_uri);

    return server;
  }

  ESP_LOGE(TAG, "Failed to start web server");
  return NULL;
}

void app_main(void) {
  esp_err_t ret;

  // Step 1: Initialize NVS (required for WiFi)
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Step 2: Initialize Power Management (AXP313A)
  ESP_LOGI(TAG, "Step 2: Initializing AXP313A...");
  ret = axp313a_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "AXP313A initialization failed, stopping.");
    return;
  }

  // Turn on camera power immediately to let it stabilize
  axp313a_camera_power_on();

  // Step 3: Initialize MQ-137 Ammonia Sensor
  ESP_LOGI(TAG, "Step 3: Initializing MQ-137 ADC...");
  init_mq137_adc();
  xTaskCreate(mq137_task, "mq137_task", 2048, NULL, 5, NULL);

  // Step 4: Initialize SHT30 temperature & humidity sensor
  ESP_LOGI(TAG, "Step 4: Initializing SHT30 sensor (SDA=IO16, SCL=IO17)...");
  ret = sht30_init();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "SHT30 initialization failed (sensor may not be connected)");
  } else {
    xTaskCreate(sht30_task, "sht30_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "SHT30 reading task started");
  }

  // Step 5: Connect to WiFi
  ESP_LOGI(TAG, "Step 5: Connecting to WiFi...");
  if (wifi_init_sta() == ESP_OK) {
    // Step 6: Start HTTP Server
    ESP_LOGI(TAG, "Step 6: Starting Web Server...");
    start_webserver();
  }

  ESP_LOGI(TAG, "System ready! IP: %s", s_ip_addr);
}
