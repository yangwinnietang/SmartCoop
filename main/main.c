#include "axp313a.h"
#include "esp_camera.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "SmartCoop";

// Store IP address for display
static char s_ip_addr[16] = "0.0.0.0";

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
// Camera Initialization
// ==========================================
static esp_err_t init_camera(void) {
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

  ESP_LOGI(TAG, "Camera initialized successfully!");
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

  while (true) {
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

// Root handler - simple HTML page with embedded stream and auto-reconnect
static esp_err_t index_handler(httpd_req_t *req) {
  const char *html =
      "<!DOCTYPE html>"
      "<html><head><title>SmartCoop Camera</title>"
      "<meta name='viewport' content='width=device-width, initial-scale=1'>"
      "<style>"
      "body{font-family:Arial,sans-serif;background:#1a1a2e;color:white;text-"
      "align:center;padding:20px;margin:0;}"
      "h1{color:#00d4ff;}"
      "#stream{max-width:100%;border:3px solid #00d4ff;border-radius:10px;}"
      "#status{color:#ff6b6b;margin:10px;}"
      ".online{color:#4caf50 !important;}"
      "</style></head><body>"
      "<h1>SmartCoop Camera Stream</h1>"
      "<p id='status' class='online'>Connected</p>"
      "<img id='stream' src='/stream' alt='Camera Stream'>"
      "<script>"
      "var img=document.getElementById('stream');"
      "var status=document.getElementById('status');"
      "var reconnectTimer=null;"
      "function reconnect(){"
      "  status.textContent='Reconnecting...';"
      "  status.className='';"
      "  img.src='/stream?t='+Date.now();"
      "}"
      "img.onerror=function(){"
      "  status.textContent='Connection lost';"
      "  status.className='';"
      "  if(!reconnectTimer)reconnectTimer=setTimeout(function(){"
      "    reconnectTimer=null;reconnect();"
      "  },2000);"
      "};"
      "img.onload=function(){"
      "  status.textContent='Connected';"
      "  status.className='online';"
      "  clearTimeout(reconnectTimer);"
      "  reconnectTimer=null;"
      "};"
      "setInterval(function(){"
      "  if(img.complete && img.naturalHeight===0)reconnect();"
      "},5000);"
      "</script>"
      "</body></html>";

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

static httpd_handle_t start_webserver(void) {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.max_uri_handlers = 4;
  config.lru_purge_enable = true; // Enable LRU purge for sockets
  config.recv_wait_timeout = 30;  // 30 second receive timeout
  config.send_wait_timeout = 30;  // 30 second send timeout
  config.stack_size = 8192;       // Larger stack for streaming

  ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);

  if (httpd_start(&server, &config) == ESP_OK) {
    // Register URI handlers
    httpd_uri_t index_uri = {.uri = "/",
                             .method = HTTP_GET,
                             .handler = index_handler,
                             .user_ctx = NULL};
    httpd_register_uri_handler(server, &index_uri);

    httpd_uri_t stream_uri = {.uri = "/stream",
                              .method = HTTP_GET,
                              .handler = stream_handler,
                              .user_ctx = NULL};
    httpd_register_uri_handler(server, &stream_uri);

    ESP_LOGI(TAG, "HTTP server started successfully");
    return server;
  }

  ESP_LOGE(TAG, "Error starting HTTP server");
  return NULL;
}

// ==========================================
// Main Application Entry Point
// ==========================================
void app_main(void) {
  ESP_LOGI(TAG, "=== SmartCoop Camera System ===");
  ESP_LOGI(TAG, "DFRobot Romeo ESP32-S3 + OV2640");

  // Initialize NVS (required for WiFi)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Step 1: Initialize AXP313A and enable camera power
  ESP_LOGI(TAG, "Step 1: Initializing AXP313A power management...");
  ret = axp313a_init();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG,
             "AXP313A init failed (may not be present on this board variant)");
    // Continue anyway - some board variants may not have AXP313A
  } else {
    ret = axp313a_camera_power_on();
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to enable camera power");
      return;
    }
  }

  // Step 2: Initialize WiFi
  ESP_LOGI(TAG, "Step 2: Connecting to WiFi...");
  ret = wifi_init_sta();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "WiFi initialization failed");
    return;
  }

  // Step 3: Initialize Camera
  ESP_LOGI(TAG, "Step 3: Initializing camera...");
  ret = init_camera();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Camera initialization failed");
    return;
  }

  // Step 4: Start HTTP Server
  ESP_LOGI(TAG, "Step 4: Starting HTTP stream server...");
  httpd_handle_t server = start_webserver();
  if (server == NULL) {
    ESP_LOGE(TAG, "Failed to start web server");
    return;
  }

  // Print access info with actual IP
  ESP_LOGI(TAG, "=========================================");
  ESP_LOGI(TAG, "Camera stream ready!");
  ESP_LOGI(TAG, "Open browser and navigate to:");
  ESP_LOGI(TAG, "  http://%s/", s_ip_addr);
  ESP_LOGI(TAG, "Or direct stream:");
  ESP_LOGI(TAG, "  http://%s/stream", s_ip_addr);
  ESP_LOGI(TAG, "=========================================");

  // Keep running
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}
