
#include <Arduino.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
 
#define STATUS_LED_PIN  33   // onboard red LED, active-LOW
 
// AI Thinker ESP32-CAM pin definition
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
 
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
 
// ---- Capture cycle config ----
#define CAPTURE_INTERVAL_SEC 3
 
// ---- Camera init config ----
#define MAX_CAMERA_INIT_ATTEMPTS 3
 
// ---------------- LED ----------------
 
void blinkError(int times, int delayMs = 200) {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH); // start off (active-LOW LED)
 
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED_PIN, LOW);  // on
    delay(delayMs);
    digitalWrite(STATUS_LED_PIN, HIGH); // off
    delay(delayMs);
  }
}
 
// ---------------- Camera ----------------
 
bool configInitCamera(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
 
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 4;
  config.fb_count = 1;
 
  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_SVGA; // fall back if no PSRAM
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
 
  esp_err_t err;
 
  for (int attempt = 1; attempt <= MAX_CAMERA_INIT_ATTEMPTS; attempt++) {
    err = esp_camera_init(&config);
    if (err == ESP_OK) {
      break; // success, stop retrying
    }
 
    Serial.printf("Camera init attempt %d failed: 0x%x\n", attempt, err);
    esp_camera_deinit();
    delay(200);
  }
 
  if (err != ESP_OK) {
    Serial.printf("Camera init failed after %d attempts: 0x%x\n", MAX_CAMERA_INIT_ATTEMPTS, err);
    blinkError(100);
    return false;
  }
 
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_special_effect(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);          // advanced AEC, better in varying light
  s->set_ae_level(s, 1);      // push exposure slightly brighter
  s->set_aec_value(s, 300);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, (gainceiling_t)2); // allow more gain in low light
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);
 
  return true;
}
 
bool takePhoto(String path){
  camera_fb_t * fb = esp_camera_fb_get();
 
  if(!fb){
    Serial.println("Camera capture failed");
    blinkError(2);
    return false;
  }
 
  File file = SD_MMC.open(path.c_str(), FILE_WRITE);
 
  if (!file) {
    Serial.println("Failed to create file");
    esp_camera_fb_return(fb);
    return false;
  }
 
  file.write(fb->buf, fb->len);
  file.close();
  esp_camera_fb_return(fb);
  return true;
}
 
// ---------------- Counter ----------------
 
uint32_t loadCounter(){
  File f = SD_MMC.open("/counter.txt");
  if(!f) return 0;
 
  uint32_t count = f.parseInt();
  f.close();
  return count;
}
 
void saveCounter(uint32_t count){
  File f = SD_MMC.open("/counter.txt", FILE_WRITE);
  if(!f) return;
 
  f.seek(0);
  f.print(count);
  f.close();
}
 
// ---------------- Sleep ----------------
 
void enterLightSleep(uint64_t sleepSeconds) {
  esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000ULL);
  esp_light_sleep_start();
  // Execution resumes here on wake - camera, SD_MMC, and sensor
  // configuration all remain intact, no re-init needed.
}
 
// ---------------- Main ----------------
 
void setup() {
  Serial.begin(115200);
 
  if(!SD_MMC.begin()){ // default 4-bit mode
    Serial.println("SD Card Mount Failed");
    blinkError(100);
    return;
  }
 
  if (!configInitCamera()) {
    return; // init failure already blinked and logged
  }
 
  uint32_t count = loadCounter();
 
  while (true) {
    String path = "/picture_" + String(count) + ".jpg";
    if (takePhoto(path)) {
      saveCounter(++count);
      Serial.println(path);
    }
 
    enterLightSleep(CAPTURE_INTERVAL_SEC);
    // Optional: uncomment if you see occasional failed captures
    // right after wake, to let the camera clock stabilize.
    // delay(20);
  }
}
 
void loop() {
  // unused - all logic lives in setup()'s while(true) loop
}