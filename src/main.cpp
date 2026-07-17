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
#define MAX_CAPTURE_ATTEMPTS 3
 
// If this many captures fail in a row, something is genuinely wrong
// (most likely the sensor stuck in a bad state from a clock desync).
// Recover via a real deep sleep: waking from deep sleep is a full reboot,
// which resets the sensor and all peripherals instantly. The sleep
// duration itself doesn't need to be long - it just needs to be nonzero
// so the timer wakeup can trigger the reboot.
#define MAX_CONSECUTIVE_FAILURES 5
#define RECOVERY_SLEEP_SEC 5
 
// ---------------- Logging ----------------
 
// Simple timestamped log to SD for diagnosing intermittent failures
// after the fact (millis() resets on every reboot/deep sleep, but the
// relative gaps between entries are still useful for spotting patterns).
void logEvent(String msg) {
  File f = SD_MMC.open("/log.txt", FILE_APPEND);
  if (!f) return;
  f.printf("[%lu ms] %s\n", millis(), msg.c_str());
  f.close();
}
 
// ---------------- LED ----------------
 
void blinkLed(int times, int delayMs = 200) {
  pinMode(STATUS_LED_PIN, OUTPUT);
 
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
  s->set_ae_level(s, 0);      // neutral exposure target, not biased bright
  s->set_aec_value(s, 300);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, (gainceiling_t)2); // moderate ceiling, not max
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
 
// A valid JPEG must start with SOI (0xFFD8) and end with EOI (0xFFD9),
// and be a plausible size. Corrupted/incomplete frames from an SCCB
// glitch or timing hiccup after light sleep usually fail this check.
bool isValidJpeg(camera_fb_t * fb) {
  if (!fb || fb->len < 1000) return false;
 
  bool validStart = (fb->buf[0] == 0xFF && fb->buf[1] == 0xD8);
  bool validEnd = (fb->buf[fb->len - 2] == 0xFF && fb->buf[fb->len - 1] == 0xD9);
 
  return validStart && validEnd;
}
 
bool takePhoto(String path){
  camera_fb_t * fb = nullptr;
 
  for (int attempt = 1; attempt <= MAX_CAPTURE_ATTEMPTS; attempt++) {
    if (fb) esp_camera_fb_return(fb);
 
    fb = esp_camera_fb_get();
 
    if (isValidJpeg(fb)) {
      break; // good frame, stop retrying
    }
 
    Serial.printf("Capture attempt %d produced a bad frame, retrying...\n", attempt);
    delay(50);
  }
 
  if (!isValidJpeg(fb)) {
    if (fb) esp_camera_fb_return(fb);
    return false;
  }
 
  File file = SD_MMC.open(path.c_str(), FILE_WRITE);
 
  if (!file) {
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
  // Keep these domains powered through light sleep. The camera's XCLK
  // is generated via LEDC, which depends on the APB clock; letting these
  // domains power down mid-sleep can interrupt XCLK and desync the
  // sensor on wake, which shows up as gray/corrupted frames.
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
 
  esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000ULL);
  esp_light_sleep_start();
  // Execution resumes here on wake - camera, SD_MMC, and sensor
  // configuration all remain intact, no re-init needed.
}
 
// ---------------- Main ----------------
 
void setup() {
  Serial.begin(115200);
 
  if(!SD_MMC.begin()){ // default 4-bit mode
    blinkLed(100);
    return;
  }
 
  if (!configInitCamera()) {
    return; // init failure, program stops here
  }
 
  blinkLed(3, 1000); // signal successful startup
 
  uint32_t count = loadCounter();
  uint32_t consecutiveFailures = 0;
 
  logEvent("Boot / camera init OK");
 
  while (true) {
    String path = "/picture_" + String(count) + ".jpg";
 
    if (takePhoto(path)) {
      saveCounter(++count);
      Serial.println(path);
      consecutiveFailures = 0;
    } else {
      consecutiveFailures++;
      logEvent("Capture failure #" + String(consecutiveFailures));
 
      if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
        logEvent("Too many consecutive failures, entering recovery deep sleep");
        esp_sleep_enable_timer_wakeup(RECOVERY_SLEEP_SEC * 1000000ULL);
        esp_deep_sleep_start();
        // Device fully reboots after this; setup() re-initializes everything clean.
      }
    }
 
    enterLightSleep(CAPTURE_INTERVAL_SEC);
    delay(20); // let camera clock/SCCB stabilize after wake before next capture
  }
}
 
void loop() {
  // unused - all logic lives in setup()'s while(true) loop
}