#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"

HardwareSerial ArduinoSerial(1);
#define ARDUINO_RX 3
#define ARDUINO_TX 40

#define SERIAL_BUFFER_SIZE 1024  // Increased UART buffer size

// Camera settings, just pinout 
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y2_GPIO_NUM      11
#define Y3_GPIO_NUM       9
#define Y4_GPIO_NUM       8
#define Y5_GPIO_NUM      10
#define Y6_GPIO_NUM      12
#define Y7_GPIO_NUM      18
#define Y8_GPIO_NUM      17
#define Y9_GPIO_NUM      16
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM    13

void setup() {
  Serial.begin(1000000);  // serial at 1M for cam stream
  ArduinoSerial.begin(9600, SERIAL_8N1, ARDUINO_RX, ARDUINO_TX);  //software serial for arduino 

  camera_config_t config; // all a bunch of camera config settings from the original espressif example code. 
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
  config.frame_size = FRAMESIZE_XGA; //1024x768 res for cam
  config.jpeg_quality = 10;  // compression 
  config.fb_count = 1; // buffer 
  config.fb_location = CAMERA_FB_IN_PSRAM; // store on larger psram 
  config.grab_mode = CAMERA_GRAB_LATEST;

if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  Serial.println("ESP32 ready.");
}

void loop() {
  // Check for the stream command from Pi
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("stream")) {
      // Capture and send a single frame when the "stream" command is received
      camera_fb_t *fb = esp_camera_fb_get(); // store camera stream, fb is pointer
      if (fb) {
        uint32_t len = fb->len;
        if (len > 0 && len < 150000) {  // Check if the frame size is valid and not null
          Serial.write(0xFF); // send start bit twice just in case. doesnt make opencv angy 
          Serial.write(0xFF);
          Serial.write((uint8_t*)&len, 4); // send data length as little endian so opencv knows how much to receive 
          Serial.write(fb->buf, fb->len); // finaly send.
          Serial.println("Frame sent to Pi");  // Add confirmation that frame was sent
        } else {
          Serial.println("Invalid frame size, skipping..."); // debug, show if code is not making proper frame before sending
        }
        esp_camera_fb_return(fb);
      } else {
        Serial.println("Failed to capture frame"); // show if frame cant be 
      }
    } else {
      // Relay the received command to Arduino
      ArduinoSerial.println(cmd);
      Serial.println("Forwarded to Arduino: " + cmd);
    }
  }

  delay(10);  // Prevent watchdog timeout. DO NOT REMOVE, LOAD BEARING COCONUT 
}