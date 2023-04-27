/*
  ESP_CAM_Camera.ino
  Captures a Picture from the ESP32 Cam and then Sends it to a Websocket Client. Only supports 1 client.
  The picture sent is encoded as .jpeg.
  Created by Nischay Joshi, 10-05-2023
*/

#include <WiFi.h>
#include <WebSocketsServer.h>
#include "esp_camera.h"
#include "PinAssignments.h"


// Replace with your network credentials
const char* ssid = "OP7";
const char* password = "EightCharLong";
const char* wifi_ssid = "Nj";
const char* wifi_pass = "r18nmbr4";
#define WifiConfigPin 16  //IO 16


// Replace with your WebSocket server settings
int server_port = 8080;

// Replace with the frequency at which you want to send images (in milliseconds)
int interval_ms = 33;
// Set up the WebSocket Server
WebSocketsServer webSocket = WebSocketsServer(server_port);
int client_num = 0;
bool connected = false;

void WebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length){
   switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            connected = false;
            break;
        case WStype_CONNECTED:
            Serial.printf("[%u] Connected!\n", num);
            client_num = num;
            connected = true;
            break;
        case WStype_TEXT:
        case WStype_BIN:
        case WStype_ERROR:      
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
        case WStype_PONG:
        case WStype_PING:
            break;
    }
}

// Initialize the ESP32-CAM camera
void setup_camera() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }
}

void SendImage(){
  //click the image
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  
  // Send the image to the WebSocket client
  webSocket.sendBIN(client_num, fb->buf, fb->len);
  // Free the camera frame buffer
  esp_camera_fb_return(fb);

}

void setup() {
  // Start the serial communication
  Serial.begin(115200);
  String IP;
  //Configure the IO pin as Input with internal pull up.
  pinMode(WifiConfigPin, INPUT_PULLUP);
  delay(500);
  //Read the status and cofigure the wifi appropriately
  if(digitalRead(WifiConfigPin)){//if pin is high then set soft AP i.e wifi host mode
      // make own wifi
    WiFi.mode(WIFI_STA);
    WiFi.softAP(ssid, password);
    Serial.println("Wifi Access Point Created");
    IP = WiFi.softAPIP().toString();
  }
  else{
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_pass);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
    }
    Serial.println("Connected to WiFi");
    IP = WiFi.localIP().toString();
  }

  // Set up the camera
  setup_camera();

  // Connect to the WebSocket server
  webSocket.begin();
  webSocket.onEvent(WebSocketEvent);
  
  Serial.print("Websocket Server Address: ws://" + IP + ":");
  Serial.println(server_port);
  Serial.println("Setup Complete");
}

void loop() {
  webSocket.loop();
  if(connected){
    SendImage();
  }
  // Wait for the defined interval before capturing the next image
  delay(interval_ms);
}
