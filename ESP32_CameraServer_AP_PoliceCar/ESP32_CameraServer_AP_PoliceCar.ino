/* [Elegoo Robot Car]
 * @Description: Robot Car setup and Network Controls
 * @version: Arduino IDE 2.0.4
 * @Author: TOREMETAL (Modified: Elegoo for Arduino IDE 2.0.4)
 * @Date: 2023-04-19
 * @LastEditors: TOREMETAL
 */
#include "ESP32_CameraServer_AP_PoliceCar.h"
#include "esp_camera.h"
#include <WiFi.h>

#define RXD2 33
#define TXD2 4
CameraWebServer_AP CameraWebServerAP;

void setup() {
  Serial.begin(19200);  //115200 9600); false true
  Serial.setDebugOutput(false);
  Serial.println();
  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);
  CameraWebServerAP.CameraWebServer_AP_Init();
  delay(100);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial2.print("{Factory}");
}

void loop() {
  CameraWebServerAP.SocketServer_Test();
  CameraWebServerAP.FactoryTest();
  //  if (WiFi.status() != WL_CONNECTED) {
  // Serial2.println("{WA_NO}");
  // Serial2.println(WiFi.status() + "" + WL_CONNECTED);
  // CameraWebServerAP.Connect_WiFi();
  //  }
  // Do nothing. Everything is done in another task by the web server
  //delay(10000);
}
