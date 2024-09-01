/* [Elegoo Robot Car]
 * @Description: Robot Car setup and Network Controls
 * @version: Arduino IDE 2.0.4
 * @Author: TOREMETAL (Modified: Elegoo for Arduino IDE 2.0.4)
 * @Date: 2023-04-19
 * @LastEditors: TOREMETAL
 */

#ifndef _CameraWebServer_AP_H
#define _CameraWebServer_AP_H
#include "esp_camera.h"
#include <WiFi.h>

class CameraWebServer_AP {

public:
  void CameraWebServer_AP_Init(void);
  void readResponse(WiFiClient* client);
  void SocketServer_Test(void);
  void FactoryTest(void);
  void findWiFi(void);
  void Connect_WiFi(void);
  void WiFi_Connected(void);
  String wifi_name;

private:
  // ===========================
  //  WiFi station ssid, password.
  // ===========================
  const char* ssid = "Robot-Car";
  const char* password = "PoliceCar";

  // ===========================
  // Enter your WiFi credentials to add
  // Wifi Networks to auto connect too
  // ===========================
  const char* ssid1 = "";
  const char* password1 = "";
  const char* ssid2 = "";
  const char* password2 = "";
  const char* ssid3 = "";
  const char* password3 = "";
};

#endif
