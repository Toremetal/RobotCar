#include "sensor.h"
/* [Elegoo Robot Car]
 * @Description: Robot Car setup and Network Controls
 * @version: Arduino IDE 2.0.4
 * @Author: TOREMETAL (Modified: Elegoo for Arduino IDE 2.0.4)
 * @Date: 2023-04-19
 * @LastEditors: TOREMETAL
 */
// WARNING!!! Make sure that you have selected Board ---> ESP32 Dev Module
//            Partition Scheme ---> Huge APP (3MB No OTA/1MB SPIFFS)
//            PSRAM ---> enabled
//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// #define HTTPD_DEFAULT_CONFIG()             \
//   {                                        \
//     .task_priority = tskIDLE_PRIORITY + 5, \
//     .stack_size = 4096,                    \
//     .server_port = 80,                     \
//     .ctrl_port = 32768,                    \
//     .max_open_sockets = 7,                 \
//     .max_uri_handlers = 8,                 \
//     .max_resp_headers = 8,                 \
//     .backlog_conn = 5,                     \
//     .lru_purge_enable = false,             \
//     .recv_wait_timeout = 5,                \
//     .send_wait_timeout = 5,                \
//     .global_user_ctx = NULL,               \
//     .global_user_ctx_free_fn = NULL,       \
//     .global_transport_ctx = NULL,          \
//     .global_transport_ctx_free_fn = NULL,  \
//     .open_fn = NULL,                       \
//     .close_fn = NULL,                      \
//   }
// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
#define CAMERA_MODEL_M5STACK_WIDE  // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD

#include "ESP32_CameraServer_AP_PoliceCar.h"
#include "camera_pins.h"
#include "esp_system.h"
//#include "SimpleBLE.h"
//#include "BluetoothSerial.h"

//#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
//#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
//#endif

//SimpleBLE ble;
//BluetoothSerial SerialBT;
WiFiServer server(100);
bool WA_en = false;
void startCameraServer();
void setupLedFlash(int pin);
void delay500();

void delay500() {
  delay(500);
}
/* Find available (Public) wifi network to connect to. */
void CameraWebServer_AP::findWiFi(void) {
  int16_t n = WiFi.scanNetworks();
  for (uint8_t i = 0; i < n; ++i) {
    if (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) {
      Serial.print("Connecting to ");
      Serial.println(WiFi.SSID(i).c_str());
      // Connect to Wi-Fi network with SSID and password.
      WiFi.begin(WiFi.SSID(i).c_str(), "");
      i = 0;
      while (WiFi.status() != WL_CONNECTED) {
        delay500();
        i++;
        if (i > 5) {
          break;
        }
      }
      if (WiFi.status() == WL_CONNECTED) {
        // Print local IP address, for web server connections.
        Serial.println(":----------------------------:");
        Serial.println("Connected to Public WiFi.");
        Serial.println(WiFi.localIP());
        Serial.println(":----------------------------:");
        Serial2.print("{Connected to Public WiFi(");
        Serial2.print(WiFi.localIP());
        Serial2.println(")}");
        Serial2.println("{WA_OK}");
        WiFi.scanDelete();
        return;
      }
    }
  }
  WiFi.scanDelete();
  return;
}
/* Optional.
  [ Set your Static IP address ].
    IPAddress local_IP(10, 0, 0, 250);
  [ Set your Gateway IP address ].
    IPAddress gateway(10, 0, 0, 1);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress primaryDNS(75, 75, 75, 75);
    IPAddress secondaryDNS(75, 75, 76, 76);
    IPAddress dnsIP(10, 0, 0, 1);
  */
void CameraWebServer_AP::Connect_WiFi(void) {
  int16_t n = WiFi.scanNetworks();
  for (uint8_t i = 0; i < n; ++i) {
    if (strcmp(WiFi.SSID(i).c_str(), ssid1) == 0) {
      // Connect to Wi-Fi network with SSID and password
      Serial.print("Connecting to:");
      Serial.println(ssid1);
      WiFi.begin(ssid1, password1);
    } else if (strcmp(WiFi.SSID(i).c_str(), ssid2) == 0) {
      // Connect to Wi-Fi network with SSID and password
      Serial.print("Connecting to:");
      Serial.println(ssid2);
      WiFi.begin(ssid2, password2);
    } else if (strcmp(WiFi.SSID(i).c_str(), ssid3) == 0) {
      // Connect to Wi-Fi network with SSID and password
      Serial.print("Connecting to:");
      Serial.println(ssid3);
      WiFi.begin(ssid3, password3);
    }
    WiFi.setSleep(false);
    // (Recycling variables to reduce Memory consumption
    // to pack more advanced coding into smaller spaces.
    // Each variable declared increases used memory space.
    // Reusing variables can help reduce consumption).
    // recycle variable i to 0 to use to limit the amount of time spent
    // attemting to connect to a network; to Prevent an infinite loop
    // if the network can not be connected too.
    i = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay500();
      if (WiFi.status() == WL_CONNECTED) {
        WiFi_Connected();
        break;
      }
      i++;
      if (i >= 15) {
        Serial.println("Connection to WiFi not Detected.");
        Serial2.println("Connection to WiFi not Detected.");
        break;
      }
    }
    break;
  }
  // Reduce Memory Consumption.
  WiFi.scanDelete();
  return;
}

void CameraWebServer_AP::WiFi_Connected(void) {
  // Serial.print();
  Serial.println(":----------------------------:");
  // Print local IP address
  Serial.println("Connected to WiFi.");
  Serial.print("Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println(":100'");
  Serial.println("To Access the Car Controller or");
  Serial.print("'http://");
  Serial.print(WiFi.localIP());
  Serial.println("'");
  Serial.println("To Access the Cam Controller");
  //Serial.println("{WA_OK}");
  Serial.println(":----------------------------:");
  // Serial2.print();
  Serial2.println("{WA_OK}");
  Serial2.print("{Connected to WiFi(");
  Serial2.print(WiFi.localIP());
  Serial2.println(")}");
  return;
}

void CameraWebServer_AP::CameraWebServer_AP_Init(void) {
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
  config.frame_size = FRAMESIZE_SVGA;    //FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 14;
  config.fb_count = 1;
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.frame_size = FRAMESIZE_UXGA;
      config.jpeg_quality = 12;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_HVGA;  //FRAMESIZE_QVGA
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t* s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

  //#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  //s->set_vflip(s, 1);
  //s->set_hmirror(s, 1);
  //#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  //ble.begin("ESP32 SimpleBLE");
  Serial.println("\r\n");
  uint64_t chipid = ESP.getEfuseMac();
  char string[10];
  sprintf(string, "%05X", (uint16_t)(chipid >> 32));
  String mac0_default = String(string);  //string
  sprintf(string, "%08X", (uint32_t)chipid);
  String mac1_default = String(string);
  String url = ssid;  // + mac0_default + mac1_default;
  const char* mac_default = url.c_str();
  wifi_name = ssid;  // mac0_default + mac1_default;
  //WiFi.disconnect();
  //delay500();
  // WiFi.mode(WIFI_AP);
  //WiFi.mode(WIFI_STA);
  //WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(mac_default, password, 1);
  // delay500();
  /*delay500();
  //Connect_WiFi();
  Serial.print("Connecting to:");
  Serial.println(ssid1);
  WiFi.begin(ssid1, password1);
  WiFi.setSleep(false);
  delay500();
  if (WiFi.status() == WL_CONNECTED) {
    WiFi_Connected();
  }*/
  startCameraServer();
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  server.begin();
  //SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println(":----------------------------:");
  Serial.println("Police-Car Wifi-Server");
  Serial.print("wifi_Name:");
  Serial.println(mac_default);
  Serial.print("wifi_Password:");
  Serial.println(password);
  Serial.print("Use 'http://");
  Serial.print(WiFi.softAPIP());
  Serial.println(":100'");
  Serial.println("To Access the Car Controller or");
  Serial.print("'http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("'");
  Serial.println("To Access the Cam Controller");
  Serial.println(":----------------------------:");

  Serial2.print("{[wifi_AP_name] ");
  Serial2.print(mac_default);
  Serial2.print(", [wifi_Password] ");
  Serial2.print(password);
  Serial2.print(", [IP] http://");
  Serial2.print(WiFi.softAPIP());
  Serial2.println(":100}");
  Connect_WiFi();
}

/**
 * Read the Returned Response from the Web Client.
 */
void CameraWebServer_AP::readResponse(WiFiClient* client) {
  String readBuff;
  String wifiIp;
  String line;
  String newline = "";
  bool readit = true;
  bool add = false;
  bool showController = true;
  unsigned long timeout = millis();

  while (client->available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      Serial2.println(">>> Client Timeout !");
      client->stop();
      return;
    }
  }
  // Read all the lines of the reply from server and print them to Serial
  while (client->available()) {
    line = client->readStringUntil('\r');
    readBuff += line;
    //Serial.print(line);
    if (readit) {
      readit = false;
      uint8_t j = 0;
      for (uint8_t i = 0; i < line.length(); i++) {
        if (line[i] == '?') {
          //if (strcmp(line[i], '?') == 0) {
          //line = client->readStringUntil('{');
          //Serial.println(line);
          if (line[i + 1] == 'c') {
            //if (!line[i + 1].equals("c")) {
            showController = false;
          }
        }  // .c_str()
        if (add) {
          if (line[i] == ':') {
            if (line[i + 1] != '"' && line[i + 2] != '"') {
              newline += '"';
            }
          }
          newline += line[i];
          if (line[i] == ',') {
            if (line[i + 1] != '"' && line[i + 2] != '"') {
              newline += '"';
            }
          }
        }
        if (line[i] == '{') {
          add = true;
          newline += line[i];
          if (line[i + 1] != '"' && line[i + 2] != '"' && line[i + 1] != '}' && line[i + 2] != '}') {
            newline += '"';
          }
          //if (strcmp(line[i], '{') == 0) {
          j = i;
        }
        if (line[i] == '}') {
          add = false;
          //if (strcmp(line[i], '}') == 0) {
          Serial.println(newline);
          Serial2.println(newline);
          //Serial.println(line.substring(j, i + 1));
          //Serial2.println(line.substring(j, i + 1));
          newline = "";
          break;
        }
      }
      line = client->readStringUntil(':');
      readBuff += line;
      //Serial.print("(1)");
      //Serial.println(line);
      line = client->readStringUntil(' ');
      readBuff += line;
      //Serial.print("(2)");
      //Serial.println(line);
      line = client->readStringUntil(':');
      wifiIp = line;
      readBuff += line;
      if (readBuff.indexOf("disconnectWifi") > -1) {  //true == readBuff.equals("{disconnectWifi}")
        WiFi.disconnect();
        Serial.println("WiFi disconnected.(WebPage)");
        Serial2.println("WiFi disconnected.(WebPage)");
      } else if (readBuff.indexOf("connectWifi") > -1) {
        String nReadBuff = readBuff.substring(readBuff.indexOf("connectWifi"));
        String wNet = nReadBuff.substring(nReadBuff.indexOf(":") + 1, nReadBuff.indexOf(","));
        String wPass = nReadBuff.substring(nReadBuff.indexOf(",") + 1, nReadBuff.indexOf(";"));
        if (wPass != "") {
          Serial.print("connecting to WiFi: ");
          Serial.print(wNet);
          Serial.print(" ");
          Serial.println(wPass);
          Serial2.print("connecting to WiFi: ");
          Serial2.print(wNet);
          Serial2.print(" ");
          Serial2.println(wPass);
          WiFi.begin(wNet, wPass);
          uint8_t i = 0;
          while (WiFi.status() != WL_CONNECTED) {
            delay500();
            if (WiFi.status() == WL_CONNECTED) {
              WiFi_Connected();
              break;
            }
            i++;
            if (i >= 15) {
              Serial.println("Connection to WiFi not Detected.");
              Serial2.println("Connection to WiFi not Detected.");
              break;
            }
          }
        }
      }
      /*if (wifiIp != "") {
        Serial.print("IP:");
        Serial.println(wifiIp);
      }*/
      // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)'Access-Control-Allow-Origin': '*'
      // and a content-type so the client knows what's coming, then a blank line:Access-Control-Allow-Origin
      if (showController) {
        client->println("HTTP/1.1 200 OK");
        client->println("Content-type: text/html");
        client->println("Access-Control-Allow-Origin: *");
        client->println("Cache-Control: max-age=10");
        client->println("Upgrade-Insecure-Requests: 1");
        client->println("Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.7");
        client->println("Accept-Encoding: gzip, deflate");
        client->println("Accept-Language: en-US,en;q=0.9");
        client->print("Referer: http://");
        client->println(wifiIp);
        //client->println("Connection: close");
        client->println("Connection: keep-alive");
        client->println();
        client->println();
        // the content of the HTTP response follows the header
        client->print("<!DOCTYPE html>");
        client->print("<html charset='utf-8' lang='en-us'>");
        client->print("<head>");
        client->print("<meta content='text/html; charset=UTF-8' http-equiv='content-type' />");
        client->print("<meta charset='utf-8' lang='en-us' />");
        client->print("<meta http-equiv='X-UA-Compatible' content='IE=edge' />");
        client->print("<meta name='viewport' content='width=device-width, initial-scale=1.0, minimum-scale=0.5, maximum-scale=2.0, user-scalable=1, shrink-to-fit=yes' />");
        client->print("<meta uiactions='zoom' />");
        client->print("<title>Police Car</title>");
        // style
        client->print("<style> body { height: 100%; width: 100%; margin: 0; padding: 0; background: url('http://");
        client->print(wifiIp);
        client->print(":81/stream') no-repeat center; background-size: cover; } #myDIV { background-color: transparent; margin: 30px; } ");
        client->print("label { cursor: pointer; }  label:hover { box-shadow: 1px 1px 2px rgba(0,0,0,0.09); } ");
        client->print("code { border-radius: 5px; border: 1px solid black; padding: 4px 0 4px 6px; }  code button { cursor: pointer; font-size: 10px; width: 20px; height: 16px } ");
        client->print("#settingsBar { z-index: 99; height: fit-content; padding-bottom: 4px; border-radius: 5px; background: border-box; background-color: darkgray; width: fit-content; margin-top:10px; } #settingsBar * { box-sizing: border-box; box-shadow: 1px 2px 5px rgba(0,0,0,0.7); } #settingsBar label { height: 18px; width: fit-content; font-size: 20px; } #settingsBar label:hover { box-shadow: 1px 1px 2px rgba(0,0,0,0.09);} ");
        client->print("#panel { position: absolute; background: grey; z-index: 100; top: 50px; right: 30px; left: 30px; margin: auto; padding: 10px; height: fit-content; width: fit-content; } #panel * { box-sizing: border-box; box-shadow: 1px 2px 5px rgba(0,0,0,0.7); } #panel #xPanelBtn { height: fit-content; width: fit-content; font-size: 10px; margin: 0 0 10px; color: red; float: right;} ");
        client->print("#myDIV div { background-color: transparent; text-align: center; padding: 0; margin: auto; font-size: 30px; object-fit: fill; width: fit-content; height: fit-content; box-sizing: border-box; box-shadow: 1px 2px 5px rgba(0,0,0,0.7); } @media screen and (min-width: 512px) {  #myDIV #leftS { float: left; }  #myDIV #rightS { float: right; } }");
        client->print("#myDIV #leftS { border-radius: 50%; padding: 4px; border: 10px solid darkgrey; margin-top: 10px; } #myDIV #rightS { border-radius: 25px; margin-top: 10px; } #myDIV button { background-color: rgba(255, 255, 255, 0.8); text-align: center; padding: 0 0 0 0; margin: 0 0 0 0; min-width: 45px; min-height: 45px; max-width: 45px; max-height: 45px; font-size: 30px; cursor: pointer; border-radius: 5px; border: 1px solid #ccc; box-sizing: border-box; box-shadow: 1px 2px 5px rgba(0,0,0,0.7); } #myDIV button:hover { box-shadow: 1px 1px 2px rgba(0,0,0,0.09); } #myDIV .button2 { border-radius: 50%; } ");
        client->print("</style>");
        client->print("</head> <body> <div align='center'>");
        // settingsBar
        client->print("<div id='settingsBar'>");
        client->print(" &nbsp; <label for='hLight' title='headlights'>&#128161;</label> <button id='hLight' title='Lights' onclick='runFun(this.id)' hidden>&#128161;</button>");
        client->print(" &nbsp; <code> <button id='dimmer' onclick='runFun(this.id)'>-</button> <sub style='cursor: default; box-shadow: 1px 1px 2px rgba(0,0,0,0.09);'><label id='led' title='LED lights' onclick='runFun(this.id)'>&#128680;</label></sub> <button id='brighter' onclick='runFun(this.id)'>+</button> </code>");
        client->print(" &nbsp;&nbsp; <label for='openPanel' title='Settings'>&#128262;</label> <button id='openPanel' name='panel' onclick='document.getElementById(this.name).hidden=!document.getElementById(this.name).hidden' hidden>&#128262;</button>");
        client->print(" &nbsp;&nbsp; <label id='s' for='standby' title='Mode'>&#127918;</label> <label id='l' for='standby' title='Mode' hidden>&#9945;</label> <label id='a' for='standby' title='Mode' hidden>&#128064;</label> <label id='f' for='standby' title='Mode' hidden>&#128660;</label><button id='standby' name='standby' onclick='runFun(this.id)' title='Mode' hidden>S</button>");
        client->print(" &nbsp; </div>");
        // Advanced Settings panel
        client->print("<div id='panel' hidden>");
        // close button
        client->print("<button id='xPanelBtn' title='panel' onclick='document.getElementById(this.title).hidden=!document.getElementById(this.title).hidden'>X</button><hr /><hr />");
        // Video Window Size Value Apply button
        client->print("<label>Window Size</label><br /> ");
        client->print("<input type='range' onchange='runFun(this.id);document.getElementById(this.title).innerText=this.value' title='framesize:0-12' placeholder='Size:0-12' id='framesize' name='framesize' max='12' min='0' value='12' style='width:100px;text-align:center;'><label id='framesize:0-12'>12</label> <hr /> ");
        // Video Quality Value Apply button
        client->print("<label>Quality</label><br /> ");
        client->print("<input type='range' onchange='runFun(this.id);document.getElementById(this.title).innerText=this.value' title='quality:0-20' placeholder='quality:0-20' id='quality' name='qualitysize' max='20' min='0' value='20' style='width:100px;text-align:center;'><label id='quality:0-20'>20</label> <hr /> ");
        // Vertical flip checkbox
        client->print("<input type='checkbox' id='vflip' onclick='runFun(this.id)' /> <label for='vflip'>vflip</label> ");
        // Horizontal flip checkbox
        client->print("<input type='checkbox' id='hmirror' onclick='runFun(this.id)' /> <label for='hmirror'>hmirror</label> <hr /> ");
        // Motor Speed Value input
        client->print("<label for='speed' style='margin:0;font-size:18px;color:red;'>Speed:</label><label id='Speed:0-250'>200</label><br /> ");
        client->print("<input id='speed' type='range' title='Speed:0-250' placeholder='Speed:0-250' name='speed' max='250' min='0' value='200' style='width:100px;margin-top:8px;text-align:center;' onchange='document.getElementById(this.title).innerText=this.value'>");

        client->print(" </div> <div id='myDIV'><input id='direction' type='number' value='0' readonly hidden />");
        // left buttons Steering
        //client->print("");
        client->print("<div id='police-car' style='width: 100%;'>");

        client->print("<div id='leftS'><button style='border:unset;box-shadow:unset;background-color:transparent;'></button><br /><button id='left' style='min-width: 38px;' onclick='runFun(this.id)'>&#128659;</button> &nbsp; <button id='straight' onclick='runFun(this.id)'>&#128660;</button> &nbsp; <button id='right' style='transform: rotateY(180deg); min-width: 38px;' onclick='runFun(this.id)'>&#128659;</button><br /><button style='border:unset;box-shadow:unset;background-color:transparent;'></button></div>");

        client->print("<div id='rightS' style='display: grid; width: fit-content; height: fit-content; align-content: center; grid-template-columns: auto; gap: 30px; padding: 10px;'>");
        client->print("<button id='forward' style='background-color:green;' class='button2' onclick='runFun(this.id)'>&#8679;</button>");
        client->print("<button id='stop' style='background-color:red;' class='button2' onclick='runFun(this.id)'>&#128721;</button>");
        client->print("<button id='backward' style='background-color: lightsteelblue;' class='button2' onclick='runFun(this.id)'>&#8681;</button>");
        client->print("</div>");  // div id='rightS'

        client->print("</div>");  // div id='police-car'

        client->print("<div id='bunny' style='width: 100%;' hidden>");
        client->print("<div id='leftS' style='padding-top: 0;'><button style='border:unset;box-shadow:unset;background-color:transparent;margin-top: 0;'></button> &nbsp; <button id='forward' style='margin-top: 0;margin-bottom: 10px;' onclick='runFun(this.id)'>&#8593;</button> &nbsp; <button style='border:unset;box-shadow:unset;background-color:transparent;margin-top: 0;'></button><br /><button id='l' style='min-width: 38px;' onclick='runFun(this.id)'>&#8592;</button> &nbsp; <button id='stop' onclick='runFun(this.id)'>&#128721;</button> &nbsp; <button id='r' style='min-width: 38px;' onclick='runFun(this.id)'>&#8594;</button><br /><button style='border:unset;box-shadow:unset;background-color:transparent;'></button> &nbsp; <button id='backward' onclick='runFun(this.id)'>&#8595;</button> &nbsp; <button style='border:unset;box-shadow:unset;background-color:transparent;'></button></div>");
        client->print("<div id='rightS'><button id='up' style='background-color: lightsteelblue;' class='button2' onclick='runFun(this.id)'>A</button> &nbsp; <button id='down' style='background-color:red;' class='button2' onclick='runFun(this.id)'>B</button><br /><br /><button id='left' style='background-color:green;' class='button2' onclick='runFun(this.id)'>X</button> &nbsp; <button id='right' style='background-color: gold;' class='button2' onclick='runFun(this.id)'>Y</button></div>");
        client->print("</div>");

        client->print("</div><script>const POLICECAR = {");
        client->print("LED: '?c{N:105,D1:4,H:7}',");
        client->print("LEDUP: '?c{N:105,D1:1,H:6}',");
        client->print("LEDDN: '?c{N:105,D1:2,H:5}',");
        client->print("HEADLIGHT: '?c{N:105,D1:3,H:4}',");
        client->print("MODE0STANDBY: '?c{N:100,H:0}',");
        client->print("MODE1LINETRACK: '?c{N:101,D1:1,H:1}',");
        client->print("MODE2FREEROAM: '?c{N:101,D1:2,H:2}',");
        client->print("MODE3FOLLOW: '?c{N:101,D1:3,H:3}',");
        client->print("SLEFT: '?c{N:106,D1:1,H:8}',");
        client->print("SRIGHT: '?c{N:106,D1:2,H:9}',");
        client->print("SUP: '?c{N:106,D1:3,H:10}',");
        client->print("SDOWN: '?c{N:106,D1:4,H:11}',");
        client->print("SCENTER: '?c{N:106,D1:5,H:12}',");
        client->print("STOP: '?c{N:100,H:0}',");  // '?c{N:102,D1:9,H:13}'
        client->print("F: '?c{H:21,N:102,D1:1,D2:',");
        client->print("B: '?c{H:22,N:102,D1:2,D2:',");
        client->print("L: '?c{H:23,N:102,D1:3,D2:',");
        client->print("R: '?c{H:24,N:102,D1:4,D2:',");
        client->print("LF: '?c{H:25,N:102,D1:5,D2:',");
        client->print("LB: '?c{H:26,N:102,D1:6,D2:',");
        client->print("RF: '?c{H:27,N:102,D1:7,D2:',");
        client->print("RB: '?c{H:28,N:102,D1:8,D2:'");
        client->print("};</script>");
        // Script Functions
        client->print("<script>function sBtnChange(lbl) { if (lbl == 's') { document.getElementById('s').hidden = false; document.getElementById('l').hidden = true; document.getElementById('a').hidden = true; document.getElementById('f').hidden = true; } else if (lbl == 'l') { document.getElementById('s').hidden = true; document.getElementById('l').hidden = false; document.getElementById('a').hidden = true; document.getElementById('f').hidden = true; } else if (lbl == 'a') { document.getElementById('s').hidden = true; document.getElementById('l').hidden = true; document.getElementById('a').hidden = false; document.getElementById('f').hidden = true; } else if (lbl == 'f') { document.getElementById('s').hidden = true; document.getElementById('l').hidden = true; document.getElementById('a').hidden = true; document.getElementById('f').hidden = false; }; }</script>");
        client->print("<script>function valcheck(value) {if (value > 250) {document.getElementById('speed').value = '250';} else if (value < 0) {document.getElementById('speed').value = '0';}}</script>");
        client->print("<script>function rot(d) { if (document.getElementById('direction').value != d) { document.getElementById('direction').value = d; document.getElementById('leftS').style ='transform:rotate('+d+'deg)'; } } function runFun(id) {valcheck(document.getElementById('speed').value);switch (id) {");
        // Movement
        client->print("case 'stop':document.getElementById('controllers').src=POLICECAR.STOP;break;");
        client->print("case 'forward':document.getElementById('controllers').src=POLICECAR.F + document.getElementById('speed').value + '}';break;");
        client->print("case 'backward':document.getElementById('controllers').src=POLICECAR.B + document.getElementById('speed').value + '}';break;");
        client->print("case 'l':document.getElementById('controllers').src=POLICECAR.L + document.getElementById('speed').value + '}';break;");
        client->print("case 'r':document.getElementById('controllers').src=POLICECAR.R + document.getElementById('speed').value + '}';break;");

        // * Motor Steer
        // client->print("case 'left':document.getElementById('controllers').src='?C{N:102,D1:3,D2:' + document.getElementById('speed').value + '}';rot('-45');break;");
        // client->print("case 'right':document.getElementById('controllers').src='?c{N:102,D1:4,D2:' + document.getElementById('speed').value + '}';rot('45');break;");
        // * Servo Steer L: 1 & 3, R: 2 & 4
        client->print("case 'left':document.getElementById('controllers').src=POLICECAR.SLEFT;rot('-30');break;");
        client->print("case 'right':document.getElementById('controllers').src=POLICECAR.SRIGHT;rot('30');break;");
        client->print("case 'up':document.getElementById('controllers').src=POLICECAR.SUP;break;");
        client->print("case 'down':document.getElementById('controllers').src=POLICECAR.SDOWN;break;");
        client->print("case 'straight':document.getElementById('controllers').src=POLICECAR.SCENTER;rot('0');break;");
        // Modes
        client->print("case 'standby': if (document.getElementById('standby').innerText == 'F') { document.getElementById('standby').innerText = 'S'; sBtnChange('s'); document.getElementById('controllers').src = POLICECAR.MODE0STANDBY; } else if (document.getElementById('standby').innerText == 'S') { document.getElementById('controllers').src = POLICECAR.MODE1LINETRACK; document.getElementById('standby').innerText = 'L'; sBtnChange('l'); } else if (document.getElementById('standby').innerText == 'L') { document.getElementById('controllers').src = POLICECAR.MODE2FREEROAM; document.getElementById('standby').innerText = 'A'; sBtnChange('a'); } else if (document.getElementById('standby').innerText == 'A') { document.getElementById('controllers').src = POLICECAR.MODE3FOLLOW; document.getElementById('standby').innerText = 'F'; sBtnChange('f'); } break;");
        client->print("case 'line':document.getElementById('controllers').src=POLICECAR.MODE1LINETRACK;break;");
        client->print("case 'avoid':document.getElementById('controllers').src=POLICECAR.MODE2FREEROAM;break;");
        client->print("case 'follow':document.getElementById('controllers').src=POLICECAR.MODE3FOLLOW;break;");
        // Lights
        client->print("case 'hLight':document.getElementById('controllers').src=POLICECAR.HEADLIGHT;break;");
        client->print("case 'led':document.getElementById('controllers').src=POLICECAR.LED;break;");
        client->print("case 'brighter':document.getElementById('controllers').src=POLICECAR.LEDUP;break;");
        client->print("case 'dimmer':document.getElementById('controllers').src=POLICECAR.LEDDN;break;");
        // Advanced Settings.
        client->print("case 'framesize':document.getElementById('controllers2').src='http://");
        client->print(wifiIp);
        client->print(":80/control?var=framesize&val=' + document.getElementById('framesize').value;break;");
        client->print("case 'quality': document.getElementById('controllers2').src='http://");
        client->print(wifiIp);
        client->print(":80/control?var=quality&val=' + document.getElementById('quality').value;break;");
        client->print("case 'vflip': if (document.getElementById('vflip').checked) {document.getElementById('controllers2').src='http://");
        client->print(wifiIp);
        client->print(":80/control?var=vflip&val=1'} else {document.getElementById('controllers2').src='http://");
        client->print(wifiIp);
        client->print(":80/control?var=vflip&val=0'} break;");
        client->print("case 'hmirror': if (document.getElementById('hmirror').checked) {document.getElementById('controllers2').src='http://");
        client->print(wifiIp);
        client->print(":80/control?var=hmirror&val=1'} else {document.getElementById('controllers2').src='http://");
        client->print(wifiIp);
        client->print(":80/control?var=hmirror&val=0'} break;");


        client->print("default:document.getElementById('controllers').src=POLICECAR.MODE0STANDBY;break;}}</script>");
        //client->print(wifiIp);
        client->print("<iframe id='controllers' src='/?c{}' style='display:none;' hidden></iframe><iframe id='controllers2' src='http://");
        client->print(wifiIp);
        client->print(":80/control?var=framesize&val=12' style='display:none;' hidden></iframe>");
        client->print("<button hidden style='width:30px;' name='bunny' title='police-car' onclick='document.getElementById(this.name).hidden=!document.getElementById(this.name).hidden;document.getElementById(this.title).hidden=!document.getElementById(this.title).hidden'>^</button></div></body></html>");
        // The HTTP response ends with another blank line:
        client->println();
      } else {
        client->println("HTTP/1.1 200 OK");
        client->println("Access-Control-Allow-Origin:*");
        client->println("Content-type:text/html");
        client->println();
        // The HTTP response ends with another blank line:
        client->println();
        //client->print("HTTP/1.1\r\nHost:");
        //client->print(wifiLocalIp);
        //client->println("\r\nConnection:close\r\n\r\n");
        //client->print("HTTP/1.1\r\nHost:ToremetalApps\r\nConnection:close\r\n\r\n");
      }
    } else {
      client->flush();
    }
  }

  Serial.printf("");
  Serial2.printf("");
}

void CameraWebServer_AP::SocketServer_Test(void) {
  static bool ED_client = true;
  //if (SerialBT.available()) {
  //Serial.write(SerialBT.read());
  //}
  WiFiClient client = server.available();
  if (client) {
    WA_en = true;
    //ED_client = true;
    Serial.println("[Client connected]");
    //Serial2.println("[Client connected]");
    static String readBuff;
    uint8_t Heartbeat_count = 0;
    bool Heartbeat_status = false;
    bool data_begin = true;
    //bool wifi_client = false;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == 'G') {
          // if G: local connection filter headers to extract cmd.
          c = '\n';
          readResponse(&client);
        }  // else {
           // Wifi connection: No modification needed.
           //wifi_client = true;
           //}
           //Serial.print(c);
        if (true == data_begin && c == '{') {
          readBuff = "";
          data_begin = false;
        }
        if (false == data_begin && c != ' ') {
          readBuff += c;
        }
        if (false == data_begin && c == '}') {
          data_begin = true;
          if (true == readBuff.equals("{Heartbeat}")) {
            Heartbeat_status = true;
          } else {
            Serial.print(readBuff);
            Serial2.print(readBuff);
          }
          //Serial2.print(readBuff);
          readBuff = "";
        }
      }
      /*if (Serial2.available()) {
        static String sendBuff;
        char c = Serial2.read();
        sendBuff += c;
        if (c == '}') {
          client.print(sendBuff);
          Serial.print(sendBuff);
        }
      }*/

      static unsigned long Heartbeat_time = 0;
      if (millis() - Heartbeat_time > 1000) {
        //client.print("{Heartbeat}");
        if (true == Heartbeat_status) {
          Heartbeat_status = false;
          Heartbeat_count = 0;
        } else if (false == Heartbeat_status) {
          Heartbeat_count += 1;
        }
        if (Heartbeat_count > 3) {
          Heartbeat_count = 0;
          Heartbeat_status = false;
          break;
        }
        Heartbeat_time = millis();
      }
      static unsigned long Test_time = 0;
      if (millis() - Test_time > 1000) {
        Test_time = millis();
        //Serial2.println(WiFi.softAPgetStationNum());
        //if (wifi_client) {
        //Serial2.print("{\"N\":100,\"H\":\"STANDBY\"}");
        break;
        //}
      }
    }
    /*if (wifi_client) {
      if (0 != (WiFi.softAPgetStationNum())) {
        Serial2.print("{\"N\":100,\"H\":\"STANDBY\"}");
      }
    }*/
    client.flush();
    client.stop();
    //Serial.println();
    //Serial.println("{\"N\":100,\"H\":\"STANDBY\"}");
    //Serial2.print("{\"N\":100,\"H\":\"STANDBY\"}");
    Serial.println("[Client disconnected]");
  } else {
    if (ED_client == true) {
      Serial.println("Connect to WiFi: connectWifi:Wifi,Password;");
      ED_client = false;
    }
    static String readBuff;
    if (Serial.available()) {
      while (Serial.available()) {
        char c = Serial.read();
        readBuff += c;
        //Serial.println(readBuff);
        if (readBuff.indexOf("connectWifi:") > -1) {
          String wNet = Serial.readStringUntil(',');
          Serial.read();
          String wPass = Serial.readStringUntil(';');
          if (wPass != "") {
            Serial.print("connecting to WiFi: ");
            Serial.print(wNet);
            Serial.print(" ");
            Serial.println(wPass);
            readBuff = "";
            WiFi.begin(wNet, wPass);
            uint8_t i = 0;
            while (WiFi.status() != WL_CONNECTED) {
              delay500();
              if (WiFi.status() == WL_CONNECTED) {
                WiFi_Connected();
                break;
              }
              i++;
              if (i >= 15) {
                Serial.println("Connection to WiFi not Detected.");
                break;
              }
            }
          }
        } else if (readBuff.indexOf("disconnectWifi") > -1) {
          readBuff = "";
          WiFi.disconnect();
          Serial.println("WiFi disconnected.(ESP32_Serial)");
        }
      }
    } else if (Serial2.available()) {
      while (Serial2.available()) {
        char c = Serial2.read();
        readBuff += c;
        if (readBuff.indexOf("connectWifi:") > -1) {
          String wNet = Serial2.readStringUntil(',');
          Serial2.read();
          String wPass = Serial2.readStringUntil(';');
          if (wPass != "") {
            Serial2.print("connecting to WiFi: ");
            Serial2.print(wNet);
            Serial2.print(" ");
            Serial2.println(wPass);
            readBuff = "";
            WiFi.begin(wNet, wPass);
            uint8_t i = 0;
            while (WiFi.status() != WL_CONNECTED) {
              delay500();
              if (WiFi.status() == WL_CONNECTED) {
                WiFi_Connected();
                break;
              }
              i++;
              if (i >= 15) {
                Serial2.println("Connection to WiFi not Detected.");
                break;
              }
            }
          }
        } else if (readBuff.indexOf("disconnectWifi") > -1) {
          readBuff = "";
          WiFi.disconnect();
          Serial2.println("WiFi disconnected.(2)");
        }
      }
    }
    //if (ED_client == true) {
    //  ED_client = false;
    //delay(700);
    //Serial.println("{\"N\":100,\"H\":\"STANDBY\"}");
    //Serial2.print("{\"N\":100,\"H\":\"STANDBY\"}");
    //}
  }
}

void CameraWebServer_AP::FactoryTest(void) {
  //static String readBuff;
  // String sendBuff;
  //if (Serial2.available()) {
  //char c = Serial2.read();
  // readBuff += c;
  // if (c == '}') {
  //  if (true == readBuff.equals("{BT_detection}")) {
  //Serial2.print("{BT_OK}");
  //Serial.println("Factory...");
  // } else if (true == readBuff.equals("{WA_detection}")) {
  //Serial2.print("{");
  //Serial2.print(wifi_name);  //CameraWebServerAP.wifi_name);
  //Serial2.print("}");
  //Serial.println("Factory...");
  //}
  // readBuff = "";
  // }
  //}
  //{
  if ((WiFi.softAPgetStationNum())) {
    if (true == WA_en) {
      digitalWrite(13, HIGH);
      // Serial.println("{WA_OK}");
      // Serial2.println("{WA_OK}");
      WA_en = false;
    }
  } else {
    //timestamp
    static unsigned long Test_time;
    static bool en = true;
    if (millis() - Test_time > 100) {
      if (false == WA_en) {
        // Serial.println("{WA_NO}");
        // Serial2.println("{WA_NO}");
        WA_en = true;
      }
      if (en == true) {
        en = false;
        digitalWrite(13, HIGH);
      } else {
        en = true;
        digitalWrite(13, LOW);
      }
      Test_time = millis();
    }
  }
  //}
}
