#include <WiFi.h>

#include <HTTPClient.h>

#define USE_SERIAL Serial


const char* SSID = "SPARTHAN";
const char* PASS = "66668888";
const String HOST = "192.168.137.1";
const uint16_t PORT = 8081;

void setup() {

  Serial.begin(115200);
  //  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  Serial.print("Wait for WiFi...");
  while (WiFi.status()!= WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(500);
}

void loop() {
  if ((WiFi.status() == WL_CONNECTED)) {
    HTTPClient http;
    http.begin("http://" + HOST + "/");
    int httpCode = http.GET();
    if (httpCode > 0) {
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        String cmd = payload;
        if (String("PALM") == cmd) {
          Serial.println("PALM");
        }
        else if (String("FIST") == cmd) {
          Serial.println("FIST");
        }
        else if (String("THUMB") == cmd) {
          Serial.println("THUMB");
        }
        else if (String("POINT") == cmd) {
          Serial.println("POINT");
        }
        else if (String("MIDFNG") == cmd) {
          Serial.println("MIDFNG");
        }
      }
    } else {
      USE_SERIAL.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
  }

  delay(500);
}
