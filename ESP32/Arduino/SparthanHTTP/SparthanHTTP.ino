#include <WiFi.h>
#include <map>
#include <HTTPClient.h>

#define USE_SERIAL Serial


const char* SSID = "SPARTHAN";
const char* PASS = "66668888";
const String HOST = "192.168.137.1";
const uint16_t PORT = 8081;
const uint16_t DATA_SIZE = 5 * sizeof(float); 

std::map<String,float*> gestures2positions;


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
  
  std::map<std::string,float*> gestures2positions;
  float pPalm[] =   {0., 0., 0., 0., 0.};
  float pFist[] =   {1., 1., 1., 1., 1.};
  float pThumb[] =  {0., 1., 1., 1., 1.};
  float pPoint[] =  {1., 0., 1., 1., 1.};
  float pMidfng[] = {1., 1., 0., 1., 1.};
  gestures2positions["PALM"] = pPalm;
  gestures2positions["FIST"] = pFist;
  gestures2positions["THUMB"] = pThumb;
  gestures2positions["POINT"] = pPoint;
  gestures2positions["MIDFNG"] = pMidfng;
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
        Serial.println(cmd);
        float *data = gestures2positions[cmd];
        uint16_t frameID = (uint16_t)  0x0004;
        //uartSendBuffer(data, DATA_SIZE, &frameID);
      }
    } else {
      USE_SERIAL.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
  }

  delay(500);
}
