#include <HardwareSerial.h>
#include "myo.h"

#define LED           32U
#define BOOT          18U
#define RST           19U

myo myo;
HardwareSerial sparthan(2);           // Initialize sparthan communication

/********************************************************************************************************
    SET CALLBACKS WHEN RECEIVING DATA
 ********************************************************************************************************/

void batteryCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  myo.battery = pData[0];
  Serial.print("Battery: ");
  Serial.println(myo.battery);
}

void imuCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  Serial.print ("EMG: \t");
  for (int i = 0; i < length; i++) {
    Serial.print(pData[i]);
    Serial.print("\t");
  }
  Serial.println(millis());
}

void emgCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  Serial.print ("IMU: \t");
  for (int i = 0; i < length; i++) {
    Serial.print(pData[i]);
    Serial.print("\t");
  }
  Serial.println(millis());
}

void gestureCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  // Print the gesture
  if (pData[0] == myohw_classifier_event_pose) {
    Serial.print ("Gesture: ");
    switch (pData[1]) {
      case myohw_pose_rest:
        Serial.println ("rest");
        sparthan.print("t");
        break;
      case myohw_pose_fist:
        Serial.println ("fist");
        break;
      case myohw_pose_wave_in:
        Serial.println ("wave in");
        sparthan.print("n");
        digitalWrite (LED, LOW);
        break;
      case myohw_pose_wave_out:
        Serial.println ("wave out");
        sparthan.print("s");
        digitalWrite (LED, HIGH);
        break;
      case myohw_pose_fingers_spread:
        Serial.println ("fingers spread");
        break;
      case myohw_pose_double_tap:
        Serial.println ("double tap");
        break;
      default:
        break;
    } 
  }
}

/********************************************************************************************************
    ARDUINO SKETCH
 ********************************************************************************************************/

void setup()
{
      
  pinMode(BOOT, OUTPUT);
  pinMode(RST, OUTPUT);
  digitalWrite(BOOT, LOW);
  digitalWrite(RST, LOW);
  delay(100);
  pinMode(RST, INPUT);
  delay(100);

  
  pinMode (LED, OUTPUT);                            // Notification LED
  digitalWrite (LED, HIGH);                         // Turn ON
  delay(100);
  sparthan.begin(3000000, SERIAL_8N1, 16, 17);
  delay(100);
  Serial.begin(115200);                             // Start serial interface for debugging
  Serial.println ("Connecting...");
  myo.connect();                                    // Connect to the myo
  Serial.println (" - Connected");
  delay(100);
  myo.set_myo_mode(myohw_emg_mode_send_emg,         // EMG mode
                   myohw_imu_mode_none,             // IMU mode
                   myohw_classifier_mode_enabled);  // Classifier mode

  if (myo.connected) {
    myo.get_firmware();                             // Retrieve and print the firmware info
    delay(100);
    Serial.print("Firmware info: ");
    Serial.print(myo.fw_major);
    Serial.print(".");
    Serial.print(myo.fw_minor);
    Serial.print(".");
    Serial.println(myo.fw_patch);
    Serial.print("Color: ");
    Serial.println(myo.fw_hardware_rev);
  }

  myo.battery_notification(TURN_ON)->registerForNotify(batteryCallback);
  myo.gesture_notification(TURN_ON)->registerForNotify(gestureCallback);
}

void loop()
{
  // Detect disconnection
  if (!myo.connected) {
    Serial.println ("Device disconnected: reconnecting...");
    myo.connect();
    Serial.println (" - Connected");
    myo.set_myo_mode(myohw_emg_mode_send_emg,               // EMG mode
                     myohw_imu_mode_none,                   // IMU mode
                     myohw_classifier_mode_enabled);        // Classifier mode
    myo.battery_notification(TURN_ON)->registerForNotify(batteryCallback);
    myo.gesture_notification(TURN_ON)->registerForNotify(gestureCallback);
  }
  delay (1000);
}
