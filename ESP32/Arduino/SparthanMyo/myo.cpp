
#include "Arduino.h"
#include "myo.h"

/* ENABLE SERIAL DEBUGGING BY SETTING myo.debug = true, **REQUIRES Serial.begin()** */

// The remote service we wish to connect to.
static BLEUUID serviceUUID("d5060001-a904-deb9-4748-2c7f4a124842");
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAddress *pServerAddress;

// Initialize status variables
boolean myo::connected = false;
boolean myo::detected = false;
boolean myo::debug = false;

/********************************************************************************************************
    BLUETOOTH CALLBACKS
 ********************************************************************************************************/

// Scan for BLE servers and find the first one that advertises the Myo ID service
class MyoAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {

    // Called for each advertising BLE server.
    void onResult(BLEAdvertisedDevice advertisedDevice) {

      myo::debug ? Serial.print("BLE Advertised Device found: ") : 0;
      myo::debug ? Serial.println(advertisedDevice.toString().c_str()) : 0;

      // We have found a Myo, check if it contains the service ID we are looking for.
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {

        myo::debug ? Serial.print(" - Myo device found") : 0; 
        // Stop scanning
        advertisedDevice.getScan()->stop();
        // Save the address of the current Myo
        pServerAddress = new BLEAddress(advertisedDevice.getAddress());
        // Update detection status
        myo::detected = true;
      }
    }
};

// Set the connect and disconnect behaviours
class MyoClientCallbacks : public BLEClientCallbacks {

    void onConnect(BLEClient* pClient) {
      // Update connection status
      myo::connected = true;
    }
    void onDisconnect(BLEClient* pClient) {
      // Update connection status
      myo::connected = false;
      // Update detection status
      myo::detected = false;
      // Disconnect the clinet
      pClient->disconnect();
      // Clean the BLE stack for a new connection
      delete pClient;
    }
};

/********************************************************************************************************
    CONNECTION
 ********************************************************************************************************/

bool connectToServer(BLEClient*  pClient, BLEAddress pAddress) {
  myo::debug ? Serial.print("Forming a connection to ") : 0;
  myo::debug ? Serial.println(pAddress.toString().c_str()): 0;

  // Connect to the remove BLE Server.
  pClient->connect(pAddress);

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    myo::debug ? Serial.print("Failed to find our service UUID: ") : 0;
    myo::debug ? Serial.println(serviceUUID.toString().c_str()): 0;
    return false;
  }
  myo::debug ? Serial.println(" - Found our service"): 0;
}

void myo::connect() {
  if (!myo::connected) {

    // Initialize BLE scan
    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan();
    // Set callbacks for finding devices
    pBLEScan->setAdvertisedDeviceCallbacks(new MyoAdvertisedDeviceCallbacks());
    // Active scan uses more power, but get results faster
    pBLEScan->setActiveScan(true);
    
    // Keep scanning untill a Myo is detected
    while (!myo::detected) {
      pBLEScan->start(10);
    }
    // Create new client and set up callbacks
    myo::pClient = BLEDevice::createClient();
    myo::pClient->setClientCallbacks(new MyoClientCallbacks());

    // Attempt Server connection
    connectToServer(myo::pClient, *pServerAddress);

  }
}


/********************************************************************************************************
    INFO
 ********************************************************************************************************/

void myo::get_info() {
  if (myo::connected) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060101-a904-deb9-4748-2c7f4a124842");
    std::string stringt;
    stringt = myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic)->readValue();
    myo::fw_serial_number[0]        = stringt[0];
    myo::fw_serial_number[1]        = stringt[1];
    myo::fw_serial_number[2]        = stringt[2];
    myo::fw_serial_number[3]        = stringt[3];
    myo::fw_serial_number[4]        = stringt[4];
    myo::fw_serial_number[5]        = stringt[5];
    myo::fw_serial_number[6]        = stringt[6];
    myo::fw_unlock_pose             = (byte)stringt[8] * 256 + (byte)stringt[7];
    myo::fw_active_classifier_type  = stringt[9];
    myo::fw_active_classifier_index = stringt[10];
    myo::fw_has_custom_classifier   = stringt[11];
    myo::fw_stream_indicating       = stringt[12];
    myo::fw_sku                     = stringt[13];
    myo::fw_reserved[0]             = stringt[14];
    myo::fw_reserved[1]             = stringt[15];
    myo::fw_reserved[2]             = stringt[16];
    myo::fw_reserved[3]             = stringt[17];
    myo::fw_reserved[4]             = stringt[18];
    myo::fw_reserved[5]             = stringt[19];
    myo::fw_reserved[6]             = stringt[20];
    myo::fw_reserved[7]             = stringt[21];
  }
}

void myo::get_firmware() {
  if (myo::connected) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060201-a904-deb9-4748-2c7f4a124842");
    std::string stringt;
    stringt = myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic)->readValue();
    myo::fw_major               = (byte)stringt[1] * 256 + (byte)stringt[0];
    myo::fw_minor               = (byte)stringt[3] * 256 + (byte)stringt[2];
    myo::fw_patch               = (byte)stringt[5] * 256 + (byte)stringt[4];
    myo::fw_hardware_rev        = (byte)stringt[7] * 256 + (byte)stringt[6];
  }
}

/********************************************************************************************************
    NOTIFICATIONS
 ********************************************************************************************************/

BLERemoteCharacteristic* myo::emg_notification(uint8_t on_off) {
  if (myo::connected) {
    BLEUUID tservice = BLEUUID("d5060005-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic0 = BLEUUID("d5060105-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic1 = BLEUUID("d5060205-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic2 = BLEUUID("d5060305-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic3 = BLEUUID("d5060405-a904-deb9-4748-2c7f4a124842");
    uint8_t NotifyOn[] = {on_off, 0x00};
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic0)->getDescriptor((uint16_t)0x2902)->writeValue((uint8_t*)NotifyOn, sizeof(NotifyOn), true);
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic1)->getDescriptor((uint16_t)0x2902)->writeValue((uint8_t*)NotifyOn, sizeof(NotifyOn), true);
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic2)->getDescriptor((uint16_t)0x2902)->writeValue((uint8_t*)NotifyOn, sizeof(NotifyOn), true);
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic3)->getDescriptor((uint16_t)0x2902)->writeValue((uint8_t*)NotifyOn, sizeof(NotifyOn), true);
    return myo::pClient->getService(BLEUUID(tservice))->getCharacteristic(BLEUUID(tcharacteristic0));
  }
}

BLERemoteCharacteristic* myo::imu_notification(uint8_t on_off) {
  if (myo::connected) {
    BLEUUID tservice = BLEUUID("d5060002-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060402-a904-deb9-4748-2c7f4a124842");
    uint8_t NotifyOn[] = {on_off, 0x00};
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic)->getDescriptor((uint16_t)0x2902)->writeValue((uint8_t*)NotifyOn, sizeof(NotifyOn), true);
    return myo::pClient->getService(BLEUUID(tservice))->getCharacteristic(BLEUUID(tcharacteristic));
  }
}

BLERemoteCharacteristic* myo::battery_notification(uint8_t on_off) {
  if (myo::connected) {
    BLEUUID tservice = BLEUUID("0000180f-0000-1000-8000-00805f9b34fb");
    BLEUUID tcharacteristic = BLEUUID("00002a19-0000-1000-8000-00805f9b34fb");
    uint8_t NotifyOn[] = {on_off, 0x00};
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic)->getDescriptor((uint16_t)0x2902)->writeValue((uint8_t*)NotifyOn, sizeof(NotifyOn), true);
    return myo::pClient->getService(BLEUUID(tservice))->getCharacteristic(BLEUUID(tcharacteristic));
  }
}

BLERemoteCharacteristic* myo::gesture_notification(uint8_t on_off) {
  if (myo::connected) {
    BLEUUID tservice = BLEUUID("d5060003-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060103-a904-deb9-4748-2c7f4a124842");
    uint8_t IndicateOn[] = {2 * on_off, 0x00};
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic)->getDescriptor(((uint16_t)0x2902))->writeValue((uint8_t*)IndicateOn, sizeof(IndicateOn), true);
    return myo::pClient->getService(BLEUUID(tservice))->getCharacteristic(BLEUUID(tcharacteristic));
  }
}

/********************************************************************************************************
    COMMANDS
 ********************************************************************************************************/

void myo::set_myo_mode(uint8_t emg_mode, uint8_t imu_mode, uint8_t clf_mode) {
  if (myo::connected) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060401-a904-deb9-4748-2c7f4a124842");
    uint8_t writeVal[] = {0x01, 0x03, emg_mode, imu_mode, clf_mode};
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic)->writeValue(writeVal, sizeof(writeVal));
  }
}

void myo::set_sleep_mode(uint8_t sleep_mode) {
  if (myo::connected) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060401-a904-deb9-4748-2c7f4a124842");
    uint8_t writeVal[] = {0x09, 0x01, sleep_mode};
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic)->writeValue(writeVal, sizeof(writeVal));
  }
}

void myo::vibration(uint8_t duration) {
  if (myo::connected) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060401-a904-deb9-4748-2c7f4a124842");
    uint8_t writeVal[] = {0x03, 0x01, duration};
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic)->writeValue(writeVal, sizeof(writeVal));
  }
}

void myo::user_action(uint8_t action_type) {
  if (myo::connected) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060401-a904-deb9-4748-2c7f4a124842");
    uint8_t writeVal[] = {myohw_command_user_action, 0x01, action_type};
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic)->writeValue(writeVal, sizeof(writeVal));
  }
}

void myo::unlock(uint8_t unlock_mode) {
  if (myo::connected) {
    BLEUUID tservice = BLEUUID("d5060001-a904-deb9-4748-2c7f4a124842");
    BLEUUID tcharacteristic = BLEUUID("d5060401-a904-deb9-4748-2c7f4a124842");
    uint8_t writeVal[] = {0x0a, 0x01, unlock_mode};
    myo::pClient->getService(tservice)->getCharacteristic(tcharacteristic)->writeValue(writeVal, sizeof(writeVal));
  }
}
