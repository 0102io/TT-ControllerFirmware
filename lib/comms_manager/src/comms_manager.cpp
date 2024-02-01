#include "comms_manager.h"
#include "utils.h"
extern "C" {
#include "esp_gap_ble_api.h"
}

BLEServer *pServer = NULL; // BLE server pointer
BLECharacteristic * pTxCharacteristic = NULL; // pointer to characteristic used for most central<-->peripheral communication
bool centralConnected = false;
uint8_t txValue = 0;

BLECharacteristic *pOtaCharacteristic = NULL; // pointer to characteristic used for over the air (OTA) firmware uploads
esp_ota_handle_t otaHandle = 0;
bool updateFlag = false;  // OTA update flag

/*
Server callbacks wrapper that can access the TapHandler object created in main.cpp, 
and has custom onConnect and onDisconnect methods.
*/
class ServerCallbacks: public BLEServerCallbacks {

  private:
    TapHandler* tapHandler;

  public:
    ServerCallbacks(TapHandler* handler) : tapHandler(handler) {}

    void onConnect(BLEServer* pServer) {
      centralConnected = true;
      setInactivityTimer(false); // disable the inactivity timer so we don't go into deep sleep
      blink(2, 200, HEX_BLUE);
    };

    void onDisconnect(BLEServer* pServer) {
      centralConnected = false;
      std::vector<uint8_t> cancelMsg = {TAP_OUT, NO_TAPOUT_ID};
      tapHandler->receiveTapOut(cancelMsg);
      BLEDevice::startAdvertising();
      setInactivityTimer(true);
    }
};

/*
Method that is called when we receive a message from central. Depending on the message type (first element in the message),
we call the relevant TapHandler or utils function.
*/
void mainCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  std::string rxStringValue = pCharacteristic->getValue();

    // Convert std::string to std::vector<uint8_t>
    std::vector<uint8_t> rxValue(rxStringValue.begin(), rxStringValue.end());

    DPRINT("Message received: ");
    for (int i = 0; i < rxValue.size(); i++) {
      DPRINT("[");
      BINPRINT(rxValue[i]);
      DPRINT("]");
    }
    DPRINTLN("");

    std::vector<uint8_t> retData;
    int msgSize = rxValue.size();

    if (msgSize > 0) {
      uint8_t msgType = rxValue[0];
      switch(msgType){
        case TAP_OUT:
          DPRINTLN("Message type: TAP_OUT");
          tapHandler->receiveTapOut(rxValue);
          break;
        case CANCEL_AND_TAP:
          DPRINTLN("Message type: TAP_OUT");
          tapHandler->cancelAndTap(rxValue);
          break;
        case GET_DEVICE_INFO:
          DPRINTLN("Message type: GET_DEVICE_INFO");
          deviceInfo[0] = macAddress[0];
          deviceInfo[1] = macAddress[1];
          deviceInfo[2] = macAddress[2];
          deviceInfo[3] = macAddress[3];
          deviceInfo[4] = macAddress[4];
          deviceInfo[5] = macAddress[5];
          deviceInfo[6] = version[0];
          deviceInfo[7] = version[1];
          deviceInfo[8] = version[2];
          deviceInfo[9] = version[3];
          deviceInfo[10] = version[4];
          deviceInfo[11] = version[5];
          deviceInfo[12] = version[6];
          deviceInfo[13] = version[7];
          notifyCentral(DEVICE_INFO, deviceInfo);
          break;
        case UPDATE_STATUS_FREQUENCY:
          DPRINTLN("Message type: UPDATE_STATUS_FREQUENCY");
          if (msgSize > 1) {
            disableStatusTimer();
            uint8_t pollingFrequency = rxValue[1];
            if (pollingFrequency == 0) pollingFrequency = 1; // this could alternatively be used to stop automatic status updates
            if (pollingFrequency > MAX_TRANSMISSION_FREQUENCY) pollingFrequency = MAX_TRANSMISSION_FREQUENCY;
            unsigned long statusInterval = 1000 / pollingFrequency;
            setStatusTimer(statusInterval);
            // TODO also tell the IMU to use the nearest sampling frequency
          }
          break;
        default:
          ARGPRINT("Message type with byte code: ", msgType);
          DPRINTLN(" not known.");
          std::vector<uint8_t> type = {msgType};
          notifyCentral(INVALID_MSG_TYPE, type);
          break;
      }
    }
}

/*
When we receive an OTA upload request from central, make the user confirm that they want to 
proceed by pressing the interact button, then load in the chunks of data until we reach the last packet 
(which won't be full). 

TODO the code hangs while waiting for an interact button press. It should probably be non blocking instead.
*/
void otaCallbacks::onWrite(BLECharacteristic *pCharacteristic)
{
  std::string rxData = pCharacteristic->getValue();
  if (!updateFlag) { //If it's the first packet of OTA since bootup, begin OTA
    unsigned long timeout = millis() + OTA_BTN_TIMEOUT;
    otaRequest = true;
    while(!otaProceed && timeout > millis()) delay(1);
    otaRequest = false; // reset flag
    if (!otaProceed) {
      notifyCentral(OTA_TIMEOUT);
      return;
    }
    DPRINTLN("BeginOTA");
    esp_ota_begin(esp_ota_get_next_update_partition(NULL), OTA_SIZE_UNKNOWN, &otaHandle);
    updateFlag = true;
  }
  if (otaProceed) {
    if (rxData.length() > 0)
    {
      esp_ota_write(otaHandle, rxData.c_str(), rxData.length());
      if (rxData.length() != FULL_PACKET)
      {
        esp_ota_end(otaHandle);
        DPRINTLN("EndOTA");
        if (ESP_OK == esp_ota_set_boot_partition(esp_ota_get_next_update_partition(NULL))) {
          uint8_t txData[5] = {1, 2, 3, 4, 5};
          pCharacteristic->setValue((uint8_t*)txData, 5);
          pCharacteristic->notify();
          delay(3000);
          esp_restart();
        }
        else {
          DPRINTLN("Upload Error");
        }
      }
    }

    uint8_t txData[5] = {1, 2, 3, 4, 5};
    pCharacteristic->setValue((uint8_t*)txData, 5);
    pCharacteristic->notify();
  }
}

/*
Creates the BLE service and starts advertising. It gets passed the TapHandler object from main.cpp.
*/
void setupBLE(TapHandler* tapHandler) {

  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(BLE_MTU);
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE | 
                      BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  pTxCharacteristic->setCallbacks(new mainCallbacks(tapHandler));
  pServer->setCallbacks(new ServerCallbacks(tapHandler));

  // Add OTA characteristic
  pOtaCharacteristic = pService->createCharacteristic(
                         CHARACTERISTIC_UUID_FW,
                         BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
                       );

  pOtaCharacteristic->addDescriptor(new BLE2902());
  pOtaCharacteristic->setCallbacks(new otaCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

/*
Used to send data to central. it can be called with just a type (default value for data will be an 
empty vector), or a data vector can be specified. 
*/
void notifyCentral(uint8_t type, const std::vector<uint8_t>& data) {
  if (!centralConnected) return;
  uint16_t sz = data.size();
  uint8_t txValue[sz + 1];
  txValue[0] = type;
  for (int i = 1; i < sz + 1; i++) {
    txValue[i] = data[i-1];
  }
  // Set the value of the BLE Characteristic
  pTxCharacteristic->setValue(txValue, sz+1);
  pTxCharacteristic->notify(); // Send the data over BLE
}