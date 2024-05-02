#include "comms_manager.h"
#include "utils.h"
#include <freertos/event_groups.h>
extern "C" {
#include "esp_gap_ble_api.h"
}

BLEServer *pServer = NULL; // BLE server pointer
BLECharacteristic * pRxCharacteristic = NULL; // pointer to characteristic used for non-OTA central-->controller communication
bool centralConnected = false;
uint8_t txValue = 0;

BLECharacteristic * pTxCharacteristic = NULL; // pointer to characteristic used for non-OTA controller-->central communication

BLECharacteristic *pOtaCharacteristic = NULL; // pointer to characteristic used for over the air (OTA) firmware uploads
esp_ota_handle_t otaHandle = 0;
bool updateFlag = false;  // OTA update flag

SemaphoreHandle_t notifyMutex;

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
      BLEDevice::stopAdvertising();
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
void fromCentralCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
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
          deviceInfo[6] = HARDWARE_VERSION_MAJOR;
          deviceInfo[7] = HARDWARE_VERSION_MINOR;
          deviceInfo[8] = FIRMWARE_VERSION_MAJOR;
          deviceInfo[9] = FIRMWARE_VERSION_MINOR;
          deviceInfo[10] = FIRMWARE_VERSION_PATCH;
          deviceInfo[11] = substrateType;
          deviceInfo[12] = substrateVMajor;
          deviceInfo[13] = substrateVMinor;
          deviceInfo[14] = (uint8_t)(onDurationUS_max >> 8);
          deviceInfo[15] = (uint8_t)(onDurationUS_max);
          notifyCentral(DEVICE_INFO, deviceInfo);
          break;
        case UPDATE_STATUS_FREQUENCY:
          DPRINTLN("Message type: UPDATE_STATUS_FREQUENCY");
          if (msgSize == 2) {
            disableStatusTimer();
            uint8_t pollingFrequency = rxValue[1];
            if (pollingFrequency == 0) pollingFrequency = 1; // right now we need status messages to be sent because that causes the board temp to be checked regularly
            if (pollingFrequency > MAX_TRANSMISSION_FREQUENCY) pollingFrequency = MAX_TRANSMISSION_FREQUENCY;
            statusNotificationFreq = pollingFrequency;
            // TODO also tell the IMU to use the nearest sampling frequency
          }
          break;
        case CHANGE_DEFAULT_SUBSTRATE:
          DPRINTLN("Message type: CHANGE_DEFAULT_SUBSTRATE");
          if (msgSize == 4) {
            substrateType = rxValue[1];
            substrateVMajor = rxValue[2];
            substrateVMinor = rxValue[3];
            preferences.begin("saved-settings", false);
            preferences.putUChar("substrateType", substrateType);
            preferences.putUChar("substrateVMajor", substrateVMajor);
            preferences.putUChar("substrateVMinor", substrateVMinor);
            preferences.end();
            ARGPRINT("Substrate changed to type: ", substrateType);
            ARGPRINT(" v", substrateVMajor);
            ARGPRINTLN(".", substrateVMinor);
            delay(500);
            esp_restart();
          }
          break;
        default:
          ARGPRINT("Message type with byte code: ", msgType);
          DPRINTLN(" not known.");
          std::vector<uint8_t> type = {msgType};
          assert(xSemaphoreTake(warningQMutex, portMAX_DELAY) == pdTRUE);
          addToWarningQ(INVALID_MSG_TYPE);
          xSemaphoreGive(warningQMutex);
          xEventGroupSetBits(notificationEventGroup, EVENT_BIT1); // unblock the warningNotification task
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
      assert(xSemaphoreTake(warningQMutex, portMAX_DELAY) == pdTRUE);
      addToWarningQ(OTA_TIMEOUT);
      xSemaphoreGive(warningQMutex);
      xEventGroupSetBits(notificationEventGroup, EVENT_BIT1); // unblock the warningNotification task
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

  std::__cxx11::string bleName;
  switch (substrateType) {
    case PATCH:
      bleName = "TAPPY PATCH";
      break;
    case PALM:
      bleName = "TAPPY PALM";
      break;
    default:
      bleName = "TAPPY CONTROLLER";
      break;
  }

  notifyMutex = xSemaphoreCreateMutex();

  BLEDevice::init(bleName);
  BLEDevice::setMTU(BLE_MTU);
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pRxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                      );
  pRxCharacteristic->addDescriptor(new BLE2902());
  pRxCharacteristic->setCallbacks(new fromCentralCallbacks(tapHandler));
  pServer->setCallbacks(new ServerCallbacks(tapHandler));

  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());
  // pTxCharacteristic->setCallbacks(new toCentralCallbacks()); // don't have anything for this right now

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
  // use the TX characteristic
  assert(xSemaphoreTake(notifyMutex, portMAX_DELAY) == pdTRUE);
  pTxCharacteristic->setValue(txValue, sz+1);
  pTxCharacteristic->notify(); // Send the data over BLE
  xSemaphoreGive(notifyMutex);
}

/*
Overloaded function that only writes out the first [numBytes] of the vector
*/
void notifyCentral(uint8_t type, const std::vector<uint8_t>& data, uint16_t numBytes) {
  if (!centralConnected) return;
  uint8_t txValue[numBytes + 1];
  txValue[0] = type;
  for (int i = 1; i < numBytes + 1; i++) {
    txValue[i] = data[i-1];
  }
  // use the TX characteristic
  pTxCharacteristic->setValue(txValue, numBytes+1);
  pTxCharacteristic->notify(); // Send the data over BLE
}