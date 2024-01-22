#ifndef COMMS_MANAGER_H
#define COMMS_MANAGER_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "TapHandler.h"
#include <esp_ota_ops.h>

// UUIDs for BLE service and characteristic
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME "TAPPY CONTROLLER"

// UUID for OTA
#define CHARACTERISTIC_UUID_FW  "c8659211-af91-4ad3-a995-a58d6fd26145"
#define FULL_PACKET 512
#define OTA_BTN_TIMEOUT 10000 // time to wait for a confirmation interact button press when OTA upload is initiated, ms

// since we're a peripheral device, these are just preferred values that central can choose to ignore
#define BLE_MTU 517
#define MIN_CONNECTION_INTERVAL 6 // in periods of 1.25ms, so 6 x 1.25 = 7.5ms
#define MAX_CONNECTION_INTERVAL 24 // in periods of 1.25ms, so 24 x 1.25 = 30ms
#define CONNECTION_LATENCY 0 // ms
#define CONNECTION_TIMEOUT 5000 // ms

// BLE Server and Service
extern BLEServer *pServer;
extern BLECharacteristic * pTxCharacteristic;
extern bool centralConnected;
extern uint8_t txValue;

extern BLECharacteristic *pOtaCharacteristic;
extern esp_ota_handle_t otaHandle;
extern bool updateFlag;

class mainCallbacks: public BLECharacteristicCallbacks {
    public:
        mainCallbacks(TapHandler* tapHandler) : tapHandler(tapHandler) {}
        void onWrite(BLECharacteristic *pCharacteristic);

    private:
        TapHandler* tapHandler;
};

// OTA over BLE code based on: https://learn.sparkfun.com/tutorials/esp32-ota-updates-over-ble-from-a-react-web-application/all 
class otaCallbacks: public BLECharacteristicCallbacks {
  public:
    otaCallbacks() {}
    void onWrite(BLECharacteristic *pCharacteristic);
};

void setupBLE(TapHandler* tapHandler);
void notifyCentral(uint8_t type, const std::vector<uint8_t>& data = std::vector<uint8_t>());

#endif // COMMS_MANAGER_H
