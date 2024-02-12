#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include "configuration.h"

// TODO byte codes should be in comms_manager.h instead
// BLE byte code received message types
#define TAP_OUT 1
#define GET_DEVICE_INFO 2
#define CANCEL_AND_TAP 3
#define UPDATE_STATUS_FREQUENCY 4

// BLE byte code sent message types
#define STATUS_UPDATE 1
#define TAPOUT_COMPLETE 2
#define DEVICE_INFO 3

// BLE byte code error/warning message types
#define INVALID_MSG_TYPE 51
#define INCORRECT_MSG_SIZE 52
#define PARAM_OOB 53
#define ROW_INDEX_OOB 55
#define COL_INDEX_OOB 56
#define OTA_TIMEOUT 61
#define QUEUE_FULL 62

// Battery pack byte codes
#define BATTERY_PERCENT 1

// colour codes for BlinkOut
#define HEX_RED 0xFF0000
#define HEX_GREEN 0x00FF00
#define HEX_BLUE 0x0000FF
#define HEX_YELLOW 0xFFFF00
#define HEX_MAGENTA 0xFF00FF
#define HEX_CYAN 0x00FFFF
#define HEX_WHITE 0xFFFFFF

// address on the controller - battery pack I2C bus (pre-v12e)
#define I2C_ADDR 0x08

// I2C address of I2C devices
#define ADDR_MAX17048 0x36

// MAX17048 Registers
#define REG_MAX17048_CONFIG 0x0C
#define REG_MAX17048_SOC 0x04
#define REG_MAX17048_VERSION 0x08
#define REG_MAX17048_ID 0x19
#define REG_MAX17048_STATUS 0x1A
#define REG_MAX17048_CMD 0xFE

// MAX17048 bit positions
#define BIT_CONFIG_ATRT 0x20

#ifdef DEBUG
  #define DPRINT(x) Serial.print(x)
  #define DPRINTLN(x) Serial.println(x)
  #define BINPRINT(x) Serial.print(x, BIN)
  #define ARGPRINT(str, var) { Serial.print(str); Serial.print(var); }
  #define ARGPRINTLN(str, var) { Serial.print(str); Serial.println(var); }
#else
  #define DPRINT(x)
  #define DPRINTLN(x)
  #define BINPRINT(x)
  #define ARGPRINT(str, var)
  #define ARGPRINTLN(str, var)
#endif

void setupUtils();
void blink(int qty, int duration, uint32_t color);
bool caseTouchDetected();
bool fpcTouchDetected();
void hbDisabledISR();
void IRAM_ATTR interactButtonPress();
void setupBattery();
void updateBatteryPercent();
void receiveAlertSOC();
void setupFuelGauge();
void setFuelGaugeConfig();
void fuelGaugeAlertISR();
void IRAM_ATTR tapTimerInterrupt();
void disableTapTimer();
void setTapTimer(uint64_t durationTenthsMS);
bool tapTimerEnabled();
void createTaskFunction(void (*taskFunction)(void *), const char *taskName, uint16_t stackSize, uint8_t priority);
void IRAM_ATTR statusTimerInterrupt();
void disableStatusTimer();
void setStatusTimer(uint64_t durationMS);
void receiveBatteryMessage(int numBytes);
void setWatchDog(bool isEnabled);
void petWatchDog();
void setInactivityTimer(bool isEnabled);
void deepSleep();

extern bool statusTimerRepeat; // determines whether statusUpdates are sent continuously or only upon message reception/completion

extern EventGroupHandle_t tapEventGroup;
extern EventGroupHandle_t statusEventGroup;

#define EVENT_BIT0 (1 << 0) // main loop
#define EVENT_BIT1 (1 << 1) // imu

extern bool hbDisbaledEvent;
extern bool otaRequest;
extern volatile bool otaProceed;

extern std::vector<uint8_t> macAddress;
extern std::vector<uint8_t> deviceInfo;

extern uint8_t batteryPercent;
extern uint16_t imuTemperature;

extern uint8_t statusUpdateFreq;

#endif // UTILS_H