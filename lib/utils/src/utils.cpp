#include "configuration.h"
#include "utils.h"
#include <Arduino.h>
#include "Adafruit_MAX1704X.h"
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#ifdef AUTO_LIGHT_SLEEP
#include <esp_pm.h>
#endif //AUTO_LIGHT_SLEEP
#include <esp_system.h>
#include <Wire.h>
#include <esp_sleep.h>

hw_timer_t * tapTimer = NULL; // used for tap offDuration timing
hw_timer_t * statusTimer = NULL; // used for sending the status messages to central
hw_timer_t * watchdogPetTimer = NULL; // used to "pet" the watchdog (resets the timer but doesn't disable it)
hw_timer_t * inactivityTimer = NULL; // used to go into deep sleep if we aren't connected to bluetooth and are idle for a while

bool statusTimerRepeat; // causes the status function to rearm the status timer
bool hbDisbaledEvent = false;
bool otaRequest = false;
volatile bool otaProceed = false;

uint8_t statusNotificationFreq;

std::vector<uint8_t> macAddress(6);
std::vector<uint8_t> deviceInfo(16);
Preferences preferences;
uint8_t substrateType;
uint8_t substrateVMajor;
uint8_t substrateVMinor;

EventGroupHandle_t tapEventGroup;
EventGroupHandle_t notificationEventGroup;

std::vector<uint8_t> warningQ(MAX_WARNING_QUEUE_SIZE);
uint16_t warningQTail = 0; // first open position in the queue
SemaphoreHandle_t warningQMutex;

uint8_t batteryPercent = 0;
float batteryVoltage = 0;
float batteryCRate = 0;
uint16_t imuTemperature;
uint8_t boardOverheatLevel = 0;

Adafruit_MAX17048 fuelGauge;
bool socChanged = false;

/*
Set up timers, auto light sleep, input/output pins, etc.
*/
void setupUtils() {
  
  #ifdef DEBUG
    Serial.begin(115200);
    delay(2000); // wait to make sure we actually get the first print statement below
    DPRINTLN("Debugging is enabled.");
  #endif

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  for (int i = 0; i < 6; i++) {
    macAddress[i] = mac[i];
  }

  preferences.begin("saved-settings", false);
  substrateType = preferences.getUChar("substrateType", 0);
  substrateVMajor = preferences.getUChar("substrateVMajor", 0);
  substrateVMinor = preferences.getUChar("substrateVMinor", 0);
  if (!substrateType) {
    preferences.putUChar("substrateType", DEFAULT_SUBSTRATE);
    preferences.putUChar("substrateVMajor", DEFAULT_SUBSTRATE_VMAJOR);
    preferences.putUChar("substrateVMinor", DEFAULT_SUBSTRATE_VMINOR);
    substrateType = DEFAULT_SUBSTRATE;
    substrateVMajor = DEFAULT_SUBSTRATE_VMAJOR;
    substrateVMinor = DEFAULT_SUBSTRATE_VMINOR;
  }
  preferences.end();

  #ifdef CPU_CLK_FREQ_OVERIDE
    setCpuFrequencyMhz(CPU_CLK_FREQ_OVERIDE);
  #endif //CPU_CLK_FREQ_OVERIDE

  tapTimer = timerBegin(0, CPU_CLK_FREQ / 1000000, true);
  timerAttachInterrupt(tapTimer, &tapTimerInterrupt, true);
  tapEventGroup = xEventGroupCreate();

  statusTimer = timerBegin(1, CPU_CLK_FREQ / 1000000, true);
  timerAttachInterrupt(statusTimer, &statusTimerInterrupt, true);
  notificationEventGroup = xEventGroupCreate();
  statusTimerRepeat = true;

  #if VERSION_IS_AT_LEAST(12, 3)
  watchdogPetTimer = timerBegin(2, CPU_CLK_FREQ / 1000000, true);
  timerAttachInterrupt(watchdogPetTimer, &petWatchDog, true);
  notificationEventGroup = xEventGroupCreate();
  #endif // VERSION_IS_AT_LEAST(12, 3)

  inactivityTimer = timerBegin(3, CPU_CLK_FREQ / 1000000, true);
  timerAttachInterrupt(inactivityTimer, &deepSleep, true);

  #if VERSION_IS(12, 4)
    pinMode(CURRENT_SENSE_PIN, INPUT);
    digitalWrite(LED, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    pinMode(LED, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    pinMode(USER_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(USER_BUTTON), interactButtonPress, FALLING);
    pinMode(REG12V_EN_PIN, OUTPUT);
    pinMode(IMU_INT1_PIN, INPUT);
    pinMode(IMU_INT2_PIN, INPUT);
    #ifndef REGULATOR_PWR_SAVE
      digitalWrite(REG12V_EN_PIN, HIGH);
    #endif
    pinMode(WD_EN_PIN, OUTPUT);
    digitalWrite(WD_EN_PIN, HIGH); // active low
    pinMode(WDI_PIN, OUTPUT);
    digitalWrite(WDI_PIN, LOW);
    pinMode(ALRT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ALRT_PIN), fuelGaugeAlertISR, FALLING);
    pinMode(QSTRT_PIN, OUTPUT);
    digitalWrite(QSTRT_PIN, LOW);
    setupFuelGauge();
    #ifdef HBEN_DISABLED_INTERRUPT
      pinMode(HBEN_INTERRUPT_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(HBEN_INTERRUPT_PIN), hbDisabledISR, FALLING);
    #endif
  #elif VERSION_IS(12, 3)
    pinMode(CURRENT_SENSE_PIN, INPUT);
    digitalWrite(LED, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
    pinMode(LED, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    pinMode(USER_BUTTON, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(USER_BUTTON), interactButtonPress, FALLING);
    pinMode(REG12V_EN_PIN, OUTPUT);
    #ifdef HBEN_DISABLED_INTERRUPT
      pinMode(HBEN_INTERRUPT_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(HBEN_INTERRUPT_PIN), hbDisabledISR, FALLING);
    #endif
    pinMode(IMU_INT1_PIN, INPUT);
    pinMode(IMU_INT2_PIN, INPUT);
    #ifndef REGULATOR_PWR_SAVE
      digitalWrite(REG12V_EN_PIN, HIGH);
    #endif
    pinMode(BATT_MONITOR, INPUT);
    pinMode(WD_EN_PIN, OUTPUT);
    digitalWrite(WD_EN_PIN, HIGH); // active low
    pinMode(WDI_PIN, OUTPUT);
    digitalWrite(WDI_PIN, LOW);
    setupBattery();
  #elif VERSION_IS(11, 2)
    digitalWrite(LED, LOW);
    #ifdef HBEN_DISABLED_INTERRUPT
      pinMode(HBEN_INTERRUPT_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(HBEN_INTERRUPT_PIN), hbDisabledISR, FALLING);
    #endif
    pinMode(6, OUTPUT);
    digitalWrite(6, HIGH); // for testing OC interrupt
  #endif
  
  #ifdef AUTO_LIGHT_SLEEP
    // from: https://community.platformio.org/t/esp32-auto-light-sleep-arduino-idf/13676/4 
    esp_pm_config_esp32s3_t PmConfig;
    PmConfig.max_freq_mhz = 80;
    PmConfig.min_freq_mhz = 80; // 80MHz is the min clock frequency we can use with bluetooth
    PmConfig.light_sleep_enable = true;
    esp_err_t err = esp_pm_configure(&PmConfig);

    // if (err == ESP_OK) blink(1, 100, HEX_GREEN);
    // else if (err == ESP_ERR_INVALID_ARG) blink(1, 100, HEX_YELLOW);
    // else if (err == ESP_ERR_NOT_SUPPORTED) blink(1, 100, HEX_RED);
  #endif //AUTO_LIGHT_SLEEP

  esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(USER_BUTTON), LOW);
  setInactivityTimer(true);
  warningQMutex = xSemaphoreCreateMutex();
}

// wrapper for xTaskCreate
void createTaskFunction(void (*taskFunction)(void *), const char *taskName, uint16_t stackSize, uint8_t priority) {
  xTaskCreate(
    taskFunction,   /* Task function. */
    taskName,       /* name of task. */
    stackSize,      /* Stack size of task (in words - 4 bytes on ESP32s3) */
    NULL,           /* parameter of the task */
    priority,       /* priority of the task */
    NULL);          /* Task handle to keep track of created task */
}

/*
Blinks the RGB indicator LED a number of times (qty), with equal on/off duration (duration).
A couple of colors are defined already, e.g. HEX_RED
Note that this function is blocking.
*/
void blink(int qty, int duration, uint32_t color) {
  uint8_t r = 255 - (color >> 16) & 0xFF;
  uint8_t g = 255 - (color >> 8) & 0xFF;
  uint8_t b = 255 - color & 0xFF;

  #if VERSION_IS_AT_LEAST(12, 1)
    for (int i = 0; i < qty; i++) {
      analogWrite(LED, r);
      analogWrite(LEDG, g);
      analogWrite(LEDB, b);
      delay(duration);
      analogWrite(LED, 255);
      analogWrite(LEDG, 255);
      analogWrite(LEDB, 255);
      delay(duration);
    }
  #else 
    for (int i = 0; i < qty; i++) {
      digitalWrite(LED, HIGH);
      delay(duration);
      digitalWrite(LED, LOW);
      delay(duration);
    }
  #endif // VERSION_IS_AT_LEAST(12, 1)
}

void setInactivityTimer(bool isEnabled) {
  if(isEnabled) {
    timerAlarmWrite(inactivityTimer, MAX_INACTIVITY_MINUTES * 60 * 1000000, false);
    timerAlarmEnable(inactivityTimer);
    DPRINTLN("enabled inactivity timer");
  }
  else {
    timerAlarmDisable(inactivityTimer);
    DPRINTLN("disabled inactivity timer");
  }
}

void deepSleep() {
  esp_deep_sleep_start();
}
// ------------------ v12e+ Functions ------------------
#if VERSION_IS_AT_LEAST(12, 4)
#define EXT_PIN 39 // unused pin on this version, but without this define there are errors in the old battery communication functions
/*
Callback for receiving alert pin interrupts from the fuel gauge, then reset the alert.
*/
void receiveAlertSOC() {
  delay(1);
  batteryPercent = (uint8_t)fuelGauge.cellPercent();
  delay(1);
  socChanged = false;
  fuelGauge.clearAlertFlag(MAX1704X_ALERTFLAG_SOC_CHANGE); // clear the alert flag in the status register
  setFuelGaugeConfig(); // this clears the alert bit in the config register (needed to pull the alert pin high again)
}

/*
Connect with the MAX17048, then configure it by setting the config register to 0x975F, 
which is the default setting (0x971C) plus:
- ALSC = 1, which allows the SOC alert to pull the ~ALRT pin low when they trigger
- ATHD = 11111b, which sets the SOC detection threshold to 1%
*/
void setupFuelGauge() {
  if (fuelGauge.begin()) {
    DPRINTLN("Fuel gauge found");
  }
  else DPRINTLN("Fuel gauge not found");
  delay(1);
  setFuelGaugeConfig();
  delay(200); // give the fuel gauge time to do it's reset and calculate a new estimate (about 200ms)
  batteryPercent = (uint8_t)fuelGauge.cellPercent();
  ARGPRINTLN("battery percent = ", batteryPercent);
}

void setFuelGaugeConfig() {
  uint8_t msb = 0x97;
  uint8_t lsb = 0x5F;
  Wire.beginTransmission(ADDR_MAX17048);
  Wire.write(REG_MAX17048_CONFIG); // CONFIG register address
  Wire.write(msb); // Write high byte
  Wire.write(lsb); // Write low byte
  uint8_t err = Wire.endTransmission();
  delay(1);
  if (err != 0) ARGPRINTLN("I2C error on fuel gauge bus:", err);
}

void fuelGaugeAlertISR() {
  if (digitalRead(ALRT_PIN) == LOW) socChanged = true;
}

void updateBatteryVoltageAndCRate() {
  batteryVoltage = fuelGauge.cellVoltage();
  batteryCRate = fuelGauge.chargeRate();
}
#endif // VERSION_IS_AT_LEAST(12, 4)

// ------------------ v12d+ Functions ------------------
#if VERSION_IS_AT_LEAST(12, 3)

void setWatchDog(bool isEnabled) {
  if(isEnabled) {
    digitalWrite(WD_EN_PIN, LOW); // enables the chip and starts the timer
    timerAlarmWrite(watchdogPetTimer, WATCHDOG_PET_INTERVAL * 1000, true);
    timerAlarmEnable(watchdogPetTimer);
    DPRINTLN("set watchdog");
  }
  else digitalWrite(WD_EN_PIN, HIGH); // resets the timer and disables the chip
}

void petWatchDog() {
  digitalWrite(WDI_PIN, HIGH); // resets the timer; another timer period starts as long as WD_EN_PIN is low
  delayMicroseconds(2); // min WDI pulse width for STWD100YNX is 1us
  digitalWrite(WDI_PIN, LOW);
}

#endif // VERSION_IS_AT_LEAST(12, 3)

// ------------------ v12c+ Functions ------------------

#if VERSION_IS_AT_LEAST(12, 2)

/*
TOUCH_PIN2 is connected to the mounting holes in the controller's enclosure, and this 
function can be used to detect whether the screws are close to the user's skin. This isn't 
used for anything right now, but it could be useful for determining if the controller is
being worn.
*/
bool caseTouchDetected() {
  int touchValue = touchRead(TOUCH_PIN2);
  ARGPRINTLN("Touch 2 value: ", touchValue);
  if (touchValue > caseTouchThreshold) {
    return true;
  } else return false;
}

/*
Callback for receiving messages from the battery pack.
*/
void receiveBatteryMessage(int numBytes) {
  uint8_t code = Wire.read();
  if (numBytes == 2) {
    uint8_t value = Wire.read();
    if (code == BATTERY_PERCENT) batteryPercent = value;
    ARGPRINTLN("Percent: ", batteryPercent);
  }
  else if (numBytes == 1) {
    ARGPRINTLN("Received single byte code: ", code);
  }
  else if (numBytes) {
    uint8_t data;
    for (int i = 1; i < numBytes; i++) {
      data = Wire.read();
    }
    DPRINTLN("Unexpected message size");
  }
  else DPRINTLN("Error receiving battery message");
  digitalWrite(EXT_PIN, LOW);
}

/*
Sets up Wire as an I2C peripheral, and sets the callback for 
messages sent on that bus. 
*/
void setupBattery() {
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveBatteryMessage);
  delay(10);
  pinMode(EXT_PIN, OUTPUT);
}

/*
To prompt the v12d battery pack for an updated battery %, we write 
EXT_PIN high.
*/
void updateBatteryPercent() {
  digitalWrite(EXT_PIN, HIGH);
}

#endif // VERSION_IS_AT_LEAST(12, 2)

// ------------------ v12b+ Functions ------------------

#if VERSION_IS_AT_LEAST(12, 1)

/*
TOUCH_PIN is connected to a trace in the flex tail / substrate PCBs, and this 
function can be used to detect whether that trace is close to the user's skin. This isn't 
used for anything right now, but it could be useful for determining if the substrate is
connected, or possibly what type of substrate it is (e.g. v12d Patch vs v12b Palm).
*/
bool fpcTouchDetected() {
  int touchValue = touchRead(TOUCH_PIN);
  ARGPRINTLN("Touch 1 value: ", touchValue);
  if (touchValue > fpcTouchThreshold) {
    return true;
  } else return false;
}

/*
Detects an overcurrent event (i.e. the HBEN_INTERRUPT_PIN got pulled low by the watchdog or overcurrent circuit).
This likely would only happen when the ESP hangs or is damaged, in which case we couldn't get act on it through 
firmware.

TODO: figure out what this should be used for.
*/
void hbDisabledISR() {
  if (digitalRead(HBEN_INTERRUPT_PIN) == LOW) hbDisbaledEvent = true;
  DPRINTLN("------------------- H Bridge Disabled event detected -------------------");
}

// ISR for the interact button
void IRAM_ATTR interactButtonPress() {
  if (otaRequest) otaProceed = true;
}

#endif // VERSION_IS_AT_LEAST(12, 1)

// ------------------ Tap Timer Functions ------------------

/*
Function that is called when the tapTimer alarm fires.
*/
void IRAM_ATTR tapTimerInterrupt() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(tapEventGroup, EVENT_BIT0, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void disableTapTimer() {
  timerAlarmDisable(tapTimer);
  timerRestart(tapTimer);
}

void setTapTimer(uint64_t durationMS) {
  timerAlarmWrite(tapTimer, durationMS * 1000, false);
  timerAlarmEnable(tapTimer);
}

bool tapTimerEnabled() {
  return timerAlarmEnabled(tapTimer);
}

// ------------------ Status Timer Functions ------------------

/*
Function that is called when the statusTimer alarm fires.
*/
void IRAM_ATTR statusTimerInterrupt() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(notificationEventGroup, EVENT_BIT0, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void disableStatusTimer() {
  timerAlarmDisable(statusTimer);
  timerRestart(statusTimer);
}

void setStatusTimer(uint64_t durationMS) {
  timerAlarmWrite(statusTimer, durationMS * 1000, false);
  timerAlarmEnable(statusTimer);
}

// ------------------ Warning Notificaion Functions ------------------

/*
returns the remaining room in the warning notificaion queue/array
*/
uint8_t warningQRoom() {
  return (MAX_WARNING_QUEUE_SIZE - warningQTail);
}

/*
Adds a byte to the warning queue if there is room; returns true if the byte was added
*/
bool addToWarningQ(uint8_t byte) {
  if (warningQRoom()) {
    warningQ[warningQTail] = byte;
    warningQTail++;
    return true;
  }
  return false;
}

/*
Sets the board temperature level (used for attenuating taps if the board is hot), 
and sends a warning notification if the level has changed
*/
void updateBoardTempLevel(uint16_t temperature) {
  uint8_t lastOverHeatLevel = boardOverheatLevel;
  if (temperature > 62) boardOverheatLevel = 4;
  else if (temperature >= 58) boardOverheatLevel = 3;
  else if (temperature >= 54) boardOverheatLevel = 2;
  else if (temperature >= 50) boardOverheatLevel = 1;
  else boardOverheatLevel = 0;

  if (lastOverHeatLevel != boardOverheatLevel) { // this probably needs some hysteresis so we don't send a bunch of warnings as we're passing each threshold
    if (xSemaphoreTake(warningQMutex, portMAX_DELAY) == pdTRUE) {
      addToWarningQ(BOARD_OVERHEAT);
      addToWarningQ(boardOverheatLevel);
      EventBits_t uxBits = xEventGroupSetBits(notificationEventGroup, EVENT_BIT1); // unblock the warningNotification task
      xSemaphoreGive(warningQMutex);
    }
  }
}