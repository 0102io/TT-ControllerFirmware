#include <Arduino.h>
#include "IMU.h"
#include "TapHandler.h"
#include "comms_manager.h"
#include "utils.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

TapHandler tapHandler;
IMU imu;

SemaphoreHandle_t notifyMutex;

/*
This function runs as a separate task, and executes when the status timer alarm fires. 
It sends regular updates of system info, tap related info, and IMU data to central. 
*/
void statusNotification(void * parameter) {
  std::vector<uint8_t> statusArray(30);

  for (;;) {
    xEventGroupWaitBits(notificationEventGroup, EVENT_BIT0, pdTRUE, pdFALSE, portMAX_DELAY);
    disableStatusTimer();

    if (centralConnected) {
      // These are written out individually so that it's easy to add new bytes to the end of the sent array
      statusArray[0] = batteryPercent;

      std::vector<uint8_t> tapHandlerStatus = tapHandler.getStatus();
      statusArray[1] = tapHandlerStatus[0]; // currentTapOutID
      statusArray[2] = tapHandlerStatus[1]; // tapQ.headroom MSB
      statusArray[3] = tapHandlerStatus[2]; // tapQ.headroom LSB

      imu.poll();
      statusArray[4] = imu.accelXptr[0]; // LSB first for default javascript interpretation
      statusArray[5] = imu.accelXptr[1];
      statusArray[6] = imu.accelXptr[2];
      statusArray[7] = imu.accelXptr[3];
      statusArray[8] = imu.accelYptr[0];
      statusArray[9] = imu.accelYptr[1];
      statusArray[10] = imu.accelYptr[2];
      statusArray[11] = imu.accelYptr[3];
      statusArray[12] = imu.accelZptr[0];
      statusArray[13] = imu.accelZptr[1];
      statusArray[14] = imu.accelZptr[2];
      statusArray[15] = imu.accelZptr[3];
      statusArray[16] = imu.gyroXptr[0];
      statusArray[17] = imu.gyroXptr[1];
      statusArray[18] = imu.gyroXptr[2];
      statusArray[19] = imu.gyroXptr[3];
      statusArray[20] = imu.gyroYptr[0];
      statusArray[21] = imu.gyroYptr[1];
      statusArray[22] = imu.gyroYptr[2];
      statusArray[23] = imu.gyroYptr[3];
      statusArray[24] = imu.gyroZptr[0];
      statusArray[25] = imu.gyroZptr[1];
      statusArray[26] = imu.gyroZptr[2];
      statusArray[27] = imu.gyroZptr[3];
      statusArray[28] = (imu.temperature >> 8) & 0xFF;
      statusArray[29] = imu.temperature & 0xFF;

      if (xSemaphoreTake(notifyMutex, portMAX_DELAY) == pdTRUE) {
        notifyCentral(STATUS_UPDATE, statusArray);
        xSemaphoreGive(notifyMutex);
      }
    }
    unsigned long interval = 1000 / statusNotificationFreq;
    if (statusTimerRepeat) setStatusTimer(interval);
  }
}

void warningNotification (void * parameter) {
  for (;;) {
    xEventGroupWaitBits(notificationEventGroup, EVENT_BIT1, pdTRUE, pdFALSE, portMAX_DELAY);
    if (xSemaphoreTake(notifyMutex, portMAX_DELAY) == pdTRUE) {
      if (xSemaphoreTake(warningQMutex, portMAX_DELAY) == pdTRUE) { 
        notifyCentral(WARNING, warningQ, warningQTail);
        warningQTail = 0;
        xSemaphoreGive(warningQMutex);
      }
      xSemaphoreGive(notifyMutex);
    }
  }
}

void setup() {
  setupUtils();
  tapHandler.setupTapHandler();
  setupBLE(&tapHandler); // should move this later so we can't connect before we've finished setting up
  #if VERSION_IS_AT_LEAST(12,0)
  imu.setupIMU();
  #endif // VERSION_IS_AT_LEAST(12,0)

  notifyMutex = xSemaphoreCreateMutex();
  
  // TODO find more appropriate stack size with uxTaskGetStackHighWaterMark()
  uint16_t stackSize = 10000;
  uint8_t priority = 1;
  createTaskFunction(statusNotification, "statusTask", stackSize, priority);
  createTaskFunction(warningNotification, "warningTask", stackSize, priority);

  statusNotificationFreq = DEFAULT_TRANSMISSION_FREQUENCY;

  #if VERSION_IS_AT_LEAST(12,3)
    setWatchDog(true);
  #endif // VERSION_IS_AT_LEAST(12,3)

  // Blink out to indicate board is ready to go
  blink(2, 200, HEX_GREEN);

  setStatusTimer(1);
  #ifdef STRESS_TEST
    tapHandler.stressTest();
  #endif // STRESS_TEST
  updateBatteryPercent();
  delay(100);
  if (!batteryPercent) updateBatteryPercent();
}

void loop() {
  xEventGroupWaitBits(tapEventGroup, EVENT_BIT0, pdTRUE, pdFALSE, portMAX_DELAY);
  while (!tapHandler.isDoneTapping()) {
    tapHandler.tap();
  }
  DPRINTLN("Tapout Complete");
  #ifdef REGULATOR_PWR_SAVE
    digitalWrite(REG12V_EN_PIN, LOW);
  #endif
  #ifdef STRESS_TEST
    tapHandler.stressTest();
  #endif // STRESS_TEST
}