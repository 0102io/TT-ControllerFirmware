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
      statusArray[4] = (imu.imuData.accelX >> 24) & 0xFF;
      statusArray[5] = (imu.imuData.accelX >> 16) & 0xFF;
      statusArray[6] = (imu.imuData.accelX >> 8) & 0xFF;
      statusArray[7] = imu.imuData.accelX & 0xFF;
      statusArray[8] = (imu.imuData.accelY >> 24) & 0xFF;
      statusArray[9] = (imu.imuData.accelY >> 16) & 0xFF;
      statusArray[10] = (imu.imuData.accelY >> 8) & 0xFF;
      statusArray[11] = imu.imuData.accelY & 0xFF;
      statusArray[12] = (imu.imuData.accelZ >> 24) & 0xFF;
      statusArray[13] = (imu.imuData.accelZ >> 16) & 0xFF;
      statusArray[14] = (imu.imuData.accelZ >> 8) & 0xFF;
      statusArray[15] = imu.imuData.accelZ & 0xFF;
      statusArray[16] = (imu.imuData.gyroX >> 24) & 0xFF;
      statusArray[17] = (imu.imuData.gyroX >> 16) & 0xFF;
      statusArray[18] = (imu.imuData.gyroX >> 8) & 0xFF;
      statusArray[19] = imu.imuData.gyroX & 0xFF;
      statusArray[20] = (imu.imuData.gyroY >> 24) & 0xFF;
      statusArray[21] = (imu.imuData.gyroY >> 16) & 0xFF;
      statusArray[22] = (imu.imuData.gyroY >> 8) & 0xFF;
      statusArray[23] = imu.imuData.gyroY & 0xFF;
      statusArray[24] = (imu.imuData.gyroZ >> 24) & 0xFF;
      statusArray[25] = (imu.imuData.gyroZ >> 16) & 0xFF;
      statusArray[26] = (imu.imuData.gyroZ >> 8) & 0xFF;
      statusArray[27] = imu.imuData.gyroZ & 0xFF;
      statusArray[28] = (imu.imuData.temperature >> 8) & 0xFF;
      statusArray[29] = imu.imuData.temperature & 0xFF;

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
  if (!tapHandler.isDoneTapping()) {
    tapHandler.tap();
  }
  else {
    DPRINTLN("Tapout Complete");
    #ifdef REGULATOR_PWR_SAVE
      digitalWrite(REG12V_EN_PIN, LOW);
    #endif
    #ifdef STRESS_TEST
      tapHandler.stressTest();
    #endif // STRESS_TEST
  }
}