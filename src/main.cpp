#include <Arduino.h>
#include "IMU.h"
#include "TapHandler.h"
#include "comms_manager.h"
#include "utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

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
      statusArray[4] = imu.dataPtr[0]; // accelX MSB
      statusArray[5] = imu.dataPtr[1]; // accelX
      statusArray[6] = imu.dataPtr[2]; // accelX
      statusArray[7] = imu.dataPtr[3]; // accelX LSB
      statusArray[8] = imu.dataPtr[4]; // accelY MSB
      statusArray[9] = imu.dataPtr[5]; // accelY
      statusArray[10] = imu.dataPtr[6]; // accelY
      statusArray[11] = imu.dataPtr[7]; // accelY LSB
      statusArray[12] = imu.dataPtr[8]; // accelZ MSB
      statusArray[13] = imu.dataPtr[9]; // accelZ
      statusArray[14] = imu.dataPtr[10]; // accelZ
      statusArray[15] = imu.dataPtr[11]; // accelZ LSB
      statusArray[16] = imu.dataPtr[12]; // gyroX MSB
      statusArray[17] = imu.dataPtr[13]; // gyroX
      statusArray[18] = imu.dataPtr[14]; // gyroX
      statusArray[19] = imu.dataPtr[15]; // gyroX LSB
      statusArray[20] = imu.dataPtr[16]; // gyroY MSB
      statusArray[21] = imu.dataPtr[17]; // gyroY
      statusArray[22] = imu.dataPtr[18]; // gyroY
      statusArray[23] = imu.dataPtr[19]; // gyroY LSB
      statusArray[24] = imu.dataPtr[20]; // gyroZ MSB
      statusArray[25] = imu.dataPtr[21]; // gyroZ
      statusArray[26] = imu.dataPtr[22]; // gyroZ
      statusArray[27] = imu.dataPtr[23]; // gyroZ LSB
      statusArray[28] = imu.dataPtr[24]; // temperature * 10 (MSB)
      statusArray[29] = imu.dataPtr[25]; // temperature * 10 (LSB)
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