#include "configuration.h"
#include "TapHandler.h"
#include "utils.h"

//---------------------- Tap Handler Class ----------------------

TapHandler::TapHandler() {
    lastOnDur = onDur_init;
    lastOffDur = (uint16_t)offDur_init;
}

void TapHandler::setupTapHandler() {
    // Construct H bridge driver objects
    driver = new HBDriver*[numDrivers];
    for (int i = 0; i < numDrivers; i++) {
        driver[i] = new HBDriver(CSPin[i]);
    }

    // Create array of TapperMonitors, one for every possible output pin intersection, so n x n where n = number of chips x number of outputs
    #ifdef OVERTAP_PROTECTION
      int n = numDrivers * NUM_OUTPUTS_PER_DRIVER; // 10 output pins per driver chip
      tapperMonitor = new TapperMonitor*[n];
      for(int i = 0; i < n; ++i) {
          tapperMonitor[i] = new TapperMonitor[n];
          for (int j = 0; j < n; j++) {
            tapperMonitor[i][j].lastMicros = 0;
            tapperMonitor[i][j].heat = 0;
          }
      }
    #endif // OVERTAP_PROTECTION

    // SPI setup
    SPI.begin();

    // Clear error bits on each register, seems to always start with PSF+OC faults?
    for (int i = 0; i < numDrivers; i++) {
        srrReset(i, 1);
        driver[i]->srrTimerLastReset_outputs1to6 = millis();
        delay(150); // need to delay at least 100ms between successive SRR commands to the same IC
        srrReset(i, 0);
        driver[i]->srrTimerLastReset_outputs7to10 = millis();
    }
    
    currentTapOutID = NO_TAPOUT_ID;
}

/*
This method is called by comms manager when a CANCEL_AND_TAP message is received from central
*/
void TapHandler::cancelAndTap(std::vector<uint8_t> msg) {
  // cancel queue and call normal tapout function
    tapQ.clear();
    receiveTapOut(msg);
}

/*
This method is called by comms manager when a TAP_OUT message is received from central. If the ID of the 
message is NO_TAPOUT_ID, we clear whatever else was in the queue and add the pattern from this new message. 
Otherwise, we add the new pattern to the end of the queue. Then start the tap timer if it isn't already active.
*/
void TapHandler::receiveTapOut(std::vector<uint8_t> msg) {

  uint8_t id = msg[1];
  msg.erase(msg.begin(), msg.begin() + 2); // get rid of the type byte and id
  int len = (int) msg.size();

  if (id == NO_TAPOUT_ID || !len) {
    // cancel the existing tapout if there was one already
    tapQ.clear();
    return;
  }
  else {
    // add the data into the buffer, and take the new tapout id
    DPRINTLN("adding pattern to buffer");
    addToQueue(msg);
    currentTapOutID = id; // even though we haven't necessarily finished the last pattern, we change the pattern ID to the most recently received one
    #ifdef REGULATOR_PWR_SAVE
      digitalWrite(REG12V_EN_PIN, HIGH); // also enable the 12V regulator
    #endif
    xEventGroupSetBits(tapEventGroup, EVENT_BIT0); // unblock the tap task (main loop)
    return;
  }
}


/*
This method reads the elements of a TAP_OUT message and adds the tap indicies and settings to the queue.
It first checks to make sure the indicies are in bounds and the on duration isn't too long (set in configuration.cpp).
If an array index is out of bounds (OOB), this adds an "empty tap" - i.e. it pauses for the amount of time
a normal tap would, but it doesn't actually turn on the motor drivers. This helps to keep the cadence of the
pattern intact. If the on duration is too long, this sets it to the max on duration. 

If the queue fills up during this method, we stop adding taps starting with the first one that can't fit, and we tell central.

If the total message size is incorrect, we clear the queue because we don't know exactly where the missing or extra data is, and 
if we tap it out as is then it will almost certainly feel wrong.

For proper structuring of the TAP_OUT message, see the docs.

TODO: build a message with all the warning indicies and values to send back to central
*/
void TapHandler::addToQueue(std::vector<uint8_t> data) {

  // temp variables for checking if the received parameters are in bounds
  TapSettings newTap;
  bool queueFull = false;
  uint16_t rejectedIndex;
  bool incorrectMsgSize = false;
  bool indexOOB = false;

  // read each row and col into tap arrays
  int len = (int) data.size();

  for (int i = 0; i < len; i+= 6) {
    if (i + 6 <= len) {
      newTap.onDuration = (data[i] << 8) | data[i+1];
      newTap.offDuration = (data[i+2] << 8) | data[i+3];
      newTap.anodeID = (data[i+4] >> 4) & 0b11;
      newTap.anodeOutputPin = data[i+4] & 0b00001111;
      newTap.cathodeID = (data[i+5] >> 4) & 0b11;
      newTap.cathodeOutputPin = data[i+5] & 0b00001111;

      if (!tapQ.isFull()) tapQ.push(newTap);
      else {
        queueFull = true;
        rejectedIndex = i;
        ARGPRINTLN("Queue full. Index of first rejected tap: ", i);
        break;
      }
    }
    else {
      // message size is incorrect; clear the queue since the whole message is likely not formatted according to protocol
      tapQ.clear();
      incorrectMsgSize = true;
      DPRINTLN("Error: incorrect message size.");
      break;
    }
  }

  // add warnings to warningQ
  if (queueFull || incorrectMsgSize) {
    if (xSemaphoreTake(warningQMutex, 0) == pdTRUE) { // set xBlockTime to 0, because if something else is writing a warning msg it's probably more important and we don't want to block here
      if (queueFull) {
        addToWarningQ(QUEUE_FULL);
        addToWarningQ((uint8_t)(rejectedIndex >> 8));
        addToWarningQ((uint8_t)rejectedIndex);
      }
      if (incorrectMsgSize) addToWarningQ(INCORRECT_MSG_SIZE);
      xSemaphoreGive(warningQMutex);
      xEventGroupSetBits(notificationEventGroup, EVENT_BIT1); // unblock the warningNotification task
    }
  }

  return;
}

bool TapHandler::checkDiagnosticData(uint16_t diagnosticData, uint8_t driverID, uint8_t driverChannelSelectBit) {

  bool newErrorFlag = false;

  uint8_t oldErrors = driver[driverID]->errorFlags;
  uint8_t addedErrors = getErrorsFromDiagnosticReg(diagnosticData, driverID, driverChannelSelectBit);
  uint8_t newErrors = oldErrors | addedErrors;

  if (oldErrors != newErrors) {
    driver[driverID]->errorFlags = newErrors;
    newErrorFlag = true;
  }

  // if we have any over current errors, we need to try to do a status register reset
  if (newErrors & (1 << OC_7TO10_ERROR_BIT) && (millis() - driver[driverID]->srrTimerLastReset_outputs7to10) > SRR_MIN_RESET_MS) {
    srrReset(driverID, 1);
    driver[driverID]->srrTimerLastReset_outputs7to10 = millis();
  }
  if (newErrors & (1 << OC_1TO6_ERROR_BIT) && (millis() - driver[driverID]->srrTimerLastReset_outputs1to6) > SRR_MIN_RESET_MS) {
    srrReset(driverID, 0);
    driver[driverID]->srrTimerLastReset_outputs1to6 = millis();
  }

  return newErrorFlag;
}

/*
This method parses data sent by an MP6527 h-bridge driver. For a detailed overview of these errors, 
see Tables 3 and 4 here: https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MP6527GF/document_id/10142/
*/
uint8_t TapHandler::getErrorsFromDiagnosticReg(uint16_t diagnosticRegister, uint8_t driverID, uint8_t channelSelectBit) {
  uint8_t returnData = 0;

  for(uint8_t i = 0; i < 16; i++) {
    if(diagnosticRegister & (1 << i)) {
      switch(i) {
        case 0: //thermal warning, resets automatically if the junction temperature drops below its recovery point; SRR not needed
          ARGPRINT("Thermal warning for Hbridge ", driverID);
          if (channelSelectBit) DPRINTLN(", outputs 7-10");
          else DPRINTLN(", outputs 1-6");
          returnData |= (1 << TW_WARNING_BIT);
          break;
        // case 13: // open load detected, though as far as I've tested this always triggers for our application; this doesn't turn off an output unless OLD_SD is set; SRR reset is needed to clear it 
        //   ARGPRINT("Open load detected for Hbridge ", driverID);
        //   if (channelSelectBit) {
        //     DPRINTLN(", outputs 7-10");
        //     returnData |= (1 << OLD_7TO10_ERROR_BIT);
        //   }
        //   else {
        //     DPRINTLN(", outputs 1-6");
        //     returnData |= (1 << OLD_1TO6_ERROR_BIT);
        //   }
        //   break;
        case 14: // power supply failure; resets automatically if the supply returns to normal operating range; SRR reset not needed
          ARGPRINT("Power supply failure detected for Hbridge ", driverID);
          if (channelSelectBit) DPRINTLN(", outputs 7-10");
          else DPRINTLN(", outputs 1-6");
          returnData |= (1 << PSF_ERROR_BIT);
          break;
        case 15: 
          ARGPRINT("Over current detected for Hbridge ", driverID);
          if (channelSelectBit) {
            DPRINTLN(", outputs 7-10");
            returnData |= (1 << OC_7TO10_ERROR_BIT);
          }
          else {
            DPRINTLN(", outputs 1-6");
            returnData |= (1 << OC_1TO6_ERROR_BIT);
          }
          break;
      }
    }
  }
  return returnData;
}

/*
This method resets the status register on an h-bridge driver chip, which allows it to send new error messages. The chip has a 100ms 
cooldown between successive SRR writes: https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MP6527GF/document_id/10142/
*/
void TapHandler::srrReset(uint8_t id, uint8_t channelSelectBit) {
  unsigned long resetTimer = driver[id]->SRRtimer;
  unsigned long currentMillis = millis();
  if (currentMillis > resetTimer) {
    uint8_t cs = driver[id]->CS;
    ARGPRINT("Resetting SRR for driver: ", id);
    if (channelSelectBit) DPRINTLN(", HB7-10");
    else DPRINTLN(", HB1-6");

    uint16_t resetRegister = (1 << 15);
    if (channelSelectBit) resetRegister = resetRegister | (1 << 14);
    
    digitalWrite(cs, LOW);
    SPI.beginTransaction(SPISettings(clockSpeed, MSBFIRST, SPI_MODE1));
    SPI.transfer16(resetRegister);
    SPI.endTransaction();
    digitalWrite(cs, HIGH);

    driver[id]->SRRtimer = millis() + 100;
  } else {
    ARGPRINT("Tried to set SRR for driver ", id);
    ARGPRINT(" but the timer hasn't reset. Millis: ", currentMillis);
    ARGPRINTLN(", Reset Timer: ", resetTimer);
  }
}

/*
Loads the next tap in the TapQueue, then does the tap:
- calls for the appropriate output channels to be turned on
- delays for the tap's onDuration
- calls for the appropriate output channels to be turned off
- sets the tap timer to trigger again once the offDuration cooldown ellapses

This is currently set up to accomodate charlieplexing, where every output can be either an anode or cathode.
*/
void TapHandler::tap() {
  TapSettings currentTap = tapQ.pop();

  // load settings for this tap
  uint8_t anodeID = currentTap.anodeID;
  uint8_t cathodeID = currentTap.cathodeID;
  uint8_t anodeOutputPin = currentTap.anodeOutputPin;
  uint8_t cathodeOutputPin = currentTap.cathodeOutputPin;
  unsigned long onDurationUS = currentTap.onDuration * 10;
  unsigned long offDurationUS = currentTap.offDuration * 100;

  if (onDurationUS == 0) {
    delayMicroseconds(offDurationUS);
    return;
  }
  else if (onDurationUS > onDur_max) {
    onDurationUS = onDur_max;
    // send an oob warning
    if(xSemaphoreTake(warningQMutex, 0) == pdTRUE) { // xBlockTime set to 0 because we don't want to block while tapping
      addToWarningQ(PARAM_OOB);
      xSemaphoreGive(warningQMutex);
      xEventGroupSetBits(notificationEventGroup, EVENT_BIT1); // unblock the warningNotification task
    }
  }

  if (anodeOutputPin < NUM_OUTPUTS_PER_DRIVER && cathodeOutputPin < NUM_OUTPUTS_PER_DRIVER && anodeID < numDrivers && cathodeID < numDrivers) {

    // #ifdef DEBUG
    //   #if VERSION_IS_AT_LEAST(12, 1)
    //     // note that these take ~140us each
    //     analogWrite(LEDB, 0);
    //     analogWrite(LEDG, 0);
    //   #else
    //     digitalWrite(LED, HIGH);
    //   #endif
    // #endif

    unsigned long onDurationReduction = 0;

    #ifdef OVERTAP_PROTECTION
      uint8_t anodeIndex = anodeID * NUM_OUTPUTS_PER_DRIVER + anodeOutputPin;
      uint8_t cathodeIndex = cathodeID * NUM_OUTPUTS_PER_DRIVER + cathodeOutputPin;
      unsigned long interval = (micros() - tapperMonitor[anodeIndex][cathodeIndex].lastMicros); // TODO: catch timer overflow
      
      int currentHeat = tapperMonitor[anodeIndex][cathodeIndex].heat - interval/COOLING_DENOMINATOR;
      if (currentHeat < 0) currentHeat = 0; // don't let heat be negative

      if (currentHeat > ACCEPTABLE_HEAT) {
        uint32_t attenuation = (currentHeat - ACCEPTABLE_HEAT) / ATTENUATION_DENOMINATOR;
        onDurationUS = onDurationUS - attenuation - onDurationUS / ATTENUATION_DENOMINATOR;
        // ARGPRINTLN("", onDurationUS);
        // Create an overtap warning
        if (xSemaphoreTake(warningQMutex, 0) == pdTRUE) { // xBlockTime set to 0 because we don't want to block while tapping
          addToWarningQ(OVERTAP);
          addToWarningQ(anodeIndex);
          addToWarningQ(cathodeIndex);
          xSemaphoreGive(warningQMutex);
          xEventGroupSetBits(notificationEventGroup, EVENT_BIT1); // unblock the warningNotification task
        }
      }

      tapperMonitor[anodeIndex][cathodeIndex].heat = currentHeat + onDurationUS;
      tapperMonitor[anodeIndex][cathodeIndex].lastMicros = micros();

    #endif //OVERTAP_PROTECTION

    // attenuate the tap if the IMU is hot (which means the board is hot)
    switch (boardOverheatLevel) {
      case 0:
        break;
      case 1:
        onDurationReduction += onDurationUS / 4;
        onDurationUS = onDurationUS * 3 / 4;
        break;
      case 2:
        onDurationReduction += onDurationUS / 2;
        onDurationUS = onDurationUS / 2;
        break;
      case 3:
        onDurationReduction += onDurationUS * 3 / 4;
        onDurationUS = onDurationUS / 4;
        break;
      case 4:
        onDurationReduction += onDurationUS * 7 / 8;
        onDurationUS = onDurationUS / 8;
        break;
    }

    // containers for diagnostic data sent from drivers to the controller
    uint16_t anodeDiagnosticDataOn;
    uint16_t cathodeDiagnosticDataOn;
    uint16_t anodeDiagnosticDataOff;
    uint16_t cathodeDiagnosticDataOff;

    // prepare the output data for anode and cathode
    uint16_t anodeOutputRegister = driver[anodeID]->setOutput(anodeOutputPin, ANODE);
    uint16_t cathodeOutputRegister = driver[cathodeID]->setOutput(cathodeOutputPin, CATHODE);
  
    // turning the anode on:
    digitalWrite(driver[anodeID]->CS, LOW); // tell the anode chip we're about to write data to it
    SPI.beginTransaction(SPISettings(clockSpeed, MSBFIRST, SPI_MODE1)); // this takes a lock on the SPI bus, so take it as late as possible and give it up as early as possible (though I don't know if any other module uses this bus)
    anodeDiagnosticDataOn = SPI.transfer16(anodeOutputRegister);
    digitalWrite(driver[anodeID]->CS, HIGH); // this clocks the data in and the chip starts to act on it
    
    // turning the cathode on:
    digitalWrite(driver[cathodeID]->CS, LOW); // repeat for the cathode
    cathodeDiagnosticDataOn = SPI.transfer16(cathodeOutputRegister);
    digitalWrite(driver[cathodeID]->CS, HIGH); // as soon as this completes, the tapper will be powered. so start timing the onDuration immediately.
    
    // setting up for turning both off:
    uint32_t tStart = micros();
    anodeOutputRegister = driver[anodeID]->clrOutputVal(anodeOutputPin);
    cathodeOutputRegister = driver[cathodeID]->clrOutputVal(cathodeOutputPin);
    digitalWrite(driver[anodeID]->CS, LOW); // we can start to load this data into the driver chip before onDurationUS has elapsed, so we can turn it off with as little latency as possible
    anodeDiagnosticDataOff = SPI.transfer16(anodeOutputRegister);
    uint32_t elapsedTime = micros() - tStart;
    onDurationUS = elapsedTime < onDurationUS ? onDurationUS - elapsedTime : 0;
    delayMicroseconds(onDurationUS); // delay for the remainder of the onDuration
    
    // turn the anode off:
    digitalWrite(driver[anodeID]->CS, HIGH);

    // turn the cathode off:
    digitalWrite(driver[cathodeID]->CS, LOW);
    cathodeDiagnosticDataOff = SPI.transfer16(cathodeOutputRegister);
    SPI.endTransaction();
    digitalWrite(driver[cathodeID]->CS, HIGH);

    tStart = micros();

    // #ifdef DEBUG
    //   #if VERSION_IS_AT_LEAST(12, 1)
    //     analogWrite(LEDB, 255);
    //     analogWrite(LEDG, 255);
    //   #else
    //     digitalWrite(LED, LOW);
    //   #endif
    // #endif

    // check the diagnostic data - only checking diagnostic data from turning the outputs off, since the error flags won't reset themselves (except TW and PSF which likely wounldn't reset while the tap is active)
    uint8_t anodeChannelSelectBit = (anodeOutputRegister >> driver[anodeID]->CH_SEL) & 1;
    uint8_t cathodeChannelSelectBit = (cathodeOutputRegister >> driver[cathodeID]->CH_SEL) & 1;
    bool sendWarningForAnode = checkDiagnosticData(anodeDiagnosticDataOff, anodeID, anodeChannelSelectBit);
    bool sendWarningForCathode = false;
    if ((cathodeID != anodeID) || (cathodeChannelSelectBit != anodeChannelSelectBit)) sendWarningForCathode = checkDiagnosticData(cathodeDiagnosticDataOff, cathodeID, cathodeChannelSelectBit);

    if (sendWarningForAnode || sendWarningForCathode) {
      // add any warnings to the warning queue
      assert(xSemaphoreTake(warningQMutex, portMAX_DELAY) == pdTRUE);
      DPRINTLN("new Hbrdige driver warning");
      if (sendWarningForAnode) {
        addToWarningQ(HBRIDGE_DIAGNOSTIC_ERRORS);
        addToWarningQ(anodeID);
        addToWarningQ(driver[anodeID]->errorFlags);
      }
      if (sendWarningForCathode) {
        addToWarningQ(HBRIDGE_DIAGNOSTIC_ERRORS);
        addToWarningQ(cathodeID);
        addToWarningQ(driver[cathodeID]->errorFlags);
      }
      xSemaphoreGive(warningQMutex);
      xEventGroupSetBits(notificationEventGroup, EVENT_BIT1); // unblock the warningNotification task
    }

    if (onDurationReduction) offDurationUS += onDurationReduction;
    elapsedTime = micros() - tStart; // from doing the diagnostic data error checking
    if (offDurationUS > elapsedTime) offDurationUS -= elapsedTime;
  }
  else { // if it's an empty tap - ie the tapper is out of bounds. delay the same amount of time so that if there are in-bound taps it doesn't mess with the pattern cadence (which would be more confusing to debug as a designer, I think)
    offDurationUS += onDurationUS;
    // DPRINTLN("Empty tap");
  }
  if (offDurationUS) delayMicroseconds(offDurationUS);
}

/*
Helper function to check the TapQueue object (which is private).
*/
bool TapHandler::isDoneTapping() {
  return tapQ.isEmpty();
}

/*
Helper function to collect all the status data.
*/
std::vector<uint8_t> TapHandler::getStatus() {
  std::vector<uint8_t> status(3);
  status[0] = currentTapOutID;
  status[1] = (uint8_t)(tapQ.headroom() >> 8);
  status[2] = (uint8_t)tapQ.headroom();
  return status;
}

/*
Loads the stressTestVect from configuration.cpp as if it were a message we received from central. 
*/
void TapHandler::stressTest() {
  if (tapQ.isEmpty()) {
    receiveTapOut(stressTestVect);
  }
}

//---------------------- Tap Queue Class ----------------------

// helper function
bool TapQueue::isFull() {
  return getSize() == MAX_QUEUE_SIZE;
}

// helper function
bool TapQueue::isEmpty() {
  return getSize() == 0;
}

// helper function, returns the number of additional taps that we have room for.
uint16_t TapQueue::headroom() {
  return MAX_QUEUE_SIZE - getSize();
}

// helper function
uint16_t TapQueue::getSize() {
  assert(xSemaphoreTake(tapQMutex, portMAX_DELAY) == pdTRUE);
  uint16_t currentSize = size;
  xSemaphoreGive(tapQMutex);
  return currentSize;
}

// adds a new tap index + settings to the back of the queue (FIFO)
bool TapQueue::push(TapSettings newTap) {
  if (isFull()) {
      return false;
  }
  assert(xSemaphoreTake(tapQMutex, portMAX_DELAY) == pdTRUE);
  taps[rear] = newTap;
  rear = (rear + 1) % MAX_QUEUE_SIZE; // wrap around if at the end
  ++size;
  xSemaphoreGive(tapQMutex);
  return true;
}

// removes the frontmost tap + settings from the queue (FIFO)
TapSettings TapQueue::pop() {
  TapSettings tap = {};
  if (isEmpty()) {
      return tap;
  }
  assert(xSemaphoreTake(tapQMutex, portMAX_DELAY) == pdTRUE);
  tap = taps[front];
  front = (front + 1) % MAX_QUEUE_SIZE; // wrap around if at the end
  --size; 
  xSemaphoreGive(tapQMutex);
  return tap;
}

// empties the queue
void TapQueue::clear() {
  assert(xSemaphoreTake(tapQMutex, portMAX_DELAY) == pdTRUE);
  size = 0;
  front = 0;
  rear = 0;
  xSemaphoreGive(tapQMutex);
}