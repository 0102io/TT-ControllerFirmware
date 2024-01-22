#include "configuration.h"
#include "TapHandler.h"
#include "utils.h"

//---------------------- Tap Handler Class ----------------------

TapHandler::TapHandler() {
    lastOnDur = onDur_init;
    lastOffDurM = (uint8_t)(offDur_init >> 8);
    lastOffDurL = (uint8_t)offDur_init;
}

void TapHandler::setupTapHandler() {
    // Construct H bridge driver objects
    driver = new HBDriver*[numDrivers];
    for (int i = 0; i < numDrivers; i++) {
        driver[i] = new HBDriver(CSPin[i]);
    }

    // Create array of TapperMonitors
    tapperMonitor = new TapperMonitor*[numRows];
    for(int i = 0; i < numRows; ++i) {
        tapperMonitor[i] = new TapperMonitor[numCols];
        for (int j = 0; j < numCols; j++) {
          tapperMonitor[i][j].lastMillis = 0;
          tapperMonitor[i][j].heat = 0;
        }
    }


    // SPI setup
    SPI.begin();

    // Clear error bits on each register, seems to always start with PSF+OC faults?
    uint16_t outputs1to6 = 0;
    uint16_t outputs7to10 = (1 << 14);
    for (int i = 0; i < numDrivers; i++) {
        srrReset(i, outputs1to6);
        delay(150); // need to delay at least 100ms between successive SRR commands to the same IC
        srrReset(i, outputs7to10);
    }
    
    currentTapOutID = NO_TAPOUT_ID;
    halfFullRising = false;
    halfFullFalling = false;
    overtappedRowIndex = EMPTY_TAP;
    overtappedColIndex = EMPTY_TAP;
}

/*
This method is called by comms manager when a CANCEL_AND_TAP message is received from central
*/
void TapHandler::cancelAndTap(std::vector<uint8_t> msg) {
  // cancel queue and call normal tapout function
    disableTapTimer();
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
    disableTapTimer();
    tapQ.clear();
    setTapTimer(1); // set it to execute almost immediately (let this task finish first)
    return;
  }
  else {
    // add the data into the buffer, and take the new tapout id
    DPRINTLN("adding pattern to buffer");
    addToQueue(msg);
    if (!tapTimerEnabled()) {
      setTapTimer(1); // if we aren't already tapping then start the timer
      #ifdef REGULATOR_PWR_SAVE
        digitalWrite(REG12V_EN_PIN, HIGH); // also enable the 12V regulator
      #endif
    }
    currentTapOutID = id; // even though we haven't necessarily finished the last pattern, we change the pattern ID to the most recently received one
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
*/
void TapHandler::addToQueue(std::vector<uint8_t> data) {

  // temp variables for checking if the received parameters are in bounds
  uint8_t r;
  uint8_t c;
  uint8_t on;
  uint8_t offM;
  uint8_t offL;

  // read each row and col into tap arrays
  int len = (int) data.size();
  if (((tapQ.getSize() + len) > (MAX_QUEUE_SIZE / 2)) && !halfFullRising) halfFullRising = true; // let central know we just passed 50% full threshold
  
  uint16_t i; // used after the loop

  for (i = 0; i < len; i+= 2) {
    if (data[i] & 0x80) { // 1 as the most significant bit in the row index means we should expect to read settings
      // this block will contain row, col, onDur, offDur
      if (i + 4 < len) {
        r = (data[i] & 0x7F); // get rid of the settings indicator bit
        c = data[i+1];
        on = data[i+2];
        offM = data[i+3];
        offL = data[i+4];

        // check the parameters to make sure they're in bounds
        if (r > numRows - 1) {
          r = EMPTY_TAP;
          warningCode = PARAM_OOB;
          warningValue = i;
          ARGPRINTLN("Error: row index OOB. Index: ", i);
        }
        if (c > numCols - 1) {
          c = EMPTY_TAP;
          warningCode = PARAM_OOB;
          warningValue = i + 1;
          ARGPRINTLN("Error: col index OOB. Index: ", i + 1);
        }
        if (on > onDur_max) {
          on = onDur_max;
          warningCode = PARAM_OOB;
          warningValue = i + 2;
          ARGPRINTLN("Error: onDur OOB. Index: ", i + 2);
        }
        // no need to check offDur, any 16 bit value is valid
        

        if (!tapQ.isFull()) tapQ.push(r, c, on, offM, offL);
        else {
          firstRejectedIndex = i;
          ARGPRINTLN("Queue full. Index of first rejected tap: ", i);
          return;
        }
        lastOnDur = data[i+2];
        lastOffDurM = data[i+3];
        lastOffDurL = data[i+4];
        i+=3; // increment past the extra settings
      }
      else {
        // message size is incorrect; clear the queue since the whole message is likely not formatted according to protocol
        tapQ.clear();
        warningCode = INCORRECT_MSG_SIZE;
        DPRINTLN("Error: incorrect message size.");
        return;
      }
    }
    else {
      r = data[i];
      c = data[i+1];

      // check the parameters to make sure they're in bounds
      if (r > numRows - 1) {
        r = EMPTY_TAP;
        warningCode = PARAM_OOB;
        warningValue = i;
        ARGPRINTLN("Error: row index OOB. Index: ", i);
      }
      if (c > numCols - 1) {
        c = EMPTY_TAP;
        warningCode = PARAM_OOB;
        warningValue = i + 1;
        ARGPRINTLN("Error: col index OOB. Index: ", i + 1);
      }

      if (!tapQ.isFull()) tapQ.push(r, c, lastOnDur, lastOffDurM, lastOffDurL);
      else {
        firstRejectedIndex = i;
        ARGPRINTLN("Queue full. Index of first rejected tap: ", i);
        return;
      }

    }
  }
  firstRejectedIndex = i; // send the position of the nth + 1 index
  return;
}


/*
This is the method that puts together the SPI message that is sent to the H-Bridge drivers. It turns on the output according to
outputNumber (1-based, just like the datasheet). The data it sends is pulled from what is stored in the HBDriver object's output
registers.

The H-bridge ICs send data back on the SPI bus, which we check for errors.
*/
void TapHandler::writeOutput(uint8_t id, uint8_t outputNumber) {
  uint8_t cs = driver[id]->CS;
  uint16_t outReg;
  if (outputNumber < 7) outReg = driver[id]->outputRegister1to6;
  else outReg = driver[id]->outputRegister7to10;
  uint16_t retData;
  // TODO: the SPI transaction seems to take 50-150us (usually 50-70 but the last call to turn off a col takes 150-200us sometimes, not sure why). This is with SPI clockspeed set to 2MHz and core speed 80MHz.
  digitalWrite(cs, LOW);
  SPI.beginTransaction(SPISettings(clockSpeed, MSBFIRST, SPI_MODE1));
  retData = SPI.transfer16(outReg);
  SPI.endTransaction();
  digitalWrite(cs, HIGH);
  checkDiagnosticReg(retData, id, outReg);
}

/*
This method parses data that was written to the controller by the last h-bridge driver we messaged. It contains a couple of 
possible error notifications, none of which we currently act on.
*/
void TapHandler::checkDiagnosticReg(uint16_t returnedData, uint8_t id, uint16_t diagnosticRegister) {
  uint8_t cs = driver[id]->CS;
  bool srrAlreadyReset = false; // make sure we don't reset the register twice if there are 2 errors
  for(uint8_t i = 0; i < 16; i++) {
    if(returnedData & (1 << i)) {
      switch(i) {
        // case 13: 
          // ARGPRINT("Fault detected for CS pin ", cs);
          // if (diagnosticRegister & (1 << 14)) DPRINT(", outputs 7-10: OLD");
          // else DPRINT(", outputs 1-6: OLD");
          // if (!srrAlreadyReset) srrReset(id, diagnosticRegister);
          // srrAlreadyReset = true;
        case 14: 
          ARGPRINT("Fault detected for CS pin ", cs);
          if (diagnosticRegister & (1 << 14)) DPRINT(", outputs 7-10: PSF");
          else DPRINT(", outputs 1-6: PSF");
          if (!srrAlreadyReset) srrReset(id, diagnosticRegister); // TODO - this should set a timer to do an SRR reset so we don't have to wait for the next time this method is called to try again.
          srrAlreadyReset = true;
        case 15: 
          ARGPRINT("Fault detected for CS pin ", cs);
          if (diagnosticRegister & (1 << 14)) DPRINT(", outputs 7-10: OC");
          else DPRINT(", outputs 1-6: OC");
          if (!srrAlreadyReset) srrReset(id, diagnosticRegister);
          srrAlreadyReset = true;
      }
    }
  }
}

/*
This method resets the status register on an h-bridge driver chip, which allows it to send new error messages. The chip has a 100ms 
cooldown between successive SRR writes: https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MP6527GF/document_id/10142/
*/
void TapHandler::srrReset(uint8_t id, uint16_t diagnosticRegister) {
  unsigned long resetTimer = driver[id]->SRRtimer;
  unsigned long currentMillis = millis();
  if (currentMillis > resetTimer) {
    uint8_t cs = driver[id]->CS;
    int chSel = diagnosticRegister & (1 << 14);
    ARGPRINT("Resetting SRR for driver: ", id);
    if (chSel) DPRINTLN(", HB7-10");
    else DPRINTLN(", HB1-6");

    uint16_t resetRegister = (1 << 15);
    if (chSel) resetRegister = resetRegister | (1 << 14);
    
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
  disableTapTimer();
  std::array<uint8_t, 5> currentTap = tapQ.pop();

  // load settings for this tap
  uint8_t r = currentTap[0];
  uint8_t c = currentTap[1];
  unsigned long onDuration = currentTap[2];
  unsigned long offDuration = (currentTap[3] << 8) | currentTap[4];

  // temp values
  uint8_t rowParentID;
  uint8_t colParentID;
  uint8_t rowOutputPin;
  uint8_t colOutputPin;
  uint8_t rowPolarity;
  uint8_t colPolarity;

  if (r != EMPTY_TAP && c != EMPTY_TAP) {
    rowParentID = rows[r].parent;
    colParentID = cols[c].parent;
    rowOutputPin = rows[r].outputPin;
    colOutputPin = cols[c].outputPin;
    rowPolarity = rows[r].polarity;
    // rowPolarity is either 0 or 1 (CATHODE or ANODE), and we need colPolarity to be the opposite, and we can use XOR to get this:
    // 1 XOR 1 = 0
    // 0 XOR 1 = 1
    colPolarity = rowPolarity ^ 1; // ^ is the XOR operator

    // Set the row and col outputs
    #ifdef DEBUG
      #if VERSION_IS_AT_LEAST(12, 1)
        analogWrite(LED, 0);
      #else
        digitalWrite(LED, HIGH);
      #endif
    #endif

    #ifdef OVERTAP_PROTECTION
      unsigned long interval = (millis() - tapperMonitor[r][c].lastMillis) * 10; // tenths of a millisecond
      int lastHeat = tapperMonitor[r][c].heat;

      unsigned long onDurReduction = onDuration * lastHeat / (interval * ATTENUATION_CONSTANT); 
      if (onDurReduction != 0) {
        overtappedRowIndex = r;
        overtappedColIndex = c;
      }
      unsigned long attenuatedOnDur;
      if (onDurReduction < onDuration) attenuatedOnDur = onDuration - onDurReduction; // since we're doing unsigned integer math we have to make sure it doesn't go negative
      else attenuatedOnDur = 0;
      
      unsigned long newOnDuration = (attenuatedOnDur < onDuration) ? attenuatedOnDur : onDuration; // don't tap longer than intended
      unsigned long balance = (newOnDuration < onDuration) ? onDuration - newOnDuration : 0; // pause for this amount of time after the tap so the pattern cadance isn't messed up
      
      int newHeat = onDuration * ON_DURATION_MULTIPLIER - interval + lastHeat;
      
      tapperMonitor[r][c].heat = (newHeat > 0) ? newHeat : 0; // don't let heat be negative
      tapperMonitor[r][c].lastMillis = millis();

    #endif //OVERTAP_PROTECTION


    driver[rowParentID]->setOutputVal(rowOutputPin);
    driver[rowParentID]->setOutputCNF(rowOutputPin, rowPolarity);
    driver[colParentID]->setOutputVal(colOutputPin);
    driver[colParentID]->setOutputCNF(colOutputPin, colPolarity);
    writeOutput(rowParentID, rowOutputPin);
    writeOutput(colParentID, colOutputPin);
    delayMicroseconds(onDuration*100);
    driver[rowParentID]->clrOutputVal(rowOutputPin);
    driver[colParentID]->clrOutputVal(colOutputPin);
    writeOutput(rowParentID, rowOutputPin);
    writeOutput(colParentID, colOutputPin);


    #ifdef DEBUG
      #if VERSION_IS_AT_LEAST(12, 1)
        analogWrite(LED, 255);
      #else
        digitalWrite(LED, LOW);
      #endif
    #endif
    #ifdef OVERTAP_PROTECTION
      delayMicroseconds(balance*100);
    #endif //OVERTAP_PROTECTION
  }
  else { // if it's an empty tap - ie the tapper is out of bounds. delay the same amount of time so that if there are in-bound taps it doesn't mess with the pattern cadence (which would be more confusing to debug as a designer, I think)
    delayMicroseconds(onDuration*100);
  }
  setTapTimer(offDuration);
  if (halfFullRising && (tapQ.getSize() < (MAX_QUEUE_SIZE / 2))) {
    halfFullFalling = true;
    halfFullRising = false;
    setStatusTimer(1); // notify central that we have more than 50% space in the queue
  }
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
  std::vector<uint8_t> status(11);
  status[0] = currentTapOutID;
  status[1] = (uint8_t)halfFullFalling;
  if (halfFullFalling) halfFullFalling = false; // reset the flag once it's been sent
  status[2] = (uint8_t)(tapQ.headroom() >> 8);
  status[3] = (uint8_t)tapQ.headroom();
  status[4] = (uint8_t)(firstRejectedIndex >> 8);
  status[5] = (uint8_t)firstRejectedIndex;
  status[6] = warningCode;
  status[7] = (uint8_t)(warningValue >> 8);
  status[8] = (uint8_t)warningValue;
  status[9] = overtappedRowIndex;
  overtappedRowIndex = EMPTY_TAP; // reset the notification
  status[10] = overtappedColIndex;
  overtappedColIndex = EMPTY_TAP;
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
  return size == MAX_QUEUE_SIZE;
}

// helper function
bool TapQueue::isEmpty() {
  return size == 0;
}

// helper function, returns the number of additional taps that we have room for.
uint16_t TapQueue::headroom() {
  return MAX_QUEUE_SIZE - size;
}

// helper function
uint16_t TapQueue::getSize() {
  return size;
}

// adds a new tap index + settings to the back of the queue (FIFO)
bool TapQueue::push(uint8_t r, uint8_t c, uint8_t on, uint8_t offM, uint8_t offL) {
  if (isFull()) {
      return false;
  }

  row[rear] = r;
  col[rear] = c;
  onDur[rear] = on;
  offDurMSB[rear] = offM;
  offDurLSB[rear] = offL;
  rear = (rear + 1) % MAX_QUEUE_SIZE; // wrap around if at the end
  ++size;
  return true;
}

// removes the frontmost tap + settings from the queue (FIFO)
std::array<uint8_t, 5> TapQueue::pop() {
  std::array<uint8_t, 5> arr = {0, 0, 0, 0, 0}; // we should never have to return this, but just in case we do, all of the values are in bounds so we won't crash
  if (isEmpty()) {
      return arr;
  }

  arr[0] = row[front];
  arr[1] = col[front];
  arr[2] = onDur[front];
  arr[3] = offDurMSB[front];
  arr[4] = offDurLSB[front];
  front = (front + 1) % MAX_QUEUE_SIZE; // wrap around if at the end
  --size;

  return arr;
}

// empties the queue
void TapQueue::clear() {
  size = 0;
  front = 0;
  rear = 0;
}