#ifndef TAPHANDLER_H
#define TAPHANDLER_H

#include "configuration.h"
#include "HBDriver.h"
#include <SPI.h>
#include <array>

#define EMPTY_TAP 15 // if we try to tap somewhere out of bounds
#define NO_TAPOUT_ID 0 // if we don't have a tapout pattern that we're currently using

/*
Tracks the "heat" for each element in the tap matrix, which we use to make sure we 
don't drive them too hard. See description for OVERTAP_PROTECTION for more info.
*/
struct TapperMonitor {
    unsigned long lastMicros;
    uint32_t heat;
};

struct TapSettings {
    uint8_t anodeCS;
    uint8_t cathodeCS;
    uint8_t anodeOutputPin;
    uint8_t cathodeOutputPin;
    unsigned long onDuration;
    unsigned long offDuration;
};

/*
Circular FIFO buffer that stores the queue of tap indicies and on + off durations.

TODO: this doesn't need an attay for each of the settings, we could just use the same structure as 
the message protocol - TapHandler::tap() would have to be updated accordingly.
*/
class TapQueue {
private:
    uint16_t size;
    uint16_t front;
    uint16_t rear;
    std::array<TapSettings, MAX_QUEUE_SIZE> taps;

public:
    TapQueue() : size(0), front(0), rear(0) {}
    bool isFull();
    bool isEmpty();
    uint16_t headroom();
    uint16_t getSize();
    bool push(TapSettings newTap);
    TapSettings pop();
    void clear();
};

/*
This class is responsible for managing everything to do with taps, including:
- error checking patterns sent by the bluetooth central
- keeping track of all remaining taps (using a TapQueue object)
- performing the taps (by writing to the h-bridge drivers; also checking for errors reported by them)
*/
class TapHandler {
public:
    TapHandler();
    void setupTapHandler();
    void cancelAndTap(std::vector<uint8_t> msg);
    void receiveTapOut(std::vector<uint8_t> msg);
    void tap();
    void stressTest();
    bool isDoneTapping();
    std::vector<uint8_t> getStatus();

private:
    const uint32_t clockSpeed = 2000000;

    uint16_t lastOnDur;
    uint16_t lastOffDur;

    uint8_t currentTapOutID;

    // H bridge driver object array
    HBDriver** driver;

    // queue of taps to tap
    TapQueue tapQ;

    // overtap protection
    TapperMonitor** tapperMonitor;

    // Current sensing - not being used right now
    const float VtoCgain = 5.6; // V per A
    float adcConversionFactor; // adcConversionFactor = (3.3 * 1000) / (4095 * VtoCgain);

    void addToQueue(std::vector<uint8_t> data);
    void writeOutput(uint8_t id, uint8_t outputNumber);
    void checkDiagnosticReg(uint16_t returnedData, uint8_t id, uint16_t diagnosticRegister);
    void srrReset(uint8_t id, uint16_t diagnosticRegister);
};

#endif // TAPHANDLER_H
