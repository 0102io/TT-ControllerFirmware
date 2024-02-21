#include "HBDriver.h"

const uint8_t HBDriver::HBENbit[] = {7, 8, 9, 10, 11, 12, 7, 8, 9, 10}; // bit positions of the enable values for outtputs 1-10
const uint8_t HBDriver::HBCNFbit[] = {1, 2, 3, 4, 5, 6, 1, 2, 3, 4}; // bit positions of the config values for outputs 1-10

HBDriver::HBDriver(uint8_t cs) {
    CS = cs;
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);

    // Set these initial values
    // statusRegReset = 0;
    // openLoadShutdown = 0;
    // overVoltageLockout = 0;
    outputRegister1to6 |= (0 << SRR);
    outputRegister1to6 |= (0 << CH_SEL);
    outputRegister1to6 |= (0 << OLSD_EN);
    outputRegister1to6 |= (0 << OVLO);
    outputRegister7to10 |= (0 << SRR);
    outputRegister7to10 |= (1 << CH_SEL);
    outputRegister7to10 |= (0 << OLSD_EN);
    outputRegister7to10 |= (0 << OVLO);
}

uint16_t HBDriver::setOutput(uint8_t out, uint8_t cnf) {
    uint16_t &outputRegister = (out < 6) ? outputRegister1to6 : outputRegister7to10; 
    outputRegister |= (1 << HBENbit[out]);
    if (cnf) outputRegister |= (1 << HBCNFbit[out]); // set the bit
    else outputRegister &= ~(1 << HBCNFbit[out]); // clear the bit
    return outputRegister;
}

uint16_t HBDriver::clrOutputVal(uint8_t out) {
  uint16_t &outputRegister = (out < 6) ? outputRegister1to6 : outputRegister7to10; 
  outputRegister &= ~(1 << HBENbit[out]);
  return outputRegister;
}
