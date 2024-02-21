#ifndef HBDRIVER_H
#define HBDRIVER_H

#include <Arduino.h>

class HBDriver {
public:
    HBDriver(uint8_t cs);

    uint16_t setOutput(uint8_t out, uint8_t cnf);
    uint16_t clrOutputVal(uint8_t out);

    uint8_t CS;
    uint8_t output[10];
    uint8_t outputCNF[10];
    unsigned long SRRtimer = 0;

    // Define the output register bits for the MP6527GF H bridge driver ICs
    uint16_t outputRegister1to6 = 0;
    uint16_t outputRegister7to10 = 0;

    // Datasheet: https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MP6527GF/document_id/10142/
    // note to self: static members are shared between all instances of the class
    static const uint8_t SRR = 15; // Status register reset. Set to 1 to reset the error bits. Note time between consecutive SRR commands is 100ms.
    static const uint8_t CH_SEL = 14; // Channel select: 0 = bits 6:1, 1 = bits 10:7
    static const uint8_t OLSD_EN = 13; // Open-load detection shutdown enable bit. The allows the output stage to switch off if a "true" open load or under-load condition is detected.
    static const uint8_t HBENbit[]; // Enable bits for H bridges. 0 = Hi-Z, 1 = active. HBENbit[0] is for HBEN1 in the datasheet. HBENbit[9] is for HBEN10 in the datasheet.
    static const uint8_t HBCNFbit[]; // Bridge config bits: 0 = HS half bridge off, LS half bridge on; 1 = HS half bridge on, LS half bridge off. HBCNFbit[0] is for HBCNF1 in the datasheet, HBCN[9] is for HBCNF10 in the datasheet.
    static const uint8_t OVLO = 0; // Over-voltage lockout enable bit.

    uint8_t statusRegReset;
    uint8_t channelSelect;
    uint8_t openLoadShutdown;
    uint8_t overVoltageLockout;
};

#endif // HBDriver_H