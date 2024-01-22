#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>

/*
Adafruit definitions: this stuff was copied/modified from adafruit's feather esp32-s3 no psram
*/

#define USB_VID            0x303A // espressif VID; use default USB_PID since TinyUSB with default drivers seems to work fine for us. see: https://github.com/espressif/usb-pids
#define USB_MANUFACTURER   "0102"
#define USB_PRODUCT        "TappyTap Controller v12b"
#define USB_SERIAL         "" // Empty string for MAC adddress

#define EXTERNAL_NUM_INTERRUPTS 46
#define NUM_DIGITAL_PINS        48
#define NUM_ANALOG_INPUTS       20

#define analogInputToDigitalPin(p)  (((p)<20)?(analogChannelToDigitalPin(p)):-1)
#define digitalPinToInterrupt(p)    (((p)<48)?(p):-1)
#define digitalPinHasPWM(p)         (p < 46)

#define LED_BUILTIN         13

#define I2C_POWER           7     // I2C power pin
#define PIN_I2C_POWER       7     // I2C power pin


static const uint8_t TX = 39;
static const uint8_t RX = 38;
#define TX1 TX
#define RX1 RX

static const uint8_t SDA = 3;
static const uint8_t SCL = 4;

static const uint8_t SS    = 42;
static const uint8_t MOSI  = 35;
static const uint8_t SCK   = 36;
static const uint8_t MISO  = 37;

static const uint8_t A0 = 18;
static const uint8_t A1 = 17;
static const uint8_t A2 = 16;
static const uint8_t A3 = 15;
static const uint8_t A4 = 14;
static const uint8_t A5 = 8;
static const uint8_t A6 = 3;
static const uint8_t A7 = 4;
static const uint8_t A8 = 5;
static const uint8_t A9 = 6;
static const uint8_t A10 = 9;
static const uint8_t A11 = 10;
static const uint8_t A12 = 11;
static const uint8_t A13 = 12;
static const uint8_t A14 = 13;

static const uint8_t T3 = 3;
static const uint8_t T4 = 4;
static const uint8_t T5 = 5;
static const uint8_t T6 = 6;
static const uint8_t T8 = 8;
static const uint8_t T9 = 9;
static const uint8_t T10 = 10;
static const uint8_t T11 = 11;
static const uint8_t T12 = 12;
static const uint8_t T13 = 13;
static const uint8_t T14 = 14;

/*
TappyTap definitions
*/
#define HARDWARE_VERSION_MAJOR 12
#define HARDWARE_VERSION_MINOR 1 // 0 based; i.e. "A" is 0

#define LED 13
#define LEDG 11
#define LEDB 14
#define BOOT_MODE 16
#define CURRENT_SENSE_PIN 16 // electrically connected to 39, but hacked to 16 for v12b testing (note overlap with boot mode)
#define TOUCH_PIN T10
#define USER_BUTTON 12
#define EXT_PIN 18 // pin connected to the battery pack to let the pack know the driver is connected
#define HBEN_INTERRUPT_PIN 38 // HB driver enable pin, driven by the INA169 (current dependent voltage source)
#define REG12V_EN_PIN 42 // 12V reg enable pin
#define IMU_INT1_PIN 9
#define IMU_INT2_PIN 7
#define SDA1 3
#define SCL1 4

static const uint8_t cs_HBD0 = 17;
static const uint8_t cs_HBD1 = 41;
static const uint8_t cs_HBD2 = 40;
static const uint8_t CSPin[] = {cs_HBD0, cs_HBD1, cs_HBD2};
static const uint8_t numDrivers = sizeof(CSPin)/sizeof(CSPin[0]);

#endif /* Pins_Arduino_h */
