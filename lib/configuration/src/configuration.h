#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include <vector>
#include <Arduino.h>

#define DEBUG // uncomment this line to get serial print statements on the USB bus; it also disables auto light sleep since serial doesn't work with that
// #define STRESS_TEST // uncomment this line to make the controller repeatedly tap the pattern in stressTestVect, using init settings in configuration.cpp

#define FIRMWARE_VERSION_MAJOR 0
#define FIRMWARE_VERSION_MINOR 4
#define FIRMWARE_VERSION_PATCH 2

#define VERSION_IS(MAJOR, MINOR) (HARDWARE_VERSION_MAJOR == (MAJOR) && HARDWARE_VERSION_MINOR == (MINOR))
#define VERSION_IS_AT_LEAST(MAJOR, MINOR) ((HARDWARE_VERSION_MAJOR > (MAJOR)) || (HARDWARE_VERSION_MAJOR == (MAJOR) && HARDWARE_VERSION_MINOR >= (MINOR)))

#define PATCH 1
#define PALM 2

/*
These are the default substrate settings for a newly flashed controller, but they can be overwritten with 
a message from central and will be stored in non volatile memory. 
e.g. v12d patch = PATCH, 12, 3
*/
#define DEFAULT_SUBSTRATE PATCH
#define DEFAULT_SUBSTRATE_VMAJOR 12
#define DEFAULT_SUBSTRATE_VMINOR 3

#define MAX_QUEUE_SIZE 2048 // max number of row/col pairs in a tapout pattern
// #define CPU_CLK_FREQ_OVERIDE 240 // MHz; set in esp-idf config as 80

/*
Overtap protection limits high duty cycle tapping of each actuator.
It does this by calculating a value called "heat" which builds up with large onDurations with insufficient off time to cool down.
The values are currently balanced around an assumed safe continuous duty cycle of 3ms onDuration / 51ms offDuration repeated on a single tapper.
See this sheet for sample calculations: https://docs.google.com/spreadsheets/d/15yVmE13jGJZapsrEjx4v79JR5FFSrgQL-_rgFeacN8Q/edit?usp=sharing
*/
#define OVERTAP_PROTECTION // currently has a bug causing the board to crash when on and off durations are both very short
#ifdef OVERTAP_PROTECTION
  #define COOLING_DENOMINATOR 100
  #define ACCEPTABLE_HEAT 100000 // equivalent number of microseconds of being on continuously
  #define ATTENUATION_DENOMINATOR 25
#endif // OVERTAP_PROTECTION

// monitors the h-bridge enable net and does a print statement if it gets pulled low (by the overcurrent protection or watchdog circuits).
#define HBEN_DISABLED_INTERRUPT

#define MAX_TRANSMISSION_FREQUENCY 15 // Hz
#define DEFAULT_TRANSMISSION_FREQUENCY 10 // Hz

/*
Automatic light sleep and modem sleep idf options added to defconfig.esp32s3 with esp32-arduino-lib-builder:
# From power save example: https://github.com/espressif/esp-idf/blob/master/examples/wifi/power_save/sdkconfig.defaults
# Enable support for power management
CONFIG_PM_ENABLE=y
# Enable bt/wifi modem sleep
CONFIG_BT_CTRL_MODEM_SLEEP=y
CONFIG_BT_CTRL_MODEM_SLEEP_MODE_1=y
# Enable tickless idle mode
CONFIG_FREERTOS_USE_TICKLESS_IDLE=y
# Put related source code in IRAM
CONFIG_PM_SLP_IRAM_OPT=y
CONFIG_PM_RTOS_IDLE_OPT=y
# Enable wifi sleep iram optimization
CONFIG_ESP_WIFI_SLP_IRAM_OPT=y
# Use 1000Hz freertos tick to lower sleep time threshold
CONFIG_FREERTOS_HZ=1000
# Disable all GPIO at light sleep
CONFIG_GPIO_ESP32_SUPPORT_SWITCH_SLP_PULL=y
CONFIG_PM_SLP_DISABLE_GPIO=y
# from: https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/nimble/power_save/sdkconfig.40m.esp32s3
# Enable power down of MAC and baseband in light sleep mode
CONFIG_ESP_PHY_MAC_BB_PD=y
# Bluetooth low power clock
CONFIG_BT_CTRL_LPCLK_SEL_MAIN_XTAL=y
# Power up main XTAL during light sleep
CONFIG_BT_CTRL_MAIN_XTAL_PU_DURING_LIGHT_SLEEP=y
*/

#ifndef DEBUG
  /*
  Allows the ESP to automatically enter light sleep mode. See: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/power_management.html
  This disables serial (because light sleep turns off the APB - see related thread:  https://github.com/espressif/esp-idf/issues/8507). 
  */
  #define AUTO_LIGHT_SLEEP
#endif

#if VERSION_IS_AT_LEAST(12, 1)
  /*
  This disables the 12V regulator (which is only connected to the h-bridge drivers) when there are no taps,
  and re-enables it as we're about to start tapping.
  */
  #define REGULATOR_PWR_SAVE
#endif

#if VERSION_IS_AT_LEAST(12, 3)
  /*
  WATCHDOG enables and disables the STWD100YNX watchdog IC. This will pull the H bridge driver enable pins low if 
  the ESP crashes before it turns off the outputs, which would otherwise cause a coil to stay on and get really hot.
  Currently the watchdog IC is not connected to the HB driver enable pins by default (the overcurrent protection circuit
  is instead). To change this, cut the trace between the middle pad and the "OC" pad on the jumper next to the FPC connector,
  and solder the middle pad to the "WD" pad.
  */
  #define WATCHDOG
  #define WATCHDOG_PET_INTERVAL 50 // ms
#endif

// used for setting the outputCNF for the h bridge drivers
#define ANODE 1 // 1 = HS fet enabled, LS fet disabled on h bridge driver
#define CATHODE 0 // 0 = LS fet enabled, HS fet disabled on h bridge driver

#define MAX_INACTIVITY_MINUTES 10 // go to sleep after this many minutes

// tap parameter initial and limit values
static const unsigned long onDur_init = 300; // tens of microseconds (hundreds of a millisecond)
static const unsigned long onDur_max = 300;
static const unsigned long offDur_init = 1000; // tenths of a ms

// stress test vector
static const std::vector<uint8_t> stressTestVect = {
    0x01, 0x01, // message tpye, tapoutID
    0x00, 0x01, // on duration, hundredths of a ms
    0x00, 0x00, // off duration, tenths of a ms
    0x02, 0x09 // anode = 0b0000 0010 (cs = 0, pin = 2), cathode = 0b0000 1001 (cs = 0, pin = 9) --> on-board test points for v12d/e
};

// touch pin detection thresholds
static const int caseTouchThreshold = 34600; // needs tuning
static const int fpcTouchThreshold = 34600; // needs tuning

#endif // CONFIGURATION_H