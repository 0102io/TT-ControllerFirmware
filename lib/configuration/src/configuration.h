#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include <vector>
#include <Arduino.h>

#define DEBUG // uncomment this line to get serial print statements on the USB bus
// #define STRESS_TEST // uncomment this line to make the controller repeatedly tap the pattern in stressTestVect, using init settings in configuration.cpp

#define FIRMWARE_VERSION_MAJOR 0
#define FIRMWARE_VERSION_MINOR 4
#define FIRMWARE_VERSION_PATCH 2

#define VERSION_IS(MAJOR, MINOR) (HARDWARE_VERSION_MAJOR == (MAJOR) && HARDWARE_VERSION_MINOR == (MINOR))
#define VERSION_IS_AT_LEAST(MAJOR, MINOR) ((HARDWARE_VERSION_MAJOR > (MAJOR)) || (HARDWARE_VERSION_MAJOR == (MAJOR) && HARDWARE_VERSION_MINOR >= (MINOR)))

#define SWATCH 0 // TODO this should start at 1 instead because undefined things will evaluate as 0
#define PALM 1
#define R20C20 97
#define ONBOARD 98
#define CUSTOM 99

/*
Enter the type of substrate and version number of the substrate here.
e.g. v12d swatch = SWATCH, 12, 3
*/
#define CONNECTED_BOARD CUSTOM
#define CONNECTED_VERSION_MAJOR 12
#define CONNECTED_VERSION_MINOR 3 // 0 based; i.e. "A" is 0

static const std::vector<uint8_t> version = {HARDWARE_VERSION_MAJOR, HARDWARE_VERSION_MINOR, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH, CONNECTED_BOARD, CONNECTED_VERSION_MAJOR, CONNECTED_VERSION_MINOR};

#define MAX_QUEUE_SIZE 2048 // max number of row/col pairs in a tapout pattern
// #define CPU_CLK_FREQ_OVERIDE 240 // MHz; set in esp-idf config as 80

/*
Overtap protection limits high duty cycle tapping of each actuator.
It does this by calculating a value called "heat" which builds up with large onDurations with insufficient off time to cool down.
The values are currently balanced around an assumed safe continuous duty cycle of 3ms onDuration / 51ms offDuration repeated on a single tapper.
See this sheet for sample calculations: https://docs.google.com/spreadsheets/d/15yVmE13jGJZapsrEjx4v79JR5FFSrgQL-_rgFeacN8Q/edit?usp=sharing
*/
#define OVERTAP_PROTECTION
#ifdef OVERTAP_PROTECTION
  #define ON_DURATION_MULTIPLIER 17
  #define ATTENUATION_CONSTANT 10
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

// values used for charlieplexing
#define ANODE 1 // 1 = HS fet enabled, LS fet disabled on h bridge driver
#define CATHODE 0 // 0 = LS fet enabled, HS fet disabled on h bridge driver

// Map the rows/columns (generalized as channels) to HBDriver outputs and set them as either rows or columns
struct Channel {
  uint8_t parent; // 0 to NUM_DRIVERS
  uint8_t outputPin; // 1 to 10 for the MP6527GF
  uint8_t polarity; // ANODE or CATHODE
};

// tap parameter initial and limit values
static const unsigned long onDur_init = 30; // hundreds of microseconds (tenths of a millisecond)
static const unsigned long onDur_max = 30;
static const unsigned long offDur_init = 1000; // tenths of a ms

// stress test vector
static const std::vector<uint8_t> stressTestVect = {
    1, 1, // message tpye, tapoutID
    3, 12 // v12d on the controller (using the reverse swatch pinout) --> row (anode) is CH2, col (cathode) is CH4
    // 0, 0, // row, col
    // 1, 0,
    // 2, 0,
    // 3, 0,
    // 4, 0,
    // 5, 0,
    // 6, 0,
    // 7, 0,
    // 8, 0,
    // 9, 0,
    // 0, 1,
    // 1, 1,
    // 2, 1,
    // 3, 1,
    // 4, 1,
    // 5, 1,
    // 6, 1,
    // 7, 1,
    // 8, 1,
    // 9, 1,
    // 0, 2,
    // 1, 2,
    // 2, 2,
    // 3, 2,
    // 4, 2,
    // 5, 2,
    // 6, 2,
    // 7, 2,
    // 8, 2,
    // 9, 2
};

// touch pin detection thresholds
static const int caseTouchThreshold = 34600; // needs tuning
static const int fpcTouchThreshold = 34600; // needs tuning

/* -------------SUBSTRATE PINOUTS------------- */

#if CONNECTED_BOARD == R20C20 && (CONNECTED_VERSION_MAJOR == 11 && CONNECTED_VERSION_MINOR == 2)
  static const Channel rows[] = {
    {0, 1, ANODE}, // CH0
    {3, 4, ANODE}, // CH1
    {0, 5, ANODE}, // CH2
    {3, 6, ANODE}, // CH3
    {0, 7, ANODE}, // CH4
    {3, 9, ANODE}, // CH5
    {0, 9, ANODE}, // CH6
    {3, 7, ANODE}, // CH7
    {0, 6, ANODE}, // CH8
    {3, 5, ANODE}, // CH9
    {0, 4, ANODE}, // CH10
    {3, 1, ANODE}, // CH11
    {0, 2, ANODE}, // CH12
    {3, 3, ANODE}, // CH13
    {0, 8, ANODE}, // CH14
    {3, 2, ANODE}, // CH15
    {3, 10, ANODE},// CH16
    {3, 8, ANODE}, // CH17
    {0, 3, ANODE}, // CH18
    {2, 6, ANODE} // CH19
  };

  static const Channel cols[] = {
    {0, 10},// CH20
    {1, 5}, // CH21
    {2, 9}, // CH22
    {1, 7}, // CH23
    {2, 7}, // CH24
    {1, 9}, // CH25
    {2, 5}, // CH26
    {1, 6}, // CH27
    {2, 1}, // CH28
    {1, 4}, // CH29
    {1, 1}, // CH30
    {2, 4}, // CH31
    {2, 2}, // CH32
    {1, 3}, // CH33
    {2, 8}, // CH34
    {1, 10},// CH35
    {2, 10},// CH36
    {1, 8}, // CH37
    {2, 3}, // CH38
    {1, 2}  // CH39
  };
#endif

#if CONNECTED_BOARD == R20C20 && (CONNECTED_VERSION_MAJOR == 12 && CONNECTED_VERSION_MINOR == 0)
  static const Channel rows[] = {
    {3, 5, ANODE}, // CH0
    {3, 7, ANODE}, // CH1
    {0, 8, ANODE}, // CH2
    {3, 1, ANODE}, // CH3
    {3, 2, ANODE}, // CH4
    {0, 2, ANODE}, // CH5
    {0, 1, ANODE}, // CH6
    {3, 8, ANODE}, // CH7
    {3, 9, ANODE}, // CH8
    {3, 6, ANODE}, // CH9
    {0, 5, ANODE}, // CH10
    {0, 7, ANODE}, // CH11
    {3, 4, ANODE}, // CH12
    {3, 3, ANODE}, // CH13
    {3, 10, ANODE}, // CH14
    {0, 9, ANODE}, // CH15
    {0, 6, ANODE},// CH16
    {0, 4, ANODE}, // CH17
    {2, 8, ANODE}, // CH18
    {2, 2, ANODE} // CH19
  };

  static const Channel cols[] = {
    {0, 3},// CH20
    {0, 10}, // CH21
    {2, 1}, // CH22
    {1, 8}, // CH23
    {1, 2}, // CH24
    {1, 1}, // CH25
    {1, 5}, // CH26
    {2, 5}, // CH27
    {2, 3}, // CH28
    {2, 7}, // CH29
    {2, 10}, // CH30
    {2, 4}, // CH31
    {1, 7}, // CH32
    {1, 10}, // CH33
    {1, 3}, // CH34
    {1, 9},// CH35
    {1, 6},// CH36
    {1, 4}, // CH37
    {2, 9}, // CH38
    {2, 6}  // CH39
  };
#endif

#if CONNECTED_BOARD == PALM && (CONNECTED_VERSION_MAJOR == 12 && CONNECTED_VERSION_MINOR == 1)
  static const Channel rows[] = {
    {2, 3, ANODE},
    {1, 8, ANODE},
    {1, 7, ANODE},
    {1, 2, ANODE},
    {1, 5, ANODE},
    {1, 1, ANODE},
    {1, 6, ANODE},
    {2, 5, ANODE},
    {2, 9, ANODE},
    {2, 2, ANODE},
    {0, 8, ANODE},
    {1, 3, ANODE},
    {0, 2, ANODE}
  };

  static const Channel cols[] = {
    {2, 4},
    {2, 10},
    {2, 6},
    {2, 1},
    {1, 9},
    {2, 7},
    {1, 4},
    {0, 10},
    {0, 4},
    {0, 7},
    {0, 3},
    {0, 5}
  };
#endif

#if CONNECTED_BOARD == ONBOARD && (CONNECTED_VERSION_MAJOR == 12 && CONNECTED_VERSION_MINOR == 0)
  static const Channel rows[] = {
    {3, 1, ANODE} // CH3
  };

  static const Channel cols[] = {
    {3, 2}// CH4
  };
#endif

#if CONNECTED_BOARD == CUSTOM && (CONNECTED_VERSION_MAJOR == 12 && CONNECTED_VERSION_MINOR == 0)
  static const Channel rows[] = {
    {3, 5, ANODE}, // CH0
    {3, 7, ANODE}, // CH1
    {3, 7, CATHODE}, // CH1
    {3, 5, CATHODE} // CH0
  };

  static const Channel cols[] = {
    {0, 3},// CH20
    {0, 10} // CH21
  };
#endif

#if CONNECTED_BOARD == CUSTOM && (CONNECTED_VERSION_MAJOR == 11 && CONNECTED_VERSION_MINOR == 2)
  static const Channel cols[] = {
    {0, 1, ANODE}, // CH0
    {3, 4, ANODE}, // CH1
    {0, 5, ANODE}, // CH2
    {3, 6, ANODE}, // CH3
    {0, 7, ANODE} // CH4
  };

  static const Channel rows[] = {
    {3, 9}, // CH5
    {0, 9}, // CH6
    {3, 7}, // CH7
    {0, 6}, // CH8
    {3, 5} // CH9
  };
#endif

#if CONNECTED_BOARD == CUSTOM && (CONNECTED_VERSION_MAJOR == 12 && CONNECTED_VERSION_MINOR == 1)
  static const Channel cols[] = {
    {0, 5, ANODE},
    {0, 2, ANODE},
  };

  static const Channel rows[] = {
    {2, 10},
    {1, 2},
  };
#endif

#if CONNECTED_BOARD == SWATCH && (CONNECTED_VERSION_MAJOR == 11 && CONNECTED_VERSION_MINOR == 2)
  static const Channel rows[] = {
    {3, 3, ANODE}, // CH13
    {3, 2, ANODE}, // CH15
    {3, 1, ANODE}, // CH11
    {3, 8, ANODE}, // CH17
    {3, 7, ANODE}, // CH7
    {2, 6, ANODE}, // CH19
    {3, 9, ANODE}, // CH5
    {1, 5, ANODE}, // CH21
    {3, 6, ANODE}, // CH3
    {1, 7, ANODE}, // CH23
    {1, 2, ANODE},  // CH39
    {2, 7, ANODE}, // CH24
    {0, 6, ANODE}, // CH8
    {0, 1, ANODE}, // CH0
    {2, 9, ANODE}, // CH22
    {0, 5, ANODE}, // CH2
    {0, 10, ANODE},// CH20
    {0, 7, ANODE}, // CH4
    {0, 3, ANODE}, // CH18
    {0, 9, ANODE}, // CH6
    {3, 10, ANODE},// CH16
    {0, 4, ANODE}, // CH10
    {0, 8, ANODE}, // CH14
    {0, 2, ANODE} // CH12
  };

  static const Channel cols[] = {
    {2, 3}, // CH38
    {1, 8}, // CH37
    {2, 10},// CH36
    {1, 10},// CH35
    {2, 8}, // CH34
    {1, 3}, // CH33
    {2, 2}, // CH32
    {2, 4}, // CH31
    {1, 1}, // CH30
    {1, 4}, // CH29
    {2, 1}, // CH28
    {1, 6}, // CH27
    {2, 5}, // CH26
    {1, 9}, // CH25
    {3, 5}, // CH9
    {3, 4}, // CH1
  };
#endif

#if CONNECTED_BOARD == PALM && (CONNECTED_VERSION_MAJOR == 12 && CONNECTED_VERSION_MINOR == 2)
  static const Channel rows[] = {
    {2, 5, ANODE}, // CH24
    {2, 10, ANODE}, // CH18
    {1, 2, ANODE}, // CH07
    {0, 6, ANODE}, // CH08
    {1, 3, ANODE}, // CH05
    {0, 9, ANODE}, // CH06
    {2, 8, ANODE}, // CH25
    {2, 7, ANODE}, // CH26
    {2, 3, ANODE}, // CH20
    {1, 5, ANODE}, // CH19
    {0, 2, ANODE}, // CH16
    {1, 4, ANODE}, // CH09
    {0, 5, ANODE} // CH00

    // {0, 9, ANODE}, // CH06
    // {0, 3, ANODE}, // CH12
    // {1, 8, ANODE}, // CH23
    // {2, 4, ANODE}, // CH22
    // {2, 8, ANODE}, // CH25
    // {2, 5, ANODE}, // CH24
    // {1, 3, ANODE}, // CH05
    // {0, 7, ANODE}, // CH04
    // {0, 4, ANODE}, // CH10
    // {1, 6, ANODE}, // CH11
    // {0, 10, ANODE}, // CH14
    // {1, 1, ANODE}, // CH21
    // {2, 6, ANODE} // CH30
  };

  static const Channel cols[] = {
    {1, 8}, // CH23
    {2, 4}, // CH22
    {1, 1}, // CH21
    {1, 7}, // CH17
    {1, 10}, // CH03
    {0, 8}, // CH01
    {0, 1}, // CH02
    {0, 7}, // CH04
    {2, 6}, // CH30
    {2, 1}, // CH29
    {2, 9}, // CH28
    {2, 2} // CH27

    // {1, 2}, // CH07
    // {0, 6}, // CH08
    // {1, 4}, // CH09
    // {1, 9}, // CH13
    // {2, 2}, // CH27
    // {2, 1}, // CH29
    // {2, 9}, // CH28
    // {2, 7}, // CH26
    // {0, 5}, // CH00
    // {0, 8}, // CH01
    // {0, 1}, // CH02
    // {1, 10}, // CH03
  };
#endif

#if CONNECTED_BOARD == SWATCH && (CONNECTED_VERSION_MAJOR == 12 && CONNECTED_VERSION_MINOR == 2)
  static const Channel rows[] = {
    {2, 7, CATHODE}, // CH26
    {2, 1, CATHODE}, // CH29
    {2, 2, CATHODE}, // CH27
    {0, 8, CATHODE}, // CH01
    {1, 10, CATHODE}, // CH03
    {1, 3, CATHODE}, // CH05
    {1, 2, CATHODE}, // CH07
    {0, 6, CATHODE}, // CH08
    {2, 4, CATHODE}, // CH22
    {2, 3, CATHODE}, // CH20
    {2, 10, CATHODE}, // CH18
    {1, 4, CATHODE}, // CH09
    {1, 4, ANODE}, // CH09
    {2, 10, ANODE}, // CH18
    {2, 3, ANODE}, // CH20
    {2, 4, ANODE}, // CH22
    {0, 6, ANODE}, // CH08
    {1, 2, ANODE}, // CH07
    {1, 3, ANODE}, // CH05
    {1, 10, ANODE}, // CH03
    {0, 8, ANODE}, // CH01
    {2, 2, ANODE}, // CH27
    {2, 1, ANODE}, // CH29
    {2, 7, ANODE} // CH26
  };

  static const Channel cols[] = {
    {0, 4}, // CH10
    {1, 6}, // CH11
    {0, 3}, // CH12
    {1, 9}, // CH13
    {0, 10}, // CH14
    {0, 2}, // CH16
    {1, 7}, // CH17
    {1, 5}, // CH19
    {1, 1}, // CH21
    {1, 8}, // CH23
    {2, 5}, // CH24
    {0, 9}, // CH06
    {0, 7}, // CH04
    {0, 1}, // CH02
    {0, 5}, // CH00
    {2, 9}, // CH28
    {2, 8}, // CH25
    {2, 6} // CH30
  };
#endif

#if CONNECTED_BOARD == CUSTOM && (CONNECTED_VERSION_MAJOR == 12 && CONNECTED_VERSION_MINOR == 3)
  // NOTE: this is the reversed swatch for testing without the tail PCB
  static const Channel rows[] = {
    {0, 5, ANODE}, // CH12
    {3, 1, ANODE}, // CH31
    {1, 10, ANODE}, // CH01
    {0, 3, ANODE}, // CH02
    {0, 8, ANODE}, // CH03
    {1, 4, ANODE}, // CH05
    {1, 6, ANODE}, // CH07
    {1, 9, ANODE}, // CH09
    {0, 7, ANODE}, // CH10
    {0, 6, ANODE}, // CH00
    {1, 1, ANODE}, // CH15
    {2, 2, ANODE}, // CH33
    {3, 10, ANODE}, // CH24
    {3, 8, ANODE}, // CH26
    {2, 7, ANODE}, // CH27
    {3, 7, ANODE}, // CH28
    {3, 5, ANODE}, // CH29
    {2, 8, ANODE}, // CH34
    {2, 9, ANODE}, // CH36
    {2, 6, ANODE}, // CH38
    {2, 10, ANODE}, // CH32
    {0, 1, ANODE} // CH14
    
  };

  static const Channel cols[] = {
    {0, 2}, // CH16
    {3, 3}, // CH17
    {1, 3}, // CH18
    {3, 4}, // CH19
    {3, 6}, // CH20
    {1, 8}, // CH21
    {3, 9}, // CH22
    {1, 2}, // CH23
    {3, 2}, // CH25
    {1, 7}, // CH11
    {0, 4}, // CH08
    {0, 9}, // CH06
    {0, 10}, // CH04
    {2, 4}, // CH35
    {2, 1}, // CH37
    {2, 5}, // CH39
    {2, 3}, // CH30
    {1, 5} // CH13
  };
#endif

#if CONNECTED_BOARD == SWATCH && (CONNECTED_VERSION_MAJOR == 12 && CONNECTED_VERSION_MINOR == 3)
  static const Channel rows[] = {
    {2, 7, ANODE}, // CH27
    {0, 4, ANODE}, // CH08
    {2, 6, ANODE}, // CH38
    {2, 1, ANODE}, // CH37
    {2, 9, ANODE}, // CH36
    {2, 8, ANODE}, // CH34
    {2, 10, ANODE}, // CH32
    {2, 3, ANODE}, // CH30
    {3, 5, ANODE}, // CH29
    {2, 5, ANODE}, // CH39
    {3, 10, ANODE}, // CH24
    {0, 9, ANODE}, // CH06
    {1, 1, ANODE}, // CH15
    {1, 5, ANODE}, // CH13
    {0, 5, ANODE}, // CH12
    {1, 7, ANODE}, // CH11
    {0, 7, ANODE}, // CH10
    {1, 4, ANODE}, // CH05
    {0, 8, ANODE}, // CH03
    {1, 10, ANODE}, // CH01
    {1, 6, ANODE}, // CH07
    {3, 2, ANODE} // CH25
  };

  static const Channel cols[] = {
    {1, 2}, // CH23
    {3, 9}, // CH22
    {1, 8}, // CH21
    {3, 6}, // CH20
    {3, 4}, // CH19
    {1, 3}, // CH18
    {3, 3}, // CH17
    {0, 2}, // CH16
    {0, 1}, // CH14
    {3, 7}, // CH28
    {3, 1}, // CH31
    {2, 2}, // CH33
    {2, 4}, // CH35
    {0, 10}, // CH04
    {0, 3}, // CH02
    {0, 6}, // CH00
    {1, 9}, // CH09
    {3, 8} // CH26
  };
#endif

#if CONNECTED_BOARD == PALM && (CONNECTED_VERSION_MAJOR == 12 && CONNECTED_VERSION_MINOR == 3)
  static const Channel rows[] = {
    {3, 7, ANODE}, // CH28
    {3, 9, ANODE}, // CH22
    {0, 5, ANODE}, // CH12
    {1, 5, ANODE}, // CH13
    {0, 7, ANODE}, // CH10
    {1, 7, ANODE}, // CH11
    {3, 5, ANODE}, // CH29
    {2, 3, ANODE}, // CH30
    {3, 10, ANODE}, // CH24
    {1, 2, ANODE}, // CH23
    {3, 6, ANODE}, // CH20
    {0, 1, ANODE}, // CH14
    {1, 4, ANODE} // CH05
  };

  static const Channel cols[] = {
    {2, 7}, // CH27
    {3, 8}, // CH26
    {3, 2}, // CH25
    {1, 8}, // CH21
    {0, 4}, // CH08
    {0, 9}, // CH06
    {1, 6}, // CH07
    {1, 9}, // CH09
    {2, 8}, // CH34
    {2, 2}, // CH33
    {2, 10}, // CH32
    {3, 1}, // CH31
  };
#endif
  
static const uint8_t numRows = sizeof(rows)/sizeof(rows[0]);
static const uint8_t numCols = sizeof(cols)/sizeof(cols[0]);

#endif // CONFIGURATION_H