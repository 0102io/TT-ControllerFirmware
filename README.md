# Initial Commit Notes
This repository was copied from `https://github.com/0102io/testing/tree/main/tt_driver_fw_pio` in release v0.4.2.

# Communication Protocol for Client Device and TappyTap Controller

This section outlines the communication protocol for a client device to connect to a the TappyTap Controller using Bluetooth 5 (LE). see 'script.js' in the examples folder for an implementation of this protocol.

## Connecting to the Controller

The Controller advertises itself as a BLE server with the service UUID `4fafc201-1fb5-459e-8fcc-c5c9c331914b`. The client device should initiate a scan for BLE devices and look for the one advertising this service UUID. Once found, the client device should establish a connection to the BLE server (the Controller).

## Sending Messages to the Controller

To send a message from the client device to the Controller, the client device should write to the BLE characteristic with the UUID `beb5483e-36e1-4688-b7f5-ea07361b26a8`.

### Sent Message Types and Structure

- `TAP_OUT`: Byte code `1`. The message should follow this structure:

    - Byte 0: Message type (`1`)
    - Byte 1: idenfication number that is retruned by the controller along with the confirmation when the TAP_OUT sequence is completed. Sending followup TAP_OUT messages will have their contents added to the queue of taps, and the new ID will overwrite the last. Sending an ID of `0` will cancel an active pattern.
    - Bytes 2 to n: `row` and `column` index pairs for the pattern, as well as tap parameters (optional). Each pair consists of the `row` index followed by the `column` index. To specify tap parameters, set the most significant bit of a `row` index to `1`. Then, following the `row` and `column` index pair, the next 3 bytes should be `onDuration` (length of time to apply voltage to a coil; tenths of a millisecond) and 2 bytes for a 16 bit `offDuration` (length of time before the next tap executes; tenths of a millisecond; MSB first). These parameters are used for every tap following until new parameters are read. If `row`/`column` pairs are sent with no preceeding parameters, the controller will use the parameters that were last written to it (or the init parameters if it has not received any since last restarting). `row`/`column` pairs are tapped out FIFO.

        TAP_OUT example 1:
        | Byte Value | Description |
        |---|---|
        | 0x01 | Message type (TAP_OUT)|
        | 0x02 | TapoutID |
        | 0x03 | Row 3, use previous/init settings for this tap |
        | 0x04 | Col 4 |
        | 0x05 | Row 5, use previous/init settings for this tap |
        | 0x06 | Col 6 |

        TAP_OUT example 2:
        | Byte Value | Description |
        |---|---|
        | 0x01 | Message type (TAP_OUT)|
        | 0x02 | TapoutID |
        | 0x83 | Row 3, use new settings for this and following taps |
        | 0x04 | Col 4 |
        | 0x0A | On Duration = 1ms |
        | 0x00 | Off Duration MSB |
        | 0x64 | Off Duration LSB, read with MSB this is 10ms |
        | 0x05 | Row 5, use previous settings for this tap |
        | 0x06 | Col 6 |
        | 0x87 | Row 7, use new settings for this and following taps |
        | 0x08 | Col 8 |
        | 0x14 | On Duration = 2ms |
        | 0x03 | Off Duration MSB |
        | 0xE8 | Off Duration LSB, read with MSB this is 100ms |
      

- `GET_DEVICE_INFO`: Byte code `2`. Controller returns with `DEVICE_INFO` message.

- `CANCEL_AND_TAP`: Byte code `3`. Clears the queue of taps and then loads the new taps through the same function called by TAP_OUT. This can be used to clear the queue and start a new pattern in 1 BLE pulse.

- `UPDATE_STATUS_FREQUENCY`: Byte code `4`. A second byte with the frequency in Hz must be sent. If the requested frequency is above the limit set in the firmware, the maxiumum frequency will be used instead. If 0 is sent, the frequency will be set to 1 Hz.

## Receiving Messages from the Controller

The client device should also monitor the same BLE characteristic (UUID `beb5483e-36e1-4688-b7f5-ea07361b26a8`) for notifications to receive messages from the Controller.

### Received Message Types and Structure

- `STATUS_UPDATE`: Byte code `1`. This message is sent to the client device at a user settable frequency. The returned bytes are as follows; unless otherwise noted, multi-byte values are all MSB first


    | Byte | Description |
    |---|---|
    | 0 | Message code (STATUS_UPDATE) |
    | 1 | Status update frequency (Hz) |
    | 2-5 | System time (ms) |
    | 6 | Battery percent |
    | 7 | Last received tapout id |
    | 8 | Flag indicating transition from the queue being more than half full, to less than half full |
    | 9-10 | Headroom / free space in the queue |
    | 11-12 | First rejected index (due to a full queue; if queue is not full, this is the last tapper's index + 1, not including the message type and tapout ID bytes) |
    | 13 | Warning code from last message (e.g. PARAM_OOB) |
    | 14-15 | Value associated with warning code (e.g. index of the OOB parameter) |
    | 16-19 | IMU accel X (multiplied by 1000) |
    | 20-23 | IMU accel Y (multiplied by 1000) |
    | 24-27 | IMU accel Z (multiplied by 1000) |
    | 28-31 | IMU gyro X (multiplied by 1000) |
    | 32-35 | IMU gyro Y (multiplied by 1000) |
    | 36-39 | IMU gyro Z (multiplied by 1000) |
    | 40 | Overtapped row index * |
    | 41 | Overtapped column index * |
    | 42 | Battery detected (0 for false, 1 for true) |
    <!-- note: when reading from the status update function, remember that an additional byte is added (the message code) -->
    
    *Overtap warning: when a tapper is actuated at a higher duty cycle than the firmware-set soft limit, the firmware attenuates the on duration. The controller tracks a value called "heat" for each tapper, which is increased by long on durations and short intervals between taps (to that specific row/col). If enough heat is generated (calculated) when a tapper is supposed to turn on, the on duration is shortened proportionally to the heat. After the shortened tap, the controller pauses for the remainder of the expected on duration so that the pattern's cadence is unchanged (it will just appear as a softer tap). The row/col indices sent in the status message are the last indices to have generated the overtap warning. After the message is sent, the indicies are re-set to "EMPTY_TAP" (defined in firmware as 99); if these values are received, no tapper has experienced an overtap event since the last status message.

- `DEVICE_INFO`: Byte code `3`. This message contains configuration information that usually only needs to be passed to the client device once. The returned bytes are as follows:

    | Byte | Description |
    |---|---|
    | 0 | Message code (DEVICE_INFO) |
    | 1 | Serial Number (MAC Address) first octet |
    | 2 | Serial Number (MAC Address) second octet |
    | 3 | Serial Number (MAC Address) third octet |
    | 4 | Serial Number (MAC Address) fourth octet |
    | 5 | Serial Number (MAC Address) fifth octet |
    | 6 | Serial Number (MAC Address) sixth octet |
    | 7 | Controller hardware version (major) |
    | 8 | Controller hardware version (minor) |
    | 9 | Firmware version (major) |
    | 10 | Firmware version (minor) |
    | 11 | Firmware version (patch) |
    | 12 | Connected board type (0 for swatch, 1 for palm) |
    | 13 | Connected board version (major) |
    | 14 | Connected board version (minor) |

- `INVALID_MSG_TYPE`: Byte code `51`. This message contains 2 bytes; the first is the byte code and the second is the invalid message type that was received

- `PARAM_OOB`: Byte code `53`. This message contains 2 bytes; the first is the byte code and the second is the position of the message parameter that is out of bounds. If multiple parameters are OOB, the position of the last one will be sent. Row and column indicies that are OOB are replaced with "empty tap" indicies, which will make the controller pause for the onDuration to keep the pattern cadence intact. If the warning is given for an onDuration parameter (i.e. above the limit), the controller replaces the sent parameter with the maximum onDuration value defined in the firmware. Note that the returned index does not count the message type or tapout id bytes. needs edit

- `OTA_TIMEOUT`: Byte code `61`. This message is sent to the client device when the controller receives an OTA upload request but the user doesn't press the 'interact button' before the timeout period.


# Compiling the Firmware

This project uses re-compiled arduino-esp32 core libraries using the [esp32-arduino-lib-builder project](https://github.com/espressif/esp32-arduino-lib-builder), and it is structured for compilation with PlatformIO. I'm not an expert with PlatformIO, ESP-IDF, the library builder, etc. so I'd recommend following other guides to get those all set up first.

# Uploading Firmware
bluetooth OTA needs documentation

## Steps for Building New Core Libraries

1. Edit `esp32-arduino-lib-builder/configs/defconfig.esp32s3` (settings for this project are located in the documentation folder)

2. In a terminal window, navigate to the `esp32-arduino-lib-builder` folder and enter:
    ```bash
    rm -rf build # remove the existing build files if you've already compiled them
    rm -rf components/esp-rainmaker # I had to do this to get my build on Oct 2 to work, may not be necessary now
    ./build.sh -t esp32s3 -I idf-release/v5.1
    ./tool/copy-to-arduino # if using arduino, or copy the files listed in the script to the relevant location for platformIO - I believe it is /Users/[your_username]/.platformio/packages/framework-arduinoespressif32
    ```
Note regarding the rainmaker module: I last built core files on Oct. 2, 2023, and encountered an error similar to [this GitHub issue](https://github.com/espressif/esp32-arduino-lib-builder/issues/138). For me, pulling the latest update (even fully re-installing the repository) didn't resolve the issue, so I added a specific line to the `update-components.sh` file and removed the existing rainmaker component with the command above.
```bash
#
# CLONE/UPDATE ESP-RAINMAKER
#
echo "Updating ESP-RainMaker..."
if [ ! -d "$AR_COMPS/esp-rainmaker" ]; then
    git clone $RMAKER_REPO_URL "$AR_COMPS/esp-rainmaker" && \
    git -C "$AR_COMPS/esp-rainmaker" checkout 0414a8530ec1ac8714269302503c71c238b68836 # <------ this line
    git -C "$AR_COMPS/esp-rainmaker" submodule update --init --recursive
else
    git -C "$AR_COMPS/esp-rainmaker" fetch && \
    git -C "$AR_COMPS/esp-rainmaker" pull --ff-only && \
    git -C "$AR_COMPS/esp-rainmaker" submodule update --init --recursive
fi
if [ $? -ne 0 ]; then exit 1; fi
```
