// Define the UUIDs for the service and characteristic
const SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
const OTA_CHARACTERISTIC_UUID = "c8659211-af91-4ad3-a995-a58d6fd26145";

// Define message types
const SentMessageType = {
    TAP_OUT: 1,
    GET_DEVICE_INFO: 2,
};

const ReceivedMessageType = {
    STATUS_UPDATE: 1,
    TAPOUT_COMPLETE: 2,
    DEVICE_INFO: 3,
    INVALID_MSG_TYPE: 51,
    INCORRECT_MSG_SIZE: 52,
    PARAM_OOB: 53,
    OTA_TIMEOUT: 61,
};  

let bleDevice = null;
let bleServer = null;
let bleCharacteristic = null;
let otaCharacteristic = null;

// Function to connect to the BLE server
async function connect() {
  try {
    bleDevice = await navigator.bluetooth.requestDevice({
      filters: [{ services: [SERVICE_UUID] }],
    });

    bleServer = await bleDevice.gatt.connect();
    const service = await bleServer.getPrimaryService(SERVICE_UUID);
    bleCharacteristic = await service.getCharacteristic(CHARACTERISTIC_UUID);
    otaCharacteristic = await service.getCharacteristic(OTA_CHARACTERISTIC_UUID);

    // Enable notifications
    await bleCharacteristic.startNotifications();
    bleCharacteristic.addEventListener("characteristicvaluechanged", handleNotifications);
    toggleConnectButton(true);
    bleDevice.addEventListener('gattserverdisconnected', () => {
        console.log('Device disconnected');
        appendToConsole(`Bluetooth disconnected`);
        toggleConnectButton(false);
      });
  } catch (error) {
    console.error("Error connecting to device:", error);
    appendToConsole("Error connecting to device", error);
  }
}

async function uploadOTA() {
    appendToConsole("Preparing OTA firmware upload...");
    try {
      if (!otaCharacteristic) {
        console.error("OTA characteristic not available.");
        appendToConsole("OTA characteristic not available.");
        return;
      }
  
      // Enable notifications on the OTA characteristic
      await otaCharacteristic.startNotifications();
  
      // Read the firmware.bin file
      const response = await fetch('firmware.bin');
      const updateData = await response.arrayBuffer();
  
      if (updateData.byteLength === 0) {
        console.error("Firmware file is empty.");
        appendToConsole("Firmware file is empty.");
        return;
      }
  
      const chunkSize = 512; // Size of each chunk in bytes
      let offset = 0;
  
      while (offset < updateData.byteLength) {
        if (offset == 0) {
            appendToConsole("Please press the user input button to initiate upload.");
        }
        const nextOffset = Math.min(offset + chunkSize, updateData.byteLength);
        const chunk = updateData.slice(offset, nextOffset);

        let timeoutId;
        let resolvePromise;
        let rejectPromise;
  
        function handleAck(event) {
            clearTimeout(timeoutId);
            const value = event.target.value;
            const ackArray = [1, 2, 3, 4, 5]; // Expected acknowledgment array
            const receivedArray = [];
            
            for (let i = 0; i < 5; i++) {
                receivedArray.push(value.getUint8(i));
            }
            
            if (JSON.stringify(ackArray) === JSON.stringify(receivedArray)) {
                otaCharacteristic.removeEventListener('characteristicvaluechanged', handleAck);
                resolvePromise();
            } else {
                // TODO bug: this seems to fire at the end of a successful upload
                rejectPromise(new Error("Failed to write chunk to ESP32."));
            }
        }
  
        // Create a promise to wait for acknowledgment
        const ackReceived = new Promise((resolve, reject) => {
            resolvePromise = resolve;
            rejectPromise = reject;
            
            timeoutId = setTimeout(() => { 
                otaCharacteristic.removeEventListener('characteristicvaluechanged', handleAck);
                reject(new Error("Operation timed out."));
            }, 10000); // 10s timeout
        
            otaCharacteristic.addEventListener('characteristicvaluechanged', handleAck);
        });
  
        await otaCharacteristic.writeValue(chunk);
        await ackReceived;
          
  
        // Calculate and log the progress
        const progress = (nextOffset / updateData.byteLength) * 100;
        console.log(`Upload progress: ${progress.toFixed(1)}%`);
        appendToConsole(`Upload progress: ${progress.toFixed(1)}%`);
  
        offset = nextOffset;
      }
  
      // TODO never reach here even after a successful upload
      console.log("Code uploaded to controller.");
      appendToConsole("Code uploaded to controller.");
    } catch (error) {
      console.error("Error uploading code to device:", error);
      appendToConsole("Error uploading code to device:", error);
    }
}

function disconnect() {
    if (bleDevice && bleDevice.gatt.connected) {
        bleDevice.gatt.disconnect();
    }
    toggleConnectButton(false);
    toggleIMUStreamButton(false)
}

function toggleConnectButton(isConnected) {
    const connectBtn = document.getElementById('connectBtn');
    if (isConnected) {
        connectBtn.textContent = 'Disconnect';
        connectBtn.classList.add('connected');
        connectBtn.setAttribute('data-connected', 'true');
    } else {
        connectBtn.textContent = 'Connect';
        connectBtn.classList.remove('connected');
        connectBtn.setAttribute('data-connected', 'false');
    }
}

document.getElementById('connectBtn').addEventListener('click', () => {
    const isConnected = document.getElementById('connectBtn').getAttribute('data-connected') === 'true';
    if (isConnected) {
        disconnect();
    } else {
        connect().catch(error => {
            console.error("Error connecting to device:", error);
            appendToConsole("Error connecting to device:", error);
            toggleConnectButton(false);
        });
    }
});

function bytesToInt32(byte1, byte2, byte3, byte4) {
    return ((byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4) >> 0;
}


function handleNotifications(event) {
    const characteristic = event.target;
    const value = characteristic.value;
    if (!value) return;

    const receivedType = value.getUint8(0);
    const messageTypeString = getMessageTypeString(receivedType);

    let additionalData = [];
    let additionalDataString = "";

    // Loop through the DataView starting at index 1 to get additional data
    for (let i = 1; i < value.byteLength; i++) {
        additionalData.push(value.getUint8(i));
    }

    switch (receivedType) {
        case ReceivedMessageType.STATUS_UPDATE:
            // note: when comparing to the readme, note that the message type byte has already been taken off here
            let statusFrequency = additionalData[0];
            let systemTime = (additionalData[1] << 24) | (additionalData[2] << 16) | (additionalData[3] << 8) | additionalData[4];
            let batteryPercent = additionalData[5];
            additionalDataString = `Status Frequency: ${statusFrequency}, System Time: ${systemTime}, Battery: ${batteryPercent}%`;
        
            
            let tapHandlerStatus = additionalData.slice(6, 15);
            let tapOutID = tapHandlerStatus[0];
            let halfFullFalling = tapHandlerStatus[1];
            let tapQHeadroom = (tapHandlerStatus[2] << 8) | tapHandlerStatus[3];
            let firstRejectedIndex = (tapHandlerStatus[4] << 8) | tapHandlerStatus[5];
            let warningCode = tapHandlerStatus[6];
            let warningValue = (tapHandlerStatus[7] << 8) | tapHandlerStatus[8];
            additionalDataString += `Tap Out ID: ${tapOutID}, Half Full Falling: ${halfFullFalling}, Tap Queue Headroom: ${tapQHeadroom}, Last Rejected Index: ${firstRejectedIndex}, Warning Code: ${warningCode}`;

            let imuData = additionalData.slice(15, 39);
            let accelX = (imuData[0] << 24 | imuData[1] << 16 | imuData[2] | imuData[3]) / 1000;
            let accelY = (imuData[4] << 24 | imuData[5] << 16 | imuData[6] << 8 | imuData[7]) / 1000;
            let accelZ = (imuData[8] << 24 | imuData[9] << 16 | imuData[10] << 8 | imuData[11]) / 1000;
            let gyroX = (imuData[12] << 24 | imuData[13] << 16 | imuData[14] << 8 | imuData[15]) / 1000;
            let gyroY = (imuData[16] << 24 | imuData[17] << 16 | imuData[18] << 8 | imuData[19]) / 1000;
            let gyroZ = (imuData[20] << 24 | imuData[21] << 16 | imuData[22] << 8 | imuData[23]) / 1000;
            additionalDataString += `Accel X: ${accelX} Y: ${accelY} Z: ${accelZ}, Gyro X: ${gyroX} Y: ${gyroY} Z: ${gyroZ}`;

            let extraData = additionalData.slice(39)
            let overtappedRowIndex = extraData[0];
            let overtappedColIndex = extraData[1];
            additionalDataString += `Overtapped Row: ${overtappedRowIndex} Col: ${overtappedColIndex}`;

            break;
        case ReceivedMessageType.TAPOUT_COMPLETE:
            additionalDataString = `Message ID: ${additionalData[0]}`;
            break;
        case ReceivedMessageType.INVALID_MSG_TYPE:
            additionalDataString = `Invalid Message Type: ${additionalData[0]}`;
            break;
        case ReceivedMessageType.INCORRECT_MSG_SIZE:
            additionalDataString = `Incorrect Message Size: ${additionalData[0]}`;
            break;
        case ReceivedMessageType.PARAM_OOB:
            additionalDataString = `Parameter Out of Bounds at Position: ${additionalData[0]}`;
            break;
        case ReceivedMessageType.OTA_TIMEOUT:
            // No additional data
            break;
        case ReceivedMessageType.DEVICE_INFO:
            let serialNumberBytes = [];
            for (let i = 0; i < 6; i++) {
                let hexValue = additionalData[i].toString(16).padStart(2, '0').toUpperCase();
                serialNumberBytes.push(hexValue);
            }
            additionalDataString = "Serial Number: " + serialNumberBytes.join(":");
            additionalDataString += `, Controller Version: ${additionalData[6]}.${additionalData[7]}, `;
            additionalDataString += `Firmware Version: ${additionalData[8]}.${additionalData[9]}.${additionalData[10]}, `;
            // let board = ``;
            // switch (additionalDataString[11]) {
            //     case 0:
            //         board = `Swatch`;
            //         break;
            //     case 1:
            //         board = `PALM`;
            //         break;
            //     default:
            //         board = `Unknown: ${additionalDataString[11]}`;
            //         break;
            // }
            // additionalDataString += `Connected Board: ${board}, `;
            additionalDataString += `Connected Board: ${additionalData[11]}, `; // TODO not sure what's wrong with the above code but fixing that later
            additionalDataString += `Version: ${additionalData[12]}.${additionalData[13]}`;
            break;
        default:
            additionalDataString = `Unknown Message Type`;
            break;
    }

    console.log(`Received message: ${messageTypeString}, ${additionalDataString}`);
    appendToConsole(`Received message: ${messageTypeString}, ${additionalDataString}`);
}

async function sendMessage(buffer, messageType) {
    try {
        await bleCharacteristic.writeValue(buffer);
        const sentData = Array.from(new Uint8Array(buffer));
        const sentDataString = sentData.map(byte => byte.toString(16).padStart(2, '0')).join(' ');
        console.log(`Sent message: ${messageType}, Data: [${sentDataString}]`);
        appendToConsole(`Sent message: ${messageType}, Data: [${sentDataString}]`);
    } catch (error) {
        console.error("Error writing to device:", error);
        appendToConsole("Error writing to device:", error);
    }
}

function getDeviceInfo() {
    const buffer = new ArrayBuffer(1);
    const dataView = new DataView(buffer);
    dataView.setUint8(0, SentMessageType.GET_DEVICE_INFO);
    sendMessage(buffer, "GET_DEVICE_INFO");
}

function appendToConsole(message, additionalMessage = null) {
    const consoleLog = document.getElementById('consoleLog');
    const newLog = document.createElement('div');
    if (additionalMessage) {
        newLog.textContent = `${message} - ${additionalMessage}`;
    } else {
        newLog.textContent = message;
    }
    consoleLog.appendChild(newLog);
  
    // Auto-scroll to the bottom
    consoleLog.scrollTop = consoleLog.scrollHeight;
}

  
function getMessageTypeString(value) {
    const key = Object.keys(ReceivedMessageType).find(key => ReceivedMessageType[key] === value);
    return key || "UNKNOWN_MESSAGE_TYPE";
}

// Function to clear the console
function clearConsole() {
    const consoleLog = document.getElementById('consoleLog');
    consoleLog.innerHTML = '';
}


document.getElementById("tapOutBtn").addEventListener("click", () => {
    // Calculate the total size of the sent message
    let len = 2; // 2 bytes for message type and message ID
    for (let i = 0; i < 3; i++) {
        const qty = parseInt(document.getElementById(`qty${i}`).value);
        len += qty * 2 + 4; // 2 bytes for each additional 'row' and 'col', 4 bytes for 'onDurMSB', onDurLSB, 'offDurMSB', and 'offDurLSB' of the first element
    }

    const arr = new Uint8Array(len);

    const id = (parseInt(document.getElementById(`tapoutID`).value));
    arr[0] = SentMessageType.TAP_OUT;
    arr[1] = id;

    let offset = 2; // Store data after message type and message ID

    for (let i = 0; i <= 2; i++) {
        const row = (parseInt(document.getElementById(`rowIndex${i}`).value));
        const col = parseInt(document.getElementById(`colIndex${i}`).value);
        const onDur = parseInt(document.getElementById(`onDur${i}`).value);
        const offDur = parseInt(document.getElementById(`offDur${i}`).value);

        const onDurMSB = (onDur >> 8) & 0xFF;
        const onDurLSB = onDur & 0xFF;
        const offDurMSB = (offDur >> 8) & 0xFF;
        const offDurLSB = offDur & 0xFF;

        // Write the first element with the settings
        arr[offset++] = row | (1 << 7); // to indicate we're sending settings, the first bit of the row byte needs to be 1
        arr[offset++] = col;
        arr[offset++] = onDurMSB;
        arr[offset++] = onDurLSB;
        arr[offset++] = offDurMSB;
        arr[offset++] = offDurLSB;

        // Write the additional elements with just row and col, which will keep the last sent settings
        const qty = parseInt(document.getElementById(`qty${i}`).value);
        for (let j = 1; j < qty; j++) {
            arr[offset++] = row;
            arr[offset++] = col;
        }
    }
    sendMessage(arr, "TAP_OUT");
});

document.getElementById("clearConsoleBtn").addEventListener("click", clearConsole);
document.getElementById("uploadCodeBtn").addEventListener("click", uploadOTA);
document.getElementById("getDeviceInfoBtn").addEventListener("click", getDeviceInfo);