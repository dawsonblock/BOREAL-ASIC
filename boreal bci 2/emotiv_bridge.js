/**
 * Boreal BCI Bridge: EMOTIV Cortex v2 to FPGA
 * Requires: 'ws' and 'serialport' npm packages
 * 
 * Implements the standard Cortex API flow:
 * 1. Request Access (Approve in EMOTIV Launcher)
 * 2. Authorize (Get Token)
 * 3. Query Headsets
 * 4. Create Session
 * 5. Subscribe to 'eeg' stream
 * 6. Relay to FPGA via UART
 */
const WebSocket = require('ws');
const { SerialPort } = require('serialport');

// --- 1. CONFIGURATION ---
const EMOTIV_URL = 'wss://localhost:6868';
const FPGA_PORT = '/dev/tty.usbserial-1410'; // Update to your FPGA's COM/Serial port
const BAUDRATE = 115200;

// You must generate these via emtiv.com > Cortex Apps
const CLIENT_ID = process.env.EMOTIV_CLIENT_ID || 'YOUR_CLIENT_ID_HERE';
const CLIENT_SECRET = process.env.EMOTIV_CLIENT_SECRET || 'YOUR_CLIENT_SECRET_HERE';

// --- 2. INITIALIZATION ---

let authToken = "";
let sessionId = "";
let headsetId = "";

const port = new SerialPort({ path: FPGA_PORT, baudRate: BAUDRATE }, function (err) {
  if (err) {
    console.error('Error opening serial port: ', err.message);
    console.log('Ensure the FPGA is connected and the port is correct. Continuing with Cortex Connection anyway (for testing)...');
  }
});

const socket = new WebSocket(EMOTIV_URL, { rejectUnauthorized: false });

// Helper function to send JSON-RPC requests
function sendRequest(method, params, id) {
    const req = {
        "jsonrpc": "2.0",
        "method": method,
        "params": params,
        "id": id
    };
    socket.send(JSON.stringify(req));
}

// --- 3. STATE MACHINE HANDLING ---

socket.on('open', () => {
    console.log("Connected to EMOTIV Cortex API");
    
    // Step 1: Request Access (This will trigger a prompt in the Emotiv Launcher)
    console.log("Requesting Access...");
    sendRequest("requestAccess", {
        "clientId": CLIENT_ID,
        "clientSecret": CLIENT_SECRET
    }, 1);
});

socket.on('message', (data) => {
    const response = JSON.parse(data);
    
    // Check for errors
    if(response.error) {
        console.error("Cortex Error: ", response.error.message);
        return;
    }

    // Handle Responses by ID
    switch(response.id) {
        case 1: // requestAccess response
            if(response.result && response.result.accessGranted) {
                console.log("Access Granted. Authorizing...");
                sendRequest("authorize", {
                    "clientId": CLIENT_ID,
                    "clientSecret": CLIENT_SECRET
                }, 2);
            } else {
                console.log("Access NOT granted. Please open Emotiv Launcher and approve the application.");
            }
            break;
            
        case 2: // authorize response
            authToken = response.result.cortexToken;
            console.log("Authorized. Querying Headsets...");
            sendRequest("queryHeadsets", {}, 3);
            break;

        case 3: // queryHeadsets response
            if(response.result && response.result.length > 0) {
                headsetId = response.result[0].id; // Take the first connected headset
                console.log(`Found Headset: ${headsetId}. Creating Session...`);
                sendRequest("createSession", {
                    "cortexToken": authToken,
                    "headset": headsetId,
                    "status": "active"
                }, 4);
            } else {
                console.log("No headsets found. Please connect an EPOC X / Insight via Dongle or BTLE.");
            }
            break;

        case 4: // createSession response
            sessionId = response.result.id;
            console.log(`Session Created: ${sessionId}. Subscribing to EEG Stream...`);
            sendRequest("subscribe", {
                "cortexToken": authToken,
                "session": sessionId,
                "streams": ["eeg"]
            }, 5);
            break;

        case 5: // subscribe response
            console.log("Successfully Subscribed to EEG. Boreal Bridge Active. Waiting for neural spikes...");
            break;
    }

    // --- 4. RELAY EEG DATA TO FPGA ---
    // Emotiv streams data continuously using 'sid' (Stream ID) messages rather than specific JSON-RPC response IDs
    if (response.sid && response.eeg) {
        
        // Cortex API 'eeg' array format:
        // [COUNTER, INTERPOLATED, AF3, F7, F3, FC5, T7, P7, O1, O2, P8, T8, FC6, F4, F8, AF4, RAW_CQ, MARKER_HARDWARE]
        // We only want the 14 EEG channels (indices 2 through 15)
        const eegData = response.eeg.slice(2, 16); 
        
        // Pack into 16-bit signed integers for Boreal Core
        // Format: [Sync0] [Ch0_H] [Ch0_L] [Ch1_H] [Ch1_L] ... [Ch13_H] [Ch13_L] [Sync1]
        const buffer = Buffer.alloc(30); 
        
        buffer[0] = 0xAA; // Top Header Sync Byte
        
        eegData.forEach((val, i) => {
            // Cortex data is in microvolts (floating point).
            // We scale it and clamp it to fit into our FPGA's Signed 16-bit (Q1.15) architecture.
            // Adjust the scaling factor (* 10) depending on the needed dynamic range.
            const clampedVal = Math.max(-32768, Math.min(32767, Math.round(val * 10)));
            buffer.writeInt16BE(clampedVal, 1 + (i * 2));
        });
        
        buffer[29] = 0x55; // Bottom Footer Sync Byte
        
        // Write the 30-byte raw neural packet to the UART
        if(port.isOpen) {
             port.write(buffer); 
        }
    }
});

socket.on('error', (err) => {
    console.error("WebSocket Error: ", err);
});

socket.on('close', () => {
    console.log("WebSocket connection closed.");
});
