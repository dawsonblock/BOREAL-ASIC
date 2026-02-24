/**
 * Boreal Emotiv Bridge v2.5 - SECURE WebSocket bridge
 */
const WebSocket = require('ws');
const { SerialPort } = require('serialport');

const CONFIG = {
    emotiv: {
        url: process.env.EMOTIV_URL || 'wss://localhost:6868',
        clientId: process.env.EMOTIV_CLIENT_ID,
        clientSecret: process.env.EMOTIV_CLIENT_SECRET,
        rejectUnauthorized: process.env.NODE_ENV === 'production'
    },
    fpga: {
        port: process.env.FPGA_PORT || '/dev/ttyUSB0',
        baudRate: 115200
    }
};

if (!CONFIG.emotiv.clientId || !CONFIG.emotiv.clientSecret) {
    console.error('ERROR: Set EMOTIV_CLIENT_ID and EMOTIV_CLIENT_SECRET');
    process.exit(1);
}

class BorealBridge {
    constructor() {
        this.ws = null;
        this.serial = null;
        this.authToken = null;
    }

    async connect() {
        console.log('Connecting to FPGA...');
        this.serial = new SerialPort({ path: CONFIG.fpga.port, baudRate: CONFIG.fpga.baudRate });

        console.log('Connecting to EMOTIV...');
        this.ws = new WebSocket(CONFIG.emotiv.url, {
            rejectUnauthorized: CONFIG.emotiv.rejectUnauthorized
        });

        this.ws.on('open', () => {
            console.log('WebSocket connected');
            this.ws.send(JSON.stringify({
                id: 1, jsonrpc: '2.0', method: 'authorize',
                params: { clientId: CONFIG.emotiv.clientId, clientSecret: CONFIG.emotiv.clientSecret }
            }));
        });

        this.ws.on('message', (data) => {
            const response = JSON.parse(data);
            if (response.id === 1 && response.result) {
                this.authToken = response.result.cortexToken;
                console.log('Authenticated');
                this.subscribe();
            }
            if (response.eeg) {
                this.processEEG(response.eeg);
            }
        });
    }

    subscribe() {
        this.ws.send(JSON.stringify({
            id: 2, jsonrpc: '2.0', method: 'subscribe',
            params: { cortexToken: this.authToken, streams: ['eeg'], session: 'active' }
        }));
    }

    processEEG(eegData) {
        const channels = eegData.slice(1, 15);
        const buffer = Buffer.alloc(30);
        buffer[0] = 0xAA;
        channels.forEach((val, i) => {
            const clamped = Math.max(-32768, Math.min(32767, val * 10));
            buffer.writeInt16BE(clamped, 1 + (i * 2));
        });
        buffer[29] = 0x55;
        if (this.serial.isOpen) {
            this.serial.write(buffer);
        }
    }
}

const bridge = new BorealBridge();
bridge.connect();
