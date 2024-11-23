const WebSocket = require('ws');
const http = require('http');

class RadarBackendSimulator
{
    constructor()
    {
        // Initialize configuration with default values
        this.config = {
            maxMovingGate: 6,
            maxStationaryGate: 6,
            timeout: 5,
            distanceResolution: 0.75,
            engineeringMode: false,
            gates: Array(8).fill(null).map(() => ({
                motionSensitivity: 50,
                stationarySensitivity: 50
            }))
        };

        // Device information
        this.deviceInfo = {
            macAddress: "AA:BB:CC:DD:EE:FF",
            firmwareVersion: "1.0.0"
        };

        // Initialize status values
        this.status = {
            detectionState: 0,
            movingDistance: 0,
            stationaryDistance: 0,
            totalEnergy: 0,
            movingEnergy: new Array(8).fill(0),
            stationaryEnergy: new Array(8).fill(0)
        };

        this.setupServer();
        this.startSimulation();
    }

    setupServer()
    {
        const server = http.createServer();
        this.wss = new WebSocket.Server({ server });  // Store WSS instance in class

        this.wss.on('connection', (ws) =>
        {
            console.log('New client connected');

            // Send initial configuration and device info
            ws.send(JSON.stringify({
                type: 'config',
                ...this.config
            }));

            ws.send(JSON.stringify({
                type: 'deviceInfo',
                ...this.deviceInfo
            }));

            ws.on('message', (message) =>
            {
                try
                {
                    const data = JSON.parse(message);
                    this.handleMessage(ws, data);
                } catch (e)
                {
                    console.error('Error parsing message:', e);
                }
            });

            ws.on('close', () =>
            {
                console.log('Client disconnected');
            });
        });

        const port = 8080;
        server.listen(port, () =>
        {
            console.log(`Server is running on port ${port}`);
        });
    }

    handleMessage(ws, data)
    {
        console.log('Received message:', data);

        switch (data.command)
        {
            case 'getConfig':
                ws.send(JSON.stringify({
                    type: 'config',
                    ...this.config
                }));
                break;

            case 'getDeviceInfo':
                ws.send(JSON.stringify({
                    type: 'deviceInfo',
                    ...this.deviceInfo
                }));
                break;

            case 'setSensitivity':
                this.config.gates[data.gate][data.type === 'motion' ? 'motionSensitivity' : 'stationarySensitivity'] = data.value;
                break;

            case 'setGlobal':
                this.config[data.setting] = data.value;
                break;

            case 'setDistanceResolution':
                this.config.distanceResolution = data.value ? 0.2 : 0.75;
                break;

            case 'enableEngineeringMode':
                this.config.engineeringMode = true;
                break;

            case 'disableEngineeringMode':
                this.config.engineeringMode = false;
                break;

            case 'restart':
                this.simulateRestart(ws);
                break;

            case 'factoryReset':
                this.simulateFactoryReset(ws);
                break;
        }
    }

    simulateRestart(ws)
    {
        ws.send(JSON.stringify({ type: 'status', message: 'Restarting device...' }));
        setTimeout(() =>
        {
            ws.close();
        }, 1000);
    }

    simulateFactoryReset(ws)
    {
        // Reset all configurations to default
        this.config = {
            maxMovingGate: 6,
            maxStationaryGate: 6,
            timeout: 5,
            distanceResolution: 0.75,
            engineeringMode: false,
            gates: Array(8).fill(null).map(() => ({
                motionSensitivity: 50,
                stationarySensitivity: 50
            }))
        };
        ws.send(JSON.stringify({ type: 'config', ...this.config }));
    }

    startSimulation()
    {
        setInterval(() =>
        {
            this.updateSimulatedValues();
            this.broadcastStatus();
        }, 500);
    }

    updateSimulatedValues()
    {
        // Simulate detection states
        this.status.detectionState = Math.floor(Math.random() * 4);

        // Simulate distances with some realistic behavior
        if (this.status.detectionState > 0)
        {
            this.status.movingDistance = Math.floor(Math.random() * 600) + 100; // 100-700cm
            this.status.stationaryDistance = Math.floor(Math.random() * 600) + 100;
            this.status.totalEnergy = Math.floor(Math.random() * 100);

            // Generate realistic-looking energy values for each gate
            this.status.movingEnergy = this.generateEnergyValues(this.status.movingDistance);
            this.status.stationaryEnergy = this.generateEnergyValues(this.status.stationaryDistance);
        } else
        {
            // No detection
            this.status.movingDistance = 0;
            this.status.stationaryDistance = 0;
            this.status.totalEnergy = 0;
            this.status.movingEnergy = new Array(8).fill(0);
            this.status.stationaryEnergy = new Array(8).fill(0);
        }
    }

    generateEnergyValues(distance)
    {
        // Convert distance to gate index (assuming each gate represents ~1 meter)
        const peakGate = Math.floor(distance / 100);

        // Generate a bell curve around the peak gate
        return Array(8).fill(0).map((_, i) =>
        {
            const distanceFromPeak = Math.abs(i - peakGate);
            const baseEnergy = Math.max(0, 100 - (distanceFromPeak * 30));
            // Add some random variation
            return Math.floor(Math.max(0, Math.min(100, baseEnergy + (Math.random() * 20 - 10))));
        });
    }

    broadcastStatus()
    {
        if (this.wss.clients)
        {
            this.wss.clients.forEach(client =>
            {
                if (client.readyState === WebSocket.OPEN)
                {
                    client.send(JSON.stringify({
                        type: 'status',
                        ...this.status
                    }));
                }
            });
        }
    }
}

// Start the backend simulator
new RadarBackendSimulator();