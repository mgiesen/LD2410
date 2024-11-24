class RadarInterface
{
    constructor()
    {
        this.ws = null;
        this.reconnectTimeout = null;
        this.chart = null;
        this.engineeringMode = false;
        this.initializeChart();
        this.initializeWebSocket();
        this.createGateElements();
        this.setupEventListeners();
        this.setupAdvancedControls();
        this.requestDeviceInfo();
    }

    initializeChart()
    {
        const ctx = document.getElementById('gateChart').getContext('2d');
        this.chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: ['Gate 1', 'Gate 2', 'Gate 3', 'Gate 4', 'Gate 5', 'Gate 6', 'Gate 7', 'Gate 8'],
                datasets: [
                    {
                        label: 'Motion',
                        data: new Array(8).fill(0),
                        borderColor: 'rgb(75, 192, 192)',
                        tension: 0.1
                    },
                    {
                        label: 'Stationary',
                        data: new Array(8).fill(0),
                        borderColor: 'rgb(255, 99, 132)',
                        tension: 0.1
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    y: {
                        beginAtZero: true,
                        max: 100,
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        }
                    },
                    x: {
                        grid: {
                            color: 'rgba(255, 255, 255, 0.1)'
                        }
                    }
                },
                plugins: {
                    legend: {
                        labels: {
                            color: 'rgb(255, 255, 255)'
                        }
                    }
                }
            }
        });
    }

    initializeWebSocket()
    {
        this.setConnectionStatus('connecting');

        // Native Implementation will host the frontend on port 80
        const frontend_hosted_externally = window.location.port != "";
        if (frontend_hosted_externally)
        {
            // Port 8080 is used for the mockup server
            this.ws = new WebSocket('ws://localhost:8080');
        }
        else
        {
            this.ws = new WebSocket(`ws://${window.location.hostname}/ws`);
        }

        this.ws.onopen = () =>
        {
            this.setConnectionStatus('connected');
            this.requestConfiguration();
            this.requestDeviceInfo();
        };

        this.ws.onclose = () =>
        {
            this.setConnectionStatus('disconnected');
            this.scheduleReconnect();
        };

        this.ws.onerror = () =>
        {
            this.setConnectionStatus('disconnected');
        };

        this.ws.onmessage = (event) => this.handleMessage(event);
    }

    scheduleReconnect()
    {
        clearTimeout(this.reconnectTimeout);
        this.reconnectTimeout = setTimeout(() =>
        {
            this.initializeWebSocket();
        }, 2000);
    }

    setConnectionStatus(status)
    {
        const statusElement = document.getElementById('connectionStatus');
        statusElement.className = 'connection-status ' + status;
        switch (status)
        {
            case 'connected':
                statusElement.textContent = 'Connected';
                break;
            case 'disconnected':
                statusElement.textContent = 'Disconnected';
                break;
            case 'connecting':
                statusElement.textContent = 'Connecting...';
                break;
        }
    }

    createGateElements()
    {
        const gatesGrid = document.getElementById('gatesGrid');
        gatesGrid.innerHTML = '';

        for (let i = 0; i < 8; i++)
        {
            const gateCard = document.createElement('div');
            gateCard.className = 'gate-card';
            gateCard.id = `gate_${i}_card`;
            gateCard.innerHTML = `
                <h3>Gate ${i + 1}</h3>
                <div class="slider-container">
                    <div class="slider-label">
                        <span>Motion Sensitivity</span>
                        <span id="moveValue${i}">-</span>
                    </div>
                    <input type="range" min="0" max="100" value="50" 
                           class="slider" 
                           data-gate="${i}" 
                           data-type="motion" 
                           id="motionSlider${i}">
                </div>
                <div class="slider-container">
                    <div class="slider-label">
                        <span>Stationary Sensitivity</span>
                        <span id="statValue${i}">-</span>
                    </div>
                    <input type="range" min="0" max="100" value="50" 
                           class="slider" 
                           data-gate="${i}" 
                           data-type="stationary" 
                           id="stationarySlider${i}">
                </div>
            `;
            gatesGrid.appendChild(gateCard);
        }
    }

    requestConfiguration()
    {
        this.ws.send(JSON.stringify({
            command: 'getConfig'
        }));
    }

    requestDeviceInfo()
    {
        this.ws.send(JSON.stringify({
            command: 'getDeviceInfo'
        }));
    }

    updateChartData(movingData, stationaryData)
    {
        this.chart.data.datasets[0].data = movingData;
        this.chart.data.datasets[1].data = stationaryData;
        this.chart.update();
    }

    handleMessage(event)
    {
        try
        {
            const data = JSON.parse(event.data);

            switch (data.type)
            {
                case 'config':
                    this.handleConfigData(data);
                    break;
                case 'status':
                    this.handleStatusData(data);
                    break;
                case 'deviceInfo':
                    this.handleDeviceInfo(data);
                    break;
                default:
                    console.warn('Unknown message type:', data.type);
            }
        } catch (e)
        {
            console.error('Error parsing message:', e);
        }
    }

    handleConfigData(data)
    {
        // Update global settings
        document.getElementById('maxMovingGate').value = data.maxMovingGate;
        document.getElementById('maxMovingGateValue').textContent = data.maxMovingGate;

        document.getElementById('maxStationaryGate').value = data.maxStationaryGate;
        document.getElementById('maxStationaryGateValue').textContent = data.maxStationaryGate;

        document.getElementById('timeout').value = data.timeout;
        document.getElementById('timeoutValue').textContent = data.timeout;

        // Update gate configurations
        data.gates.forEach((gate, index) =>
        {
            document.getElementById(`motionSlider${index}`).value = gate.motionSensitivity;
            document.getElementById(`moveValue${index}`).textContent = gate.motionSensitivity;

            document.getElementById(`stationarySlider${index}`).value = gate.stationarySensitivity;
            document.getElementById(`statValue${index}`).textContent = gate.stationarySensitivity;
        });

        // Update distance resolution radio buttons
        if (data.distanceResolution !== undefined)
        {
            const resolution = data.distanceResolution;
            document.getElementById('res075').checked = resolution === 0.75;
            document.getElementById('res02').checked = resolution === 0.2;
        }

        // Update engineering mode toggle
        if (data.engineeringMode !== undefined)
        {
            document.getElementById('engineeringMode').checked = data.engineeringMode;
            this.engineeringMode = data.engineeringMode;
        }
    }

    handleStatusData(data)
    {
        // Update status indicators
        const statusMapping = {
            0: 'No Detection',
            1: 'Motion Detected',
            2: 'Stationary Target',
            3: 'Motion & Stationary'
        };

        document.getElementById('detectionStatus').textContent = statusMapping[data.detectionState];
        document.getElementById('movingDistance').textContent = `${data.movingDistance} cm`;
        document.getElementById('stationaryDistance').textContent = `${data.stationaryDistance} cm`;
        document.getElementById('totalEnergy').textContent = data.totalEnergy;

        // Update chart
        this.updateChartData(data.movingEnergy, data.stationaryEnergy);
    }

    handleDeviceInfo(data)
    {
        document.getElementById('macAddress').textContent = data.macAddress;
        document.getElementById('firmwareVersion').textContent = data.firmwareVersion;
    }

    setupAdvancedControls()
    {
        // Distance Resolution Controls
        document.querySelectorAll('input[name="resolution"]').forEach(input =>
        {
            input.addEventListener('change', (e) =>
            {
                const use020m = e.target.value === '0.2';
                this.ws.send(JSON.stringify({
                    command: 'setDistanceResolution',
                    value: use020m
                }));
            });
        });

        // Engineering Mode Toggle
        const engineeringModeToggle = document.getElementById('engineeringMode');
        engineeringModeToggle.addEventListener('change', (e) =>
        {
            const enabled = e.target.checked;
            this.ws.send(JSON.stringify({
                command: enabled ? 'enableEngineeringMode' : 'disableEngineeringMode'
            }));
            this.engineeringMode = enabled;
        });

        // Device Control Buttons
        document.getElementById('restartDevice').addEventListener('click', () =>
        {
            this.showConfirmationDialog(
                'Restart Device',
                'Are you sure you want to restart the device? The connection will be temporarily lost.',
                () =>
                {
                    this.ws.send(JSON.stringify({ command: 'restart' }));
                }
            );
        });

        document.getElementById('factoryReset').addEventListener('click', () =>
        {
            this.showConfirmationDialog(
                'Factory Reset',
                'Warning: This will reset all settings to factory defaults. This action cannot be undone.',
                () =>
                {
                    this.ws.send(JSON.stringify({ command: 'factoryReset' }));
                }
            );
        });
    }

    showConfirmationDialog(title, message, onConfirm)
    {
        const dialog = document.createElement('div');
        dialog.className = 'dialog-overlay';
        dialog.innerHTML = `
            <div class="dialog-content">
                <h3>${title}</h3>
                <p>${message}</p>
                <div class="dialog-buttons">
                    <button class="button" onclick="this.parentElement.parentElement.parentElement.remove()">Cancel</button>
                    <button class="button danger" id="confirmButton">Confirm</button>
                </div>
            </div>
        `;

        document.body.appendChild(dialog);

        document.getElementById('confirmButton').addEventListener('click', () =>
        {
            onConfirm();
            dialog.remove();
        });
    }

    setupEventListeners()
    {
        // Debounce function
        const debounce = (func, wait) =>
        {
            let timeout;
            return function executedFunction(...args)
            {
                const later = () =>
                {
                    clearTimeout(timeout);
                    func(...args);
                };
                clearTimeout(timeout);
                timeout = setTimeout(later, wait);
            };
        };

        // Gate sensitivity sliders
        document.querySelectorAll('.slider[data-gate]').forEach(slider =>
        {
            const updateSensitivity = debounce((gate, type, value) =>
            {
                this.ws.send(JSON.stringify({
                    command: 'setSensitivity',
                    gate: parseInt(gate),
                    type: type,
                    value: parseInt(value)
                }));
            }, 200);

            slider.addEventListener('input', (e) =>
            {
                const value = e.target.value;
                const gate = e.target.dataset.gate;
                const type = e.target.dataset.type;
                document.getElementById(`${type === 'motion' ? 'move' : 'stat'}Value${gate}`).textContent = value;
                updateSensitivity(gate, type, value);
            });
        });

        // Global settings sliders
        const updateGlobalSetting = debounce((setting, value) =>
        {
            this.ws.send(JSON.stringify({
                command: 'setGlobal',
                setting: setting,
                value: parseInt(value)
            }));
        }, 200);

        ['maxMovingGate', 'maxStationaryGate', 'timeout'].forEach(settingId =>
        {
            const slider = document.getElementById(settingId);
            slider.addEventListener('input', (e) =>
            {
                document.getElementById(`${settingId}Value`).textContent = e.target.value;
                updateGlobalSetting(settingId, e.target.value);
            });
        });
    }
}

// Initialize the interface when the page loads
window.addEventListener('load', () =>
{
    window.radarInterface = new RadarInterface();
});