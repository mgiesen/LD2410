# LD2410

A streamlined library for the LD2410 sensor, providing a high-level interface for UART communication and output observation, with minimal user code required. Built with the Arduino framework, making it compatible with common MCUs like Espressif chips (e.g., ESP8266, ESP32) and all Arduino boards.

Additionally, this repository includes a complete web dashboard for the LD2410 library and sensor, enabling data visualization and sensor configuration.

> [!WARNING]  
> This library is currently under development and is not yet ready for production use. The API may change frequently until the stable release.

## Features

- Optional UART communication with configurable baud rates
- Optional Output pin observation using an interrupt callback
- Optional Debugging support via serial output
- Ring buffer implementation for efficient serial data handling

## Usage

### Initialization

```cpp
LD2410 sensor;
```

### Simple Output Pin Observation

```cpp
void outputCallback(bool presenceDetected)
{
    if (presenceDetected)
    {
        Serial.println("Presence detected");
    }
    else
    {
        Serial.println("No presence detected");
    }
}

setup()
{
    Serial.begin(9600);
    sensor.beginOutputObservation(outputPin, outputCallback);
}
```

### UART Communication example

```cpp
setup()
{
    Serial.begin(9600);
    sensor.beginUART(ld2410txPin, ld2410rxPin, Serial2, 256000);
}

loop()
{
    sensor.readSensorData();

    LD2410::BasicData basicData = ld2410.getBasicData();
    Serial.println("Detection Distance: " + String(basicData.detectionDistance) + " cm");
}
```

### List of public functions

```cpp
sensor.beginUART(rx_pin, tx_pin, serial, baud);                     // Initialize UART communication with the sensor
sensor.useDebug(serialPort);                                        // Enable debugging output to specified serial port
sensor.readSensorData(maxBytesPerLoop);                             // Process incoming UART data
sensor.getBasicData();                                              // Get latest basic sensor data
sensor.getEngineeringData();                                        // Get latest engineering sensor data
sensor.prettyPrintData(output);                                     // Pretty print current sensor data to output stream
sensor.getLastErrorString();                                        // Get string description of last error
sensor.setMaxValues(movingGate, stationaryGate, timeout);           // Set maximum detection gates for moving and stationary targets
sensor.setGateSensitivityThreshold(gate, moving, stationary);       // Set sensitivity threshold for specific detection gate
sensor.enableEngineeringMode();                                     // Enable engineering mode for additional sensor data output
sensor.disableEngineeringMode();                                    // Disable engineering mode
sensor.setBaudRate(baudRate);                                       // Change sensor UART baud rate
sensor.setDistanceResolution(use020mResolution);                    // Set distance resolution (0.20m or 0.75m per gate)
sensor.factoryReset();                                              // Reset sensor to factory defaults
sensor.restart();                                                   // Restart the sensor
sensor.readConfiguration();                                         // Read current sensor configuration
sensor.beginOutputObservation(pin, callback, pinMode);              // Start observing sensor's digital output pin with callback
```

## Sensor

### Pinout

![ld2410_pinout.png](/readme/ld2410_pinout.png)

### Wiring

![ld2410_connection.svg](/readme/ld2410_connection.svg)

### Documentation

- [Manual EN](docu/Manual.pdf)
- [Serial Communication EN](docu/Serial%20Communication.pdf)

### Default Configuration

- UART: 256000 baud, 8N1 (1 stop bit, no parity)
- Max Gates: 8 (Default max distance)
- Default Resolution: 0.75m per gate
- Timeout Duration: 5s

### Command Frame

```
HEADER        LENGTH   DATA        FOOTER
FD FC FB FA   XX XX    [payload]   04 03 02 01
```

### Data Frame

```
HEADER        LENGTH   DATA        FOOTER
F4 F3 F2 F1   XX XX    [payload]   F8 F7 F6 F5
```

#### Data - Basic Target Information

The LD2410 sensor transmits basic target information in a structured data frame format.

- **DATA**:
  - **Target State (1 byte)**: Indicates the detected target status with possible values:
    - `0x00`: No Target
    - `0x01`: Moving Target
    - `0x02`: Stationary Target
    - `0x03`: Both Moving and Stationary Targets
  - **Movement Target Distance (2 bytes)**: Distance to the detected moving target in centimeters.
  - **Movement Target Energy (1 byte)**: Indicates the energy or intensity of the moving target.
  - **Stationary Target Distance (2 bytes)**: Distance to the detected stationary target in centimeters.
  - **Stationary Target Energy (1 byte)**: Energy or intensity of the stationary target.
  - **Detection Distance (2 bytes)**: The maximum detection range for the targets.

#### Data - Engineering Mode Target Information

When the LD2410 sensor operates in Engineering Mode, it transmits additional detailed data within its frames, enhancing the basic target information with energy values for each distance gate.

**Frame Breakdown:**

- **DATA**:
  - **Full Basic Target Information (same as in Basic Mode)**:
  - **Movement Gate Energy Values**: Energy levels for each distance gate (N values depending on the number of configured gates), representing the intensity of moving targets at each gate.
  - **Stationary Gate Energy Values**: Energy levels for each distance gate (N values) for stationary targets.
  - **Light Sensor Value (1 byte)**: Reports a value in the range of 0–255 representing detected light intensity (if applicable).
  - **OUT Pin State (1 byte)**: Indicates the state of the OUT pin (0 for no target, 1 for target detected).

### Protocol Rules

1. All commands require Enable Config first
2. All commands must be followed by End Config
3. All multi-byte values are little-endian

## Technical Notes

### UART Communication Strategy

- **Frame Types**: The LD2410 sensor communicates using two types of data frames:

  - **Sensor Data Frames**: Regularly sent at ~50 ms intervals in non-config mode.
  - **Command Acknowledgment Frames (ACK)**: Handled synchronously within the command execution logic.

- **Blocking Behavior**:

  - Command execution is blocking, potentially halting the main loop for up to two seconds due to a timeout mechanism.
  - Ensure time-critical tasks can accommodate this delay while sending commands to the sensor

- **Data Handling**:
  - `readSensorData()`: Reads data from the UART buffer, processes valid sensor data frames, and ignores ACK frames.
  - Commands are executed inline, preventing `readSensorData()` from processing command-related frames.

### Buffer Handling and Frame Loss

At the high default baud rate of 256000, the UART buffer can overwrite older data if `readSensorData()` is not called frequently enough to process incoming frames. This can result in the loss of short-lived events or intermediate frames.

The ring buffer architecture mitigates this by ensuring that `readSensorData()` always processes the latest available data. The buffer is sized to hold approximately two engineering data frames, ensuring that complete frames are captured during normal operation.

### Timestamp Management (`lastSensorDataUpdate` and `lastEngineeringDataUpdate`)

- **Purpose**: Track the freshness of data updates.
  - `lastSensorDataUpdate`: Timestamp of the last basic data update.
  - `lastEngineeringDataUpdate`: Timestamp of the last engineering data update.
- **Timeout Management**:

  - Use `isBasicDataCurrent()` and `isEngineeringDataCurrent()` to verify if the data is up-to-date.
  - Functions allow specifying a custom timeout duration; otherwise, a default value applies.

- **Limitations**:
  - The `millis()` function, used for timestamps, overflows after ~50 days, resetting to zero. This overflow can invalidate timestamp comparisons, but the library prioritizes critical functionality and ignores updates older than the overflow duration. As a result, the issue is deprioritized in typical usage scenarios.
