# LD2410

A lightweight library for the LD2410 sensor, enabling easy UART communication and efficient monitoring of sensor output with minimal overhead.

> [!WARNING]  
> This library is currently under development and is not yet ready for production use. The API may change frequently until the stable release.

## Features

- Optional UART communication with configurable baud rates
- Optional Output pin observation using an interrupt callback
- Optional Debugging support via serial output
- Ring buffer implementation for efficient serial data handling

## Usage

**Initialization**

```cpp
LD2410 sensor;
```

**Debugging**

```cpp
Serial.begin(115200);

sensor.useDebug(Serial);
```

**UART Communication**

```cpp
setup()
{
    sensor.beginUART(ld2410txPin, ld2410rxPin, Serial2, 256000);
}

loop()
{
    sensor.processUART();
}
```

**Output Observation**

```cpp
setup()
{
    void outputCallback(bool presenceDetected) {}

    sensor.beginOutputObservation(pin, outputCallback);
}
```

# Sensor

## Pinout

![ld2410_pinout.png](/readme/ld2410_pinout.png)

## Documentation

- [Manual EN](docu/Manual.pdf)
- [Serial Communication EN](docu/Serial%20Communication.pdf)

## Default Configuration

- UART: 256000 baud, 8N1 (1 stop bit, no parity)
- Max Gates: 8 (Default max distance)
- Default Resolution: 0.75m per gate
- Timeout Duration: 5s

## Command Frame

```
HEADER        LENGTH   DATA        FOOTER
FD FC FB FA   XX XX    [payload]   04 03 02 01
```

## Data Frame

```
HEADER        LENGTH   DATA        FOOTER
F4 F3 F2 F1   XX XX    [payload]   F8 F7 F6 F5
```

### Data - Basic Target Information

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

### Data - Engineering Mode Target Information

When the LD2410 sensor operates in Engineering Mode, it transmits additional detailed data within its frames, enhancing the basic target information with energy values for each distance gate.

**Frame Breakdown:**

- **DATA**:
  - **Basic Target Information (same as in Basic Mode)**:
    - **Target State (1 byte)**: Indicates the detected target status.
    - **Movement Target Distance (2 bytes)**: Distance to the moving target in centimeters.
    - **Movement Target Energy (1 byte)**: Energy/intensity value for the moving target.
    - **Stationary Target Distance (2 bytes)**: Distance to the stationary target in centimeters.
    - **Stationary Target Energy (1 byte)**: Energy/intensity value for the stationary target.
    - **Detection Distance (2 bytes)**: Overall maximum detection range.
  - **Movement Gate Energy Values**: Energy levels for each distance gate (N values depending on the number of configured gates), representing the intensity of moving targets at each gate.
  - **Stationary Gate Energy Values**: Energy levels for each distance gate (N values) for stationary targets.
  - **Light Sensor Value (1 byte)**: Reports a value in the range of 0–255 representing detected light intensity (if applicable).
  - **OUT Pin State (1 byte)**: Indicates the state of the OUT pin (0 for no target, 1 for target detected).

## Default Sensitivity Settings

| Gate | Motion | Stationary |
| ---- | ------ | ---------- |
| 0,1  | 50     | N/A        |
| 2    | 40     | 40         |
| 3    | 30     | 40         |
| 4    | 20     | 30         |
| 5    | 15     | 30         |
| 6-8  | 15     | 20         |

## Protocol Rules

1. All commands require Enable Config (0x00FF) first
1. All commands must be followed by End Config (0x00FE)
1. All multi-byte values are little-endian

# Technical Notes

## Important Notes on Serial Buffer Management

### Background

The LD2410 sensor transmits data frames at regular intervals (approximately every 50ms):

- Standard Data Frame: ~19 bytes
- Engineering Mode Frame: ~40 bytes
- ACK Frames: 8-12 bytes

### Potential Issues

Due to the high default data rate at 256000 baud, there's a risk of losing data if the serial buffer fills up. This can happen if:

- The main loop runs too slowly
- Other tasks block the processing
- The hardware serial buffer is too small

### Recommendations

1. **Reduce Sensor Baud Rate**  
    If your application doesn't require high update rates, you can lower the sensor's baud rate to reduce data throughput. This gives you more time to process the data and reduces the risk of buffer overflow.
   Note: The sensor needs to be restarted for the new baud rate to take effect.

   ```cpp
   radar.setBaudRate(115200);
   ```

1. **Increase Hardware Serial Buffer (Recommended)**  
   Increase buffer size before starting UART connection.

   ```cpp
   Serial2.setRxBufferSize(512);
   ```

1. **Process Data Frequently**  
   Ensure `processUART()` is called regularly in your main loop. Long delays or blocking operations can cause buffer overflow and data loss:

1. **Increase Processing Batch Size**  
   By default, `processUART()` processes up to 32 bytes per call to balance responsiveness with other tasks. For applications with less time-critical operations, you can increase this limit. Note: Higher values mean longer processing time per call but more efficient buffer handling. Lower values provide better responsiveness for other tasks but require more frequent calls.
   ```cpp
   sensor.processUART(128);
   ```

**Calculate Maximum Safe Loop Delay**

You can calculate the maximum safe delay between `processUART()` calls for your configuration:

```cpp
const uint16_t hwBufferSize = 512;    // Hardware buffer (bytes)
const uint8_t batchSize = 32;         // Bytes processed per processUART() call
const uint32_t baudRate = 256000;     // Sensor baud rate
const float safetyMargin = 0.8f;      // 20% safety margin for timing variations

// Calculations
const float bytesPerSecond = baudRate / 10.0f;  // Each byte needs 10 bits in UART communication (8 data + start + stop)
const float timeToFillBuffer = (hwBufferSize / bytesPerSecond) * 1000.0f;  // Time in ms until HW buffer is full
const float bytesPerLoop = batchSize;  // How many bytes we process per call

// Maximum delay calculation
const float maxLoopDelay = (timeToFillBuffer * safetyMargin * bytesPerLoop) / hwBufferSize;

// Example with default values:
// - HW Buffer: 512 bytes
// - Batch Size: 32 bytes
// - Baud Rate: 256000
// = ~1.6ms maximum delay between processUART() calls
```

## lastSensorDataUpdate and lastEngineeringDataUpdate

The `BasicData` and `EngineeringData` are updated by parsing data from the LD2410's serial connection. Occasionally, the sensor may stop transmitting new data. To monitor data freshness, the `lastSensorDataUpdate` and `lastEngineeringDataUpdate` variables store the timestamp (in milliseconds) of the last update. Users can verify data currency using the `isBasicDataCurrent()` and `isEngineeringDataCurrent()` functions. The timeout duration for declaring data as outdated can be defined by the user by passing a specific timeout value to the function. If not specified, a default timeout will be applied.

The timestamp is generated using the `millis()` function, which counts milliseconds since the MCU started running the current program. However, `millis()` will overflow and reset to zero after approximately 50 days. This overflow will render the timestamp invalid.

##
