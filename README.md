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

### Data Frame

```
HEADER        LENGTH   DATA        FOOTER
F4 F3 F2 F1   XX XX    [payload]   F8 F7 F6 F5
```

## Basic Target Information

The LD2410 sensor transmits basic target information in a structured data frame format. The frame layout is as follows:

**Frame Breakdown:**

- **HEADER (`F4 F3 F2 F1`)**: Identifies the start of the data frame.
- **LENGTH (`XX XX`)**: Indicates the length of the data in the frame.
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
- **FOOTER (`F8 F7 F6 F5`)**: Identifies the end of the data frame.

## Engineering Mode Target Information

When the LD2410 sensor operates in Engineering Mode, it transmits additional detailed data within its frames, enhancing the basic target information with energy values for each distance gate. The structure of the frame is as follows:

**Frame Breakdown:**

- **HEADER (`F4 F3 F2 F1`)**: Identifies the start of the data frame.
- **LENGTH (`XX XX`)**: Indicates the length of the data in the frame.
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

- **FOOTER (`F8 F7 F6 F5`)**: Identifies the end of the data frame.

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

## Engineering Mode Data

Adds additional information to standard frame:

- Moving target energy per gate
- Stationary target energy per gate
- Light sensor value (0-255)
- OUT pin state

# Technical Notes

## lastSensorDataUpdate and lastEngineeringDataUpdate

The `BasicData` and `EngineeringData` are updated by parsing data from the LD2410's serial connection. Occasionally, the sensor may stop transmitting new data. To monitor data freshness, the `lastSensorDataUpdate` and `lastEngineeringDataUpdate` variables store the timestamp (in milliseconds) of the last update. Users can verify data currency using the `isBasicDataCurrent()` and `isEngineeringDataCurrent()` functions. The timeout duration for declaring data as outdated can be defined by the user by passing a specific timeout value to the function. If not specified, a default timeout will be applied.

The timestamp is generated using the `millis()` function, which counts milliseconds since the MCU started running the current program. However, `millis()` will overflow and reset to zero after approximately 50 days. This overflow will render the timestamp invalid.

##
