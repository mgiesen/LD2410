# LD2410

A lightweight library for the LD2410 sensor, enabling easy UART communication and efficient monitoring of sensor output with minimal overhead.

> [!WARNING]  
> This library is currently under development and not yet ready for production use

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
    sensor.beginUART(rxPin, txPin, Serial2, 256000);
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

## Target States

- 0x00: No Target
- 0x01: Moving Target
- 0x02: Stationary Target
- 0x03: Moving & Stationary

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
