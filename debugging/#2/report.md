# Debugging Report: Firmware Version Query Doesen't Work

## Problem

1. The sensor appears to be echoing back the sent commands instead of responding with proper acknowledgments
2. According to the protocol documentation, we should receive:
   - For config mode: `FD FC FB FA 08 00 FF 01 00 00 01 00 40 00 04 03 02 01`
   - For version query: `FD FC FB FA 0C 00 A0 01 00 00 00 01 02 01 16 24 06 22 04 03 02 01`
   - For exit config: `FD FC FB FA 04 00 FE 01 00 00 04 03 02 01`

## Overview

This report documents attempts to read the firmware version from the HLK-LD2410B presence detection sensor using the serial protocol as specified in the manufacturer's documentation.

## Test Setup

- ESP32-S3 development board
- HLK-LD2410B sensor
- Wiring:
  - Sensor TX → ESP32 GPIO17 (RX)
  - Sensor RX → ESP32 GPIO18 (TX)
- Serial monitor baudrate: 115200
- Sensor communication baudrate: 256000

## Implementation

The test implements a broad debugging approach, attempting to read the firmware version while logging all serial communication. Here's the test sketch:

### Test Sketch

```cpp
#include <Arduino.h>
#include "LD2410.h"

// Create an instance of the LD2410 class
LD2410 ld2410;

void setup()
{
    // Initialize the serial interface for the monitor
    Serial.begin(115200);
    while (!Serial); // Wait until Serial is ready

    // Optionally enable debug messages from the library
    ld2410.useDebug(Serial);

    // Initialize the UART interface
    ld2410.beginUART(17, 18, Serial2);
    delay(1500);

    uint8_t major, minor;
    uint16_t bugfix, build;

    if (ld2410.getFirmwareVersion(major, minor, bugfix, build))
    {
        Serial.printf("Firmware Version: v%d.%d.%d.%d\n", major, minor, bugfix, build);
    }
    else
    {
        Serial.println("Failed to get firmware version");
    }
}

void loop()
{
}
```

### Version Query Implementation

```cpp
bool LD2410::getFirmwareVersion(uint8_t &major, uint8_t &minor, uint16_t &bugfix, uint16_t &build)
{
    if (!_uart.initialized)
    {
        return false;
    }

    uint8_t buffer[64] = {0};
    uint8_t bufferIndex = 0;

    debugPrintln("\n=== Starting Version Query with Endless Listen ===");

    // 1. Enter Config Mode
    const uint8_t configCmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    debugPrintHex("TX CONFIG: ", configCmd, sizeof(configCmd));
    _uart.serial->write(configCmd, sizeof(configCmd));
    _uart.serial->flush();

    debugPrintln("Listening for CONFIG response:");
    unsigned long lastByteTime = millis();
    bufferIndex = 0;

    // Listen until no data for 2 seconds
    while (true)
    {
        if (_uart.serial->available())
        {
            buffer[bufferIndex++] = _uart.serial->read();
            lastByteTime = millis();
            if (_debug_serial)
            {
                _debug_serial->printf("%02X ", buffer[bufferIndex - 1]);
            }
        }

        if (millis() - lastByteTime > 2000)
        {
            debugPrintln("\nNo more data for 2 seconds...");
            break;
        }
        yield();
    }

    // 2. Request Version
    delay(100);
    const uint8_t versionCmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA0, 0x00, 0x04, 0x03, 0x02, 0x01};
    debugPrintHex("\nTX VERSION: ", versionCmd, sizeof(versionCmd));
    _uart.serial->write(versionCmd, sizeof(versionCmd));
    _uart.serial->flush();

    // Similar listening loop for version response...
    // Code truncated for brevity, follows same pattern as above

    return false;
}
```

## Output

```
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x1 (POWERON),boot:0xb (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3808,len:0x4bc
load:0x403c9700,len:0xbd8
load:0x403cc700,len:0x2a0c
entry 0x403c98d0
Debug mode enabled
[LD2410] UART initialized successfully
=== Starting Version Query with Endless Listen ===
TX CONFIG: FD FC FB FA 04 00 FF 00 01 00 04 03 02 01
Listening for CONFIG response:
00 FD FC FB FA 04 00 FF 00 01 00 04 03 02 01
No more data for 2 seconds...
TX VERSION: FD FC FB FA 02 00 A0 00 04 03 02 01
Listening for VERSION response:
FD FC FB FA 02 00 A0 00 04 03 02 01
No more data for 2 seconds...
TX EXIT: FD FC FB FA 02 00 FE 00 04 03 02 01
Listening for EXIT response:
FD FC FB FA 02 00 FE 00 04 03 02 01
No more data for 2 seconds...
=== End of Version Query ===
Failed to get firmware version
```
