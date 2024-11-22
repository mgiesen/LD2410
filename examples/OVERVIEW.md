# basicExample

## Features

- Reads MAC-Address
- Reads firmware version
- Enables Engineering Mode
- Reads and print sensor data

## Console Output

```
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x1 (POWERON),boot:0x2b (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3808,len:0x4bc
load:0x403c9700,len:0xbd8
load:0x403cc700,len:0x2a0c
entry 0x403c98d0
Debug mode enabled
[LD2410 DEBUGGER] UART initialized successfully
Sensor MAC Address: 08:05:04:03:02:01
Firmware Version: V2.05.1606
[LD2410 DEBUGGER] Engineering mode enabled

=========================================
Basic Data
=========================================
Target State: Stationary
Moving Target - Distance: 75 cm, Energy: 0
Stationary Target - Distance: 189 cm, Energy: 100
Detection Distance: 100 cm
Light Sensor Value: 0
Output Pin: Unoccupied
Data Age: 46 ms

=========================================
Engineering Data
=========================================
Max Moving Gate: 8
Max Stationary Gate: 8

Moving Energy Gates:
Gate 0: 26
Gate 1: 18
Gate 2: 19
Gate 3: 27
Gate 4: 14
Gate 5: 12
Gate 6: 9
Gate 7: 5

Stationary Energy Gates:
Gate 0: 0
Gate 1: 0
Gate 2: 100
Gate 3: 100
Gate 4: 100
Gate 5: 100
Gate 6: 51
Gate 7: 31
Data Age: 79 ms
```
