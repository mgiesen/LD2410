# Example: Reading state of out pin

`00 - outputPinExample.cpp`

## Demonstrates

- Starting output pin monitoring
- Starting debugging
- Printing message on state changes

## Console Output

```console
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x1 (POWERON),boot:0x2b (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3808,len:0x4bc
load:0x403c9700,len:0xbd8
load:0x403cc700,len:0x2a0c
entry 0x403c98d0
[LD2410 DEBUGGER] Debug mode enabled
[LD2410 DEBUGGER] Output observation started successfully
Output pin state: LOW
Output pin state: HIGH
```

# Example: Reading basic data

`01 - basicExample.cpp`

## Demonstrates

- Starting UART interface
- Starting debugging
- Reading and printing basic sensor data

## Console Output

```console
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0x2b (SPI_FAST_FLASH_BOOT)
Saved PC:0x40378be1
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3808,len:0x4bc
load:0x403c9700,len:0xbd8
load:0x403cc700,len:0x2a0c
entry 0x403c98d0
[LD2410 DEBUGGER] Debug mode enabled
[LD2410 DEBUGGER] UART initialized successfully

=========================================
Basic Data
=========================================
Target State: Stationary
Moving Target - Distance: 84 cm, Energy: 0
Stationary Target - Distance: 84 cm, Energy: 100
Detection Distance: 70 cm
Light Sensor Value: 103
Output Pin: Occupied
Data Age: 73 ms
```

# Example: Reading engineering data

`02 - engineeringExample.cpp`

## Demonstrates

- Starting UART interface
- Starting debugging
- Enable Engineering Mode
- Reading and printing engineering sensor data

## Console Output

```console
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x1 (POWERON),boot:0x2b (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3808,len:0x4bc
load:0x403c9700,len:0xbd8
load:0x403cc700,len:0x2a0c
entry 0x403c98d0
[LD2410 DEBUGGER] Debug mode enabled
[LD2410 DEBUGGER] UART initialized successfully
[LD2410 DEBUGGER] Engineering mode enabled

=========================================
Engineering Data
=========================================
Max Moving Gate: 8
Max Stationary Gate: 8

Moving Energy Gates:
Gate 0: 18
Gate 1: 34
Gate 2: 19
Gate 3: 36
Gate 4: 13
Gate 5: 10
Gate 6: 6
Gate 7: 4

Stationary Energy Gates:
Gate 0: 0
Gate 1: 0
Gate 2: 100
Gate 3: 100
Gate 4: 100
Gate 5: 100
Gate 6: 100
Gate 7: 100
Data Age: 63 ms
```

# Example: Reading Sensor Information

`03 - sensorInfoExample.cpp`

## Demonstrates

- Getting MAC address
- Getting firmware version
- Proper error handling
- Debug output configuration

> [!WARNING]  
> MAC Address Implementation Notes:
>
> - The manufacturer's documentation mentions a 3-byte address in big endian
> - The actual response contains 6 bytes
> - The returned bytes often contain patterns similar to frame end bytes
>
> Issue needs further investigation.

## Console Output

```console
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0x2b (SPI_FAST_FLASH_BOOT)
Saved PC:0x40378be1
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3808,len:0x4bc
load:0x403c9700,len:0xbd8
load:0x403cc700,len:0x2a0c
entry 0x403c98d0
[LD2410 DEBUGGER] Debug mode enabled
[LD2410 DEBUGGER] UART initialized successfully

=========================================
Sensor Information
=========================================
MAC Address: 08:05:04:03:02:01
Firmware Version: V2.05.1606
```

# Example: Read and modify sensor configuration

`04 - readAndModifyConfig.cpp`

> [!WARNING]  
> Issue identified in sensor responses after restart:
>
> - Before restart:
>   FD FC FB FA 1C 00 61 01 00 00 AA 08 08 08 32 32 32 32 32 32 32 32 32 28 28 28 28 28 28 28 28 28
> - After restart:
>   FD FC FB FA 1A 00 61 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
>
> Issue needs further investigation.

## Demonstrates

- Reading sensor configuration
- Modifying gate sensitivities
- Verifying persistence after restart

## Console Output

```console
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0x2b (SPI_FAST_FLASH_BOOT)
Saved PC:0x40378be1
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3808,len:0x4bc
load:0x403c9700,len:0xbd8
load:0x403cc700,len:0x2a0c
entry 0x403c98d0
[LD2410 DEBUGGER] Debug mode enabled
[LD2410 DEBUGGER] UART initialized successfully
Reading initial configuration...

=========================================
Configuration Data
=========================================
Max Distance Gate: 8
Configured Max Motion Gate: 8
Configured Max Stationary Gate: 8
No Occupancy Duration: 5 seconds

Motion Sensitivity:
Gate 0: 50
Gate 1: 50
Gate 2: 50
Gate 3: 50
Gate 4: 50
Gate 5: 50
Gate 6: 50
Gate 7: 50
Gate 8: 50

Stationary Sensitivity:
Gate 0: 40
Gate 1: 40
Gate 2: 40
Gate 3: 40
Gate 4: 40
Gate 5: 40
Gate 6: 40
Gate 7: 40
Gate 8: 40

Data Age: 46 ms

Modifying all gate sensitivities...
Set gate 0
Set gate 1
Set gate 2
Set gate 3
Set gate 4
Set gate 5
Set gate 6
Set gate 7
Set gate 8

Restarting sensor, to proove that the changes are persistent...
Waiting 5 seconds for sensor to restart...
[LD2410 DEBUGGER] UART initialized successfully

Reading new configuration...
Error: Command failed
Failed to read new configuration
```

# Example: Distance Resolution Configuration

`05 - resolutionExample.cpp`

## Demonstrates

- Querying current resolution setting
- Setting new distance resolution (0.75m vs 0.2m)

## Console Output

```console
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x1 (POWERON),boot:0x2b (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3808,len:0x4bc
load:0x403c9700,len:0xbd8
load:0x403cc700,len:0x2a0c
entry 0x403c98d0
[LD2410 DEBUGGER] Debug mode enabled
[LD2410 DEBUGGER] UART initialized successfully

Initial Configuration:

=========================================
Distance Resolution Configuration
=========================================
Resolution per gate: 0.75 meters
Maximum gates: 8
Effective range: 6.00 meters

Detection distance: 418 cm
Detection distance: 423 cm
Detection distance: 423 cm
Detection distance: 423 cm
Detection distance: 423 cm
Detection distance: 423 cm
Detection distance: 423 cm
Detection distance: 423 cm
Detection distance: 423 cm
Detection distance: 423 cm

Switching to high (0.2m) resolution...

=========================================
Distance Resolution Configuration
=========================================
Resolution per gate: 0.20 meters
Maximum gates: 8
Effective range: 1.60 meters

Detection distance: 111 cm
Detection distance: 107 cm
Detection distance: 106 cm
Detection distance: 106 cm
Detection distance: 106 cm
Detection distance: 106 cm
Detection distance: 106 cm
Detection distance: 106 cm
Detection distance: 106 cm
Detection distance: 106 cm
....
```
