# Debugging Report: LD2410B Sensor Communication Interruption

## Problem

While working with the LD2410, I noticed that data communication frequently stops under specific conditions. To investigate the issue, I simplified the setup and monitored both the UART data output and the state of a physical LED connected to the output pin. I observed that data transmission is interrupted whenever the output pin goes HIGH, causing the LED to turn on.

Additionally, I noticed erratic behavior: even when the setup maintains a safe distance where the output isn’t triggered, the TX LED on the MCU indicates an inconsistent data stream. In some cases, the transmission is interrupted sporadically and may stop entirely for extended periods.

To analyze the problem further, I implemented an interrupt routine to monitor the output pin. To better visualize the issues, I recorded multiple test cases and used the timestamps to plot the behavior as diagrams.

## Hardware Setup

- ESP32 microcontroller
- LD2410B radar sensor
- Connections:
  - LD2410B OUT → GPIO5
  - LD2410B UART TX → GPIO17
  - LD2410B UART RX → GPIO18

## Test Case 1

**Steps**

1. Restarted the MCU.
2. Left the application area.
3. Entered the application area for a few seconds.
4. Repeated the process once.
5. Stopped the measurement.

**Data Interpretation**

- No UART communication occurs while a human is present or the data pin is HIGH.
- Visualized by `log_1.txt`
  ![plot_1.png](plots/plot_1.png)
- Visualized by `log_2.txt`
  ![plot_2.png](plots/plot_2.png)

**Raw Data Snippet**

```
...
[14295] F4 F3 F2 F1 0D 00 02 AA 00 4F 00 00 4F 00 1E AC 00 55 00 F8 F7 F6 F5
[14395] F4 F3 F2 F1 0D 00 02 AA 00 4F 00 00 4F 00 1F AC 00 55 00 F8 F7 F6 F5
[14495] F4 F3 F2 F1 0D 00 02 AA 00 4F 00 00 4F 00 1F AC 00 55 00 F8 F7 F6 F5
[14594] RISING
[33993] FALLING
[33994] F4 F3 F2 F1 0D 00 02 AA 00 5A 00 00 5A 00 1D 0F 01 55 00 F8 F7 F6 F5
[34094] F4 F3 F2 F1 0D 00 02 AA 00 5A 00 00 5A 00 1D 0F 01 55 00 F8 F7 F6 F5
[34194] F4 F3 F2 F1 0D 00 02 AA 00 5A 00 00 5A 00 1D 0F 01 55 00 F8 F7 F6 F5
...
```

## Test Case 2

**Steps**

1. Restarted the MCU.
2. Left the application area for > 200 seconds.
3. Entered the application area again.
4. Stopped the measurement.

## Solution

After replacing the sensor, the problem couldn't be reproduced. Therefore, I expect a part-failure. Prooven by `log_replaced_sensor.txt`

![plot_replaced_sensor.png](plots/plot_replaced_sensor.png)

## Code

```cpp
#include <Arduino.h>

#define LD2410_OUT_PIN 5

volatile unsigned long lastInterrupt = 0;
volatile bool output_state = false;
bool needTimestamp = true;

void IRAM_ATTR pinInterrupt()
{
    lastInterrupt = millis();
    output_state = digitalRead(LD2410_OUT_PIN);
}

void setup()
{
    delay(100);
    Serial.begin(115200);
    while (!Serial)
        ;

    pinMode(LD2410_OUT_PIN, INPUT_PULLDOWN);
    output_state = digitalRead(LD2410_OUT_PIN);
    Serial.printf("[%lu] Initial pin state: %s\n", millis(), output_state ? "HIGH" : "LOW");
    Serial.printf("[%lu] Starting LD2410B UART test...\n", millis());

    Serial2.begin(256000, SERIAL_8N1, 17, 18);
    attachInterrupt(digitalPinToInterrupt(LD2410_OUT_PIN), pinInterrupt, CHANGE);
}

void loop()
{
    static unsigned long lastPrintedInterrupt = 0;

    if (lastInterrupt != lastPrintedInterrupt)
    {
        Serial.printf("[%lu] %s\n", lastInterrupt, output_state ? "RISING" : "FALLING");
        lastPrintedInterrupt = lastInterrupt;
    }

    while (Serial2.available())
    {
        uint8_t byte = Serial2.read();

        if (needTimestamp)
        {
            Serial.printf("[%lu] ", millis());
            needTimestamp = false;
        }

        if (byte < 0x10)
        {
            Serial.print("0");
        }
        Serial.print(byte, HEX);
        Serial.print(" ");

        static uint8_t lastBytes[4] = {0};
        for (int i = 0; i < 3; i++)
        {
            lastBytes[i] = lastBytes[i + 1];
        }
        lastBytes[3] = byte;

        if ((lastBytes[0] == 0x04 && lastBytes[1] == 0x03 &&
             lastBytes[2] == 0x02 && lastBytes[3] == 0x01) ||
            (lastBytes[0] == 0xF8 && lastBytes[1] == 0xF7 &&
             lastBytes[2] == 0xF6 && lastBytes[3] == 0xF5))
        {
            Serial.println();
            needTimestamp = true;
        }
    }
}
```
