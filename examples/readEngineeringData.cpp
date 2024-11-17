#include <Arduino.h>
#include "LD2410.h"

// Create an instance of the LD2410 class
LD2410 ld2410;

// Switching variable for engineering mode
bool isEngineeringMode = true;

void setup()
{
    // Initialize the serial interface for the monitor
    Serial.begin(115200);
    while (!Serial)
        ; // Wait until Serial is ready

    // Enable debug messages from the library
    ld2410.useDebug(Serial);

    // Initialize the UART for the sensor
    uint8_t ld2410tx = 17; // ESP32 RX pin (connected to sensor TX)
    uint8_t ld2410rx = 18; // ESP32 TX pin (connected to sensor RX)

    if (!ld2410.beginUART(ld2410tx, ld2410rx, Serial2))
    {
        Serial.println("Failed to initialize LD2410 UART");
    }
    else
    {
        Serial.println("LD2410 UART initialized successfully");
    }
}

void loop()
{
    // Process incoming UART data from the sensor
    ld2410.processUART();

    // Periodically output sensor data and toggle mode
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 2000)
    {
        ld2410.prettyPrintData(Serial);
        lastPrintTime = millis();

        // Toggle engineering mode
        if (isEngineeringMode)
        {
            // Disable engineering mode
            if (!ld2410.disableEngineeringMode())
            {
                Serial.print("Failed to disable engineering mode: ");
                Serial.println(ld2410.getErrorString());
            }
            else
            {
                Serial.println("Engineering mode disabled");
                isEngineeringMode = false;
            }
        }
        else
        {
            // Enable engineering mode
            if (!ld2410.enableEngineeringMode())
            {
                Serial.print("Failed to enable engineering mode: ");
                Serial.println(ld2410.getErrorString());
            }
            else
            {
                Serial.println("Engineering mode enabled");
                isEngineeringMode = true;
            }
        }
    }
}
