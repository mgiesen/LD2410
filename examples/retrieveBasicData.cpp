#include <Arduino.h>
#include "LD2410.h"

// Define the MCU pins for the LD2410 UART interface (TYPE uint8_t ONLY!!)
#define ld2410tx 17
#define ld2410rx 18

// Create an instance of the LD2410 class
LD2410 ld2410;

void setup()
{
    // Initialize the serial interface for the monitor
    Serial.begin(115200);
    while (!Serial)
        ; // Wait until Serial is ready

    // Optionally enable debug messages from the library
    ld2410.useDebug(Serial);

    // Initialize the UART interface
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

    static unsigned long lastPrintTime = 0;

    if (millis() - lastPrintTime >= 3000)
    {
        ld2410.prettyPrintData(Serial);
        lastPrintTime = millis();
    }
}