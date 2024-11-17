#include <Arduino.h>
#include "LD2410.h"

// Create an instance of the LD2410 class
LD2410 ld2410;

void setup()
{
    // Initialize the serial interface for the monitor
    Serial.begin(115200);
    while (!Serial)
        ; // Wait until Serial is ready

    // Optional: Enable debug messages from the library
    ld2410.useDebug(Serial);

    // Initialize the UART for the sensor
    uint8_t esp32_rx_pin = 17; // ESP32 RX pin (connected to sensor TX)
    uint8_t esp32_tx_pin = 18; // ESP32 TX pin (connected to sensor RX)

    if (!ld2410.beginUART(esp32_rx_pin, esp32_tx_pin, Serial2))
    {
        Serial.println("Failed to initialize LD2410 UART");
    }
    else
    {
        Serial.println("LD2410 UART initialized successfully");
    }

    // Optional: Configure the sensor (e.g., enable Engineering Mode)
    if (!ld2410.enableEngineeringMode())
    {
        Serial.print("Failed to enable engineering mode: ");
        Serial.println(ld2410.getErrorString());
    }
}

void loop()
{
    // Process incoming UART data from the sensor
    ld2410.processUART();

    // Periodically output sensor data
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 1000)
    {
        ld2410.prettyPrintData(Serial);
        lastPrintTime = millis();
    }
}
