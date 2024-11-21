#include <Arduino.h>
#include "LD2410.h"

// Create an instance of the LD2410 class
LD2410 ld2410;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    ld2410.useDebug(Serial);

    if (!ld2410.beginUART(18, 17, Serial2))
    {
        Serial.println("Failed to initialize UART");
        return;
    }

    String mac = ld2410.getMacAddress();
    Serial.print("Sensor MAC Address: ");
    Serial.println(mac);

    String version = ld2410.getFirmwareVersion();
    Serial.print("Firmware Version: ");
    Serial.println(version);

    if (!ld2410.enableEngineeringMode())
    {
        Serial.println("Failed to enable engineering mode");
    }
}

void loop()
{
    ld2410.processUART();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 2000)
    {
        lastPrint = millis();
        ld2410.prettyPrintData(Serial);
    }

    yield();
}