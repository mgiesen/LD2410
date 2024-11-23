#include <Arduino.h>
#include "LD2410.h"

LD2410 ld2410;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    ld2410.useDebug(Serial);

    if (!ld2410.beginUART(18, 17, Serial2))
    {
        Serial.println("UART initialization failed");
        return;
    }

    if (!ld2410.enableEngineeringMode())
    {
        Serial.println("Failed to enable engineering mode");
    }
}

void loop()
{
    ld2410.readSensorData();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 2500)
    {
        const auto &engData = ld2410.getEngineeringData();
        engData.printOut(Serial);
        lastPrint = millis();
    }

    yield();
}