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
}

void loop()
{
    ld2410.readSensorData();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 2500)
    {
        const auto &basicData = ld2410.getBasicData();
        basicData.printOut(Serial);
        lastPrint = millis();
    }

    yield();
}