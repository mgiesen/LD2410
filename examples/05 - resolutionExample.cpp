#include <Arduino.h>
#include "LD2410.h"

LD2410 ld2410;

void printCurrentResolution()
{
    Serial.println("\n=========================================");
    Serial.println("Distance Resolution Configuration");
    Serial.println("=========================================");

    uint8_t resolution;
    if (!ld2410.getDistanceResolution(resolution))
    {
        Serial.println("Failed to read resolution");
        return;
    }

    if (!ld2410.readConfiguration())
    {
        Serial.println("Failed to read configuration");
        return;
    }

    const auto &config = ld2410.getCurrentConfiguration();
    float maxRange = config.maxDistanceGate * (resolution == 1 ? 0.2f : 0.75f);

    Serial.print("Resolution per gate: ");
    Serial.print(resolution == 1 ? "0.20" : "0.75");
    Serial.println(" meters");
    Serial.print("Maximum gates: ");
    Serial.println(config.maxDistanceGate);
    Serial.print("Effective range: ");
    Serial.print(maxRange);
    Serial.println(" meters");
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    ld2410.useDebug(Serial);

    if (!ld2410.beginUART(18, 17, Serial2))
    {
        Serial.println("UART initialization failed");
        return;
    }

    Serial.println("\nInitial Configuration:");
    printCurrentResolution();
}

void loop()
{
    static unsigned long lastResolutionToggle = 0;
    static bool useHighResolution = false;
    static unsigned long lastPrint = 0;

    // Toggle resolution every 10 seconds
    if (millis() - lastResolutionToggle >= 10000)
    {
        useHighResolution = !useHighResolution;
        Serial.print("\nSwitching to ");
        Serial.print(useHighResolution ? "high (0.2m)" : "low (0.75m)");
        Serial.println(" resolution...");

        if (!ld2410.setDistanceResolution(useHighResolution))
        {
            Serial.println("Failed to set resolution");
        }
        else
        {
            printCurrentResolution();
            delay(250);
        }

        lastResolutionToggle = millis();
    }

    ld2410.readSensorData();

    // Print detection data every second
    if (millis() - lastPrint >= 1000)
    {
        const auto &basicData = ld2410.getBasicData();
        Serial.print("Detection distance: ");
        Serial.print(basicData.detectionDistance);
        Serial.println(" cm");
        lastPrint = millis();
    }

    yield();
}