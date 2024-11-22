#include <Arduino.h>
#include "LD2410.h"

LD2410 ld2410;

LD2410::ConfigurationData originalConfig;

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

    // Reading current configuration
    Serial.println("Reading initial configuration...");
    if (!ld2410.readConfiguration())
    {
        Serial.println("Failed to read initial configuration");
        return;
    }
    originalConfig = ld2410.getCurrentConfiguration();
    originalConfig.printOut(Serial);

    // Modify all gates with test values
    Serial.println("\nModifying all gate sensitivities...");
    for (uint8_t gate = 0; gate <= 8; gate++)
    {
        uint8_t testMoving = 50;     // Set moving sensitivities to 50
        uint8_t testStationary = 40; // Set stationary sensitivities to 40

        if (!ld2410.setGateSensitivityThreshold(gate, testMoving, testStationary))
        {
            Serial.print("Failed to set gate ");
            Serial.println(gate);
            return;
        }
        Serial.print("Set gate ");
        Serial.println(gate);
    }

    // Restart sensor
    Serial.println("\nRestarting sensor, to proove that the changes are persistent...");
    if (!ld2410.restart())
    {
        Serial.println("Failed to restart sensor");
        return;
    }

    // Wait for sensor to reboot
    Serial.println("Waiting 5 seconds for sensor to restart...");
    delay(5000);

    // Reinitialize UART
    if (!ld2410.beginUART(18, 17, Serial2))
    {
        Serial.println("Failed to reinitialize UART after restart");
        return;
    }

    // Read new configuration
    Serial.println("\nReading new configuration...");
    if (!ld2410.readConfiguration())
    {
        Serial.println("Failed to read new configuration");
        return;
    }

    // Compare configurations
    Serial.println("\nComparing configurations:");
    const auto &newConfig = ld2410.getCurrentConfiguration();
    bool success = true;

    for (uint8_t gate = 0; gate <= 8; gate++)
    {
        Serial.print("Gate ");
        Serial.print(gate);
        Serial.println(":");

        Serial.print("  Moving     - Original: ");
        Serial.print(originalConfig.motionSensitivity[gate]);
        Serial.print(", New: ");
        Serial.print(newConfig.motionSensitivity[gate]);
        Serial.println(newConfig.motionSensitivity[gate] == 50 ? " ✓" : " ✗");

        Serial.print("  Stationary - Original: ");
        Serial.print(originalConfig.stationarySensitivity[gate]);
        Serial.print(", New: ");
        Serial.print(newConfig.stationarySensitivity[gate]);
        Serial.println(newConfig.stationarySensitivity[gate] == 40 ? " ✓" : " ✗");

        if (newConfig.motionSensitivity[gate] != 50 ||
            newConfig.stationarySensitivity[gate] != 40)
        {
            success = false;
        }
    }

    Serial.println(success ? "\nTest PASSED - All values persisted correctly" : "\nTest FAILED - Some values did not persist");
}

void loop()
{
}