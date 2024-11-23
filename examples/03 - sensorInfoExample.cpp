#include <Arduino.h>
#include "LD2410.h"

LD2410 ld2410;

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

    Serial.println("\n=========================================");
    Serial.println("Sensor Information");
    Serial.println("=========================================");

    String macAddress = ld2410.getMacAddress();
    Serial.print("MAC Address: ");
    Serial.println(macAddress);

    String firmware = ld2410.getFirmwareVersion();
    Serial.print("Firmware Version: ");
    Serial.println(firmware);
}

void loop()
{
}