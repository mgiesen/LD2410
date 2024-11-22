#include <Arduino.h>
#include "LD2410.h"

LD2410 ld2410;

volatile bool stateChanged = false;
volatile bool currentState = false;

void onPresenceChange(bool occupied)
{
    stateChanged = true;
    currentState = occupied;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    ld2410.useDebug(Serial);

    ld2410.beginOutputObservation(5, onPresenceChange, INPUT_PULLDOWN);
}

void loop()
{
    if (stateChanged)
    {
        Serial.print("Output pin state: ");
        Serial.println(currentState ? "HIGH" : "LOW");
        stateChanged = false;
    }

    yield();
}