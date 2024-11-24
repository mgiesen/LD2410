#include <Arduino.h>
#include <WiFi.h>
#include "LD2410.h"
#include "LD2410_Dashboard.h"

// WiFi Configuration
const char *ssid = "YOUR_SSID";
const char *password = "YOUR_PASSWORD";

// Global instances
LD2410 radar;                      // Radar sensor instance
LD2410_Dashboard dashboard(radar); // Dashboard instance

void setupWiFi()
{
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void setup()
{
    // Initialize Serial for debugging
    Serial.begin(115200);
    Serial.println("\nStarting LD2410 Demo with Dashboard...");

    // Initialize WiFi
    setupWiFi();

    // Initialize radar sensor
    if (!radar.beginUART(18, 17, Serial2))
    {
        Serial.println("Failed to initialize radar sensor!");
        while (1)
        {
            delay(100);
        }
    }
    Serial.println("Radar sensor initialized");

    // Enable engineering mode for detailed data
    if (radar.enableEngineeringMode())
    {
        Serial.println("Engineering mode enabled");
    }
    else
    {
        Serial.println("Failed to enable engineering mode");
    }

    // Initialize dashboard
    if (!dashboard.begin())
    {
        Serial.println("Failed to initialize dashboard!");
        while (1)
        {
            delay(100);
        }
    }
    Serial.println("Dashboard initialized");

    // Print access information
    Serial.println("\nSystem ready!");
    Serial.print("Dashboard available at: http://");
    Serial.println(WiFi.localIP());
}

void loop()
{
    // Process radar data
    radar.readSensorData();

    // Handle dashboard tasks
    dashboard.handle();

    // Small delay to prevent tight looping
    delay(10);
}