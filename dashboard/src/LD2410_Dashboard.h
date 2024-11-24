#ifndef LD2410_DASHBOARD_H
#define LD2410_DASHBOARD_H

#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include "LD2410.h"

class LD2410_Dashboard
{
public:
    LD2410_Dashboard(LD2410 &sensor, uint16_t port = 80);

    bool begin();
    void end();
    void handle();

private:
    LD2410 &_sensor;
    AsyncWebServer _server;
    AsyncWebSocket _ws;
    unsigned long _lastUpdate;
    bool _running;

    void onWebSocketEvent(AsyncWebSocket *server,
                          AsyncWebSocketClient *client,
                          AwsEventType type,
                          void *arg,
                          uint8_t *data,
                          size_t len);

    void handleSensorData(const LD2410::BasicData &basic,
                          const LD2410::EngineeringData &engineering);
    void handleConfigData(const LD2410::ConfigurationData &config);

    String createStatusJSON(const LD2410::BasicData &basic,
                            const LD2410::EngineeringData &engineering);
    String createConfigJSON(const LD2410::ConfigurationData &config);
    String createDeviceInfoJSON();

    void handleCommand(AsyncWebSocketClient *client,
                       JsonDocument &command);

    void setupRoutes();

    void broadcastStatus();
    void broadcastConfig();
    void broadcastDeviceInfo();
};

#endif // LD2410_DASHBOARD_H