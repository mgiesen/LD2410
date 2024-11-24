#include "LD2410_Dashboard.h"

LD2410_Dashboard::LD2410_Dashboard(LD2410 &sensor, uint16_t port)
    : _sensor(sensor), _server(port), _ws("/ws"), _lastUpdate(0), _running(false)
{
}

bool LD2410_Dashboard::begin()
{
    if (_running)
    {
        return true;
    }

    if (!SPIFFS.begin())
    {
        return false;
    }

    // Setup WebSocket handlers
    _ws.onEvent([this](AsyncWebSocket *server,
                       AsyncWebSocketClient *client,
                       AwsEventType type,
                       void *arg,
                       uint8_t *data,
                       size_t len)
                { this->onWebSocketEvent(server, client, type, arg, data, len); });

    // Setup sensor callbacks
    _sensor.onStatusUpdate([this](const LD2410::BasicData &basic,
                                  const LD2410::EngineeringData &eng)
                           { this->handleSensorData(basic, eng); });

    _sensor.onConfigUpdate([this](const LD2410::ConfigurationData &config)
                           { this->handleConfigData(config); });

    // Add WebSocket to server
    _server.addHandler(&_ws);
    setupRoutes();

    // Start server
    _server.begin();
    _running = true;
    return true;
}

void LD2410_Dashboard::end()
{
    if (!_running)
    {
        return;
    }

    _ws.closeAll();
    _server.end();
    _running = false;
}

void LD2410_Dashboard::handle()
{
    if (!_running)
    {
        return;
    }

    _ws.cleanupClients();

    if (millis() - _lastUpdate >= 100)
    {
        broadcastStatus();
        _lastUpdate = millis();
    }
}

void LD2410_Dashboard::setupRoutes()
{
    _server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
}

void LD2410_Dashboard::onWebSocketEvent(AsyncWebSocket *server,
                                        AsyncWebSocketClient *client,
                                        AwsEventType type,
                                        void *arg,
                                        uint8_t *data,
                                        size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        // Send initial state on connect
        broadcastStatus();
        broadcastConfig();
        broadcastDeviceInfo();
        break;

    case WS_EVT_DATA:
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len)
        {
            StaticJsonDocument<512> doc;
            DeserializationError error = deserializeJson(doc, (char *)data);

            if (!error)
            {
                handleCommand(client, doc);
            }
        }
        break;
    }
    }
}

void LD2410_Dashboard::handleCommand(AsyncWebSocketClient *client,
                                     JsonDocument &command)
{
    const char *cmd = command["command"];
    if (!cmd)
        return;

    if (strcmp(cmd, "getConfig") == 0)
    {
        _sensor.readConfiguration();
    }
    else if (strcmp(cmd, "getDeviceInfo") == 0)
    {
        client->text(createDeviceInfoJSON());
    }
    else if (strcmp(cmd, "setSensitivity") == 0)
    {
        uint8_t gate = command["gate"];
        const char *type = command["type"];
        uint8_t value = command["value"];

        if (strcmp(type, "motion") == 0)
        {
            _sensor.setGateSensitivityThreshold(gate, value, 0);
        }
        else if (strcmp(type, "stationary") == 0)
        {
            _sensor.setGateSensitivityThreshold(gate, 0, value);
        }
    }
    else if (strcmp(cmd, "setGlobal") == 0)
    {
        const char *setting = command["setting"];
        uint8_t value = command["value"];

        if (strcmp(setting, "maxMovingGate") == 0)
        {
            _sensor.setMaxValues(value, 0, 0);
        }
        else if (strcmp(setting, "maxStationaryGate") == 0)
        {
            _sensor.setMaxValues(0, value, 0);
        }
        else if (strcmp(setting, "timeout") == 0)
        {
            _sensor.setMaxValues(0, 0, value);
        }
    }
    else if (strcmp(cmd, "setDistanceResolution") == 0)
    {
        bool value = command["value"];
        _sensor.setDistanceResolution(value);
    }
    else if (strcmp(cmd, "enableEngineeringMode") == 0)
    {
        _sensor.enableEngineeringMode();
    }
    else if (strcmp(cmd, "disableEngineeringMode") == 0)
    {
        _sensor.disableEngineeringMode();
    }
    else if (strcmp(cmd, "restart") == 0)
    {
        _sensor.restart();
    }
    else if (strcmp(cmd, "factoryReset") == 0)
    {
        _sensor.factoryReset();
    }
}

void LD2410_Dashboard::handleSensorData(const LD2410::BasicData &basic,
                                        const LD2410::EngineeringData &engineering)
{
    if (_ws.count() > 0)
    {
        String json = createStatusJSON(basic, engineering);
        _ws.textAll(json);
    }
}

void LD2410_Dashboard::handleConfigData(const LD2410::ConfigurationData &config)
{
    if (_ws.count() > 0)
    {
        String json = createConfigJSON(config);
        _ws.textAll(json);
    }
}

String LD2410_Dashboard::createStatusJSON(const LD2410::BasicData &basic,
                                          const LD2410::EngineeringData &engineering)
{
    StaticJsonDocument<1024> doc;

    doc["type"] = "status";
    doc["detectionState"] = static_cast<int>(basic.targetState);
    doc["movingDistance"] = basic.movingTargetDistance;
    doc["stationaryDistance"] = basic.stationaryTargetDistance;
    doc["totalEnergy"] = basic.movingTargetEnergy + basic.stationaryTargetEnergy;

    JsonArray movingEnergy = doc.createNestedArray("movingEnergy");
    JsonArray stationaryEnergy = doc.createNestedArray("stationaryEnergy");

    for (int i = 0; i < LD2410_MAX_GATES; i++)
    {
        movingEnergy.add(engineering.movingEnergyGates[i]);
        stationaryEnergy.add(engineering.stationaryEnergyGates[i]);
    }

    String output;
    serializeJson(doc, output);
    return output;
}

String LD2410_Dashboard::createConfigJSON(const LD2410::ConfigurationData &config)
{
    StaticJsonDocument<1024> doc;

    doc["type"] = "config";
    doc["maxMovingGate"] = config.configuredMaxMotionGate;
    doc["maxStationaryGate"] = config.configuredMaxStationaryGate;
    doc["timeout"] = config.noOccupancyDuration;

    JsonArray gates = doc.createNestedArray("gates");
    for (int i = 0; i < LD2410_MAX_GATES; i++)
    {
        JsonObject gate = gates.createNestedObject();
        gate["motionSensitivity"] = config.motionSensitivity[i];
        gate["stationarySensitivity"] = config.stationarySensitivity[i];
    }

    String output;
    serializeJson(doc, output);
    return output;
}

String LD2410_Dashboard::createDeviceInfoJSON()
{
    StaticJsonDocument<256> doc;

    doc["type"] = "deviceInfo";
    doc["macAddress"] = _sensor.getMacAddress();
    doc["firmwareVersion"] = _sensor.getFirmwareVersion();

    String output;
    serializeJson(doc, output);
    return output;
}

void LD2410_Dashboard::broadcastStatus()
{
    if (_ws.count() > 0)
    {
        handleSensorData(_sensor.getBasicData(), _sensor.getEngineeringData());
    }
}

void LD2410_Dashboard::broadcastConfig()
{
    if (_ws.count() > 0)
    {
        handleConfigData(_sensor.getCurrentConfiguration());
    }
}

void LD2410_Dashboard::broadcastDeviceInfo()
{
    if (_ws.count() > 0)
    {
        _ws.textAll(createDeviceInfoJSON());
    }
}