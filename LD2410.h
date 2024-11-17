/*
 * LD2410.h
 *
 * Author: Maximilian Giesen
 * Version: UNRELEASED
 * Repository: https://github.com/mgiesen/ld2410
 *
 * A lightweight library for the HiLink HLK-LD2410 sensor, enabling easy UART communication
 * and efficient monitoring of sensor output with minimal overhead.
 */

#ifndef LD2410_H
#define LD2410_H

#include <Arduino.h>

// Protocol Constants
#define LD2410_BUFFER_SIZE 1024
#define LD2410_MAX_FRAME_LENGTH 1024
#define LD2410_COMMAND_TIMEOUT 1000
#define LD2410_COMMAND_DELAY 100

// Distance Gates
#define LD2410_MAX_GATES 8
#define LD2410_MAX_DISTANCE_075M (LD2410_MAX_GATES * 75) // cm with 0.75m resolution
#define LD2410_MAX_DISTANCE_020M (LD2410_MAX_GATES * 20) // cm with 0.20m resolution

// Sensitivity Ranges
#define LD2410_MIN_SENSITIVITY 0
#define LD2410_MAX_SENSITIVITY 100

class LD2410
{
public:
    // Error states
    enum class Error
    {
        NONE = 0,
        BUFFER_OVERFLOW,
        INVALID_FRAME,
        COMMAND_FAILED,
        INVALID_PARAMETER,
        TIMEOUT,
        NOT_INITIALIZED
    };

    // Target states as defined in protocol
    enum class TargetState
    {
        NO_TARGET = 0x00,
        MOVING = 0x01,
        STATIONARY = 0x02,
        MOVING_AND_STATIONARY = 0x03
    };

    struct SensorData
    {
        TargetState targetState;
        uint16_t movingTargetDistance;     // in cm
        uint8_t movingTargetEnergy;        // 0-100
        uint16_t stationaryTargetDistance; // in cm
        uint8_t stationaryTargetEnergy;    // 0-100
        uint16_t detectionDistance;        // in cm
        uint8_t lightSensorValue;          // 0-255
        bool outPinState;                  // true = occupied
    };

    struct EngineeringData : public SensorData
    {
        uint8_t maxMovingGate;
        uint8_t maxStationaryGate;
        uint8_t movingEnergyGates[LD2410_MAX_GATES];
        uint8_t stationaryEnergyGates[LD2410_MAX_GATES];
    };

    LD2410();

    // Setup functions
    bool beginUART(uint8_t esp32_rx_pin, uint8_t esp32_tx_pin, HardwareSerial &serial, unsigned long baud = 256000);
    bool beginOutputObservation(uint8_t pin, void (*callback)(bool), uint8_t pin_mode_option);
    void useDebug(Stream &debugSerial);
    Error getLastError() const { return _lastError; }
    const char *getErrorString() const;

    // Main Loop
    void processUART();

    // Basic configuration
    bool setMaxValues(uint8_t movingGate, uint8_t stationaryGate, uint16_t timeout);
    bool setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary);
    bool enableEngineeringMode();
    bool disableEngineeringMode();

    // Advanced configuration
    bool setBaudRate(unsigned long baudRate);
    bool setDistanceResolution(bool use020mResolution);
    bool factoryReset();
    bool restart();

    // Data access
    const SensorData &getCurrentData() const { return _currentData; }
    const EngineeringData &getEngineeringData() const { return _engineeringData; }

    // Utility functions
    bool readConfiguration();
    void prettyPrintData(Stream &output);

private:
    // Output observation configuration
    struct OutputObservation
    {
        uint8_t pin;
        void (*callback)(bool);
        bool started;
        volatile bool lastState;
    } _outputObservation;

    // UART interface configuration
    struct UARTInterface
    {
        HardwareSerial *serial;
        bool initialized;
        bool commandMode;
        bool isEngineeringMode;
        unsigned long lastCommandTime;

        // Buffer management
        uint8_t buffer[LD2410_BUFFER_SIZE];
        uint16_t bufferHead;
        uint16_t bufferTail;

        // Frame handling
        uint8_t frame[LD2410_MAX_FRAME_LENGTH];
        uint16_t framePosition;
        bool frameStarted;
        bool isAckFrame;

        void init()
        {
            serial = nullptr;
            initialized = false;
            commandMode = false;
            isEngineeringMode = false;
            lastCommandTime = 0;
            bufferHead = 0;
            bufferTail = 0;
            framePosition = 0;
            frameStarted = false;
            isAckFrame = false;
        }
    } _uart;

    // Hardware interfaces
    Stream *_debug_serial;

    // State management
    Error _lastError;

    // Data storage
    SensorData _currentData;
    EngineeringData _engineeringData;

    // Output observation
    static void IRAM_ATTR digitalOutputInterrupt(void *arg);

    // Internal functions
    bool sendCommand(const uint8_t *cmd, size_t length);
    bool enterConfigMode();
    bool exitConfigMode();
    bool waitForAck(uint16_t expectedCommand);

    // Buffer management
    bool addToBuffer(uint8_t byte);
    bool readFromBuffer(uint8_t &byte);
    void clearBuffer();

    // Frame handling
    bool findFrameStart();
    bool readFrame();
    bool validateFrame();
    void parseDataFrame();
    bool parseCommandFrame(uint16_t expectedCommand);

    // Utility functions
    void debugPrint(const char *message);
    void debugPrintln(const char *message);
    void waitFor(unsigned long ms);
    bool validateGate(uint8_t gate) const;
    bool validateSensitivity(uint8_t sensitivity) const;
    void setError(Error error);
};

#endif // LD2410_H
