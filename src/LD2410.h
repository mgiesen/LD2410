/*
 * LD2410.h
 *
 * Author: Maximilian Giesen
 * Version: UNRELEASED
 * Repository: https://github.com/mgiesen/ld2410
 *
 * A streamlined library for the LD2410 sensor, providing a high-level interface for UART communication and output observation, with minimal user code required.
 */

#ifndef LD2410_H
#define LD2410_H

#include <Arduino.h>

// Protocol Constants
#define LD2410_BUFFER_SIZE 120
#define LD2410_MAX_FRAME_LENGTH 60
#define LD2410_COMMAND_TIMEOUT 2000
#define LD2410_CONFIG_DELAY 100

// Distance Gates
#define LD2410_MAX_GATES 8

// Sensitivity Ranges
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
        NOT_INITIALIZED,
    };

    // Target states as defined in protocol
    enum class TargetState
    {
        NO_TARGET = 0x00,
        MOVING = 0x01,
        STATIONARY = 0x02,
        MOVING_AND_STATIONARY = 0x03
    };

    struct BasicData
    {
        TargetState targetState;
        uint16_t movingTargetDistance;
        uint8_t movingTargetEnergy;
        uint16_t stationaryTargetDistance;
        uint8_t stationaryTargetEnergy;
        uint16_t detectionDistance;
        uint8_t lightSensorValue;
        bool outPinState;
        unsigned long lastSensorDataUpdate;

        bool isBasicDataCurrent(const int timeout = 1500) const
        {
            return millis() - lastSensorDataUpdate < timeout;
        }

        void printOut(Stream &serial) const
        {
            serial.println();
            serial.println(F("========================================="));
            serial.println(F("Basic Data "));
            serial.println(F("========================================="));
            serial.print(F("Target State: "));
            switch (targetState)
            {
            case TargetState::NO_TARGET:
                serial.println(F("No Target"));
                break;
            case TargetState::MOVING:
                serial.println(F("Moving"));
                break;
            case TargetState::STATIONARY:
                serial.println(F("Stationary"));
                break;
            case TargetState::MOVING_AND_STATIONARY:
                serial.println(F("Moving & Stationary"));
                break;
            }

            serial.print(F("Moving Target - Distance: "));
            serial.print(movingTargetDistance);
            serial.print(F(" cm, Energy: "));
            serial.println(movingTargetEnergy);

            serial.print(F("Stationary Target - Distance: "));
            serial.print(stationaryTargetDistance);
            serial.print(F(" cm, Energy: "));
            serial.println(stationaryTargetEnergy);

            serial.print(F("Detection Distance: "));
            serial.print(detectionDistance);
            serial.println(F(" cm"));

            serial.print(F("Light Sensor Value: "));
            serial.println(lightSensorValue);

            serial.print(F("Output Pin: "));
            serial.println(outPinState ? F("Occupied") : F("Unoccupied"));

            serial.print(F("Data Age: "));
            serial.print(millis() - lastSensorDataUpdate);
            serial.println(F(" ms"));
        }
    };

    struct EngineeringData
    {
        uint8_t maxMovingGate;
        uint8_t maxStationaryGate;
        uint8_t movingEnergyGates[LD2410_MAX_GATES];
        uint8_t stationaryEnergyGates[LD2410_MAX_GATES];
        unsigned long lastEngineeringDataUpdate;

        bool isEngineeringDataCurrent(const int timeout = 1500) const
        {
            return millis() - lastEngineeringDataUpdate < timeout;
        }

        void printOut(Stream &serial) const
        {
            serial.println();
            serial.println(F("========================================="));
            serial.println(F("Engineering Data "));
            serial.println(F("========================================="));
            serial.print(F("Max Moving Gate: "));
            serial.println(maxMovingGate);
            serial.print(F("Max Stationary Gate: "));
            serial.println(maxStationaryGate);

            serial.println();
            serial.println(F("Moving Energy Gates:"));
            for (int i = 0; i < LD2410_MAX_GATES; i++)
            {
                serial.print(F("Gate "));
                serial.print(i);
                serial.print(F(": "));
                serial.println(movingEnergyGates[i]);
            }

            serial.println();
            serial.println(F("Stationary Energy Gates:"));
            for (int i = 0; i < LD2410_MAX_GATES; i++)
            {
                serial.print(F("Gate "));
                serial.print(i);
                serial.print(F(": "));
                serial.println(stationaryEnergyGates[i]);
            }

            serial.print(F("Data Age: "));
            serial.print(millis() - lastEngineeringDataUpdate);
            serial.println(F(" ms"));
        }
    };

    struct ConfigurationData
    {
        uint8_t maxDistanceGate;
        uint8_t configuredMaxMotionGate;
        uint8_t configuredMaxStationaryGate;
        uint8_t motionSensitivity[9];
        uint8_t stationarySensitivity[9];
        uint16_t noOccupancyDuration;
        unsigned long lastConfigurationgDataUpdate;

        void printOut(Stream &serial) const
        {
            serial.println();
            serial.println(F("========================================="));
            serial.println(F("Configuration Data "));
            serial.println(F("========================================="));
            serial.print(F("Max Distance Gate: "));
            serial.println(maxDistanceGate);
            serial.print(F("Configured Max Motion Gate: "));
            serial.println(configuredMaxMotionGate);
            serial.print(F("Configured Max Stationary Gate: "));
            serial.println(configuredMaxStationaryGate);
            serial.print(F("No Occupancy Duration: "));
            serial.print(noOccupancyDuration);
            serial.println(F(" seconds"));

            serial.println();
            serial.println(F("Motion Sensitivity:"));
            for (int i = 0; i < 9; i++)
            {
                serial.print(F("Gate "));
                serial.print(i);
                serial.print(F(": "));
                serial.println(motionSensitivity[i]);
            }

            serial.println();
            serial.println(F("Stationary Sensitivity:"));
            for (int i = 0; i < 9; i++)
            {
                serial.print(F("Gate "));
                serial.print(i);
                serial.print(F(": "));
                serial.println(stationarySensitivity[i]);
            }

            serial.println();
            serial.print(F("Data Age: "));
            serial.print(millis() - lastConfigurationgDataUpdate);
            serial.println(F(" ms"));
        }
    };

    LD2410();

    //===========================================
    // Initialization & Setup
    //===========================================

    bool beginUART(uint8_t mcu_rx_pin, uint8_t mcu_tx_pin, HardwareSerial &serial, unsigned long baud = 256000);
    bool beginOutputObservation(uint8_t pin, void (*callback)(bool), uint8_t pin_mode_option);
    void useDebug(Stream &debugSerial);

    //===========================================
    // Loop
    //===========================================

    void readSensorData(uint8_t maxBytesPerLoop = 32);

    //===========================================
    // Data Access
    //===========================================

    const BasicData &getBasicData() const { return _basicData; }
    const EngineeringData &getEngineeringData() const { return _engineeringData; }
    const ConfigurationData &getCurrentConfiguration() const { return _currentConfig; }
    const char *getLastErrorString() const;

    String getMacAddress();
    String getFirmwareVersion();

    //===========================================
    // Configuration Commands
    //===========================================
    bool setMaxValues(uint8_t movingGate, uint8_t stationaryGate, uint16_t timeout);
    bool setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary);
    bool enableEngineeringMode();
    bool disableEngineeringMode();
    bool setBaudRate(unsigned long baudRate);
    bool getDistanceResolution(uint8_t &resolution);
    bool setDistanceResolution(bool use020mResolution);
    bool factoryReset();
    bool restart();
    bool readConfiguration();

    //===========================================
    // Callbacks
    //===========================================
    void onStatusUpdate(std::function<void(const BasicData &, const EngineeringData &)> callback)
    {
        _statusCallback = callback;
    }

    void onConfigUpdate(std::function<void(const ConfigurationData &)> callback)
    {
        _configCallback = callback;
    }

private:
    std::function<void(const BasicData &, const EngineeringData &)> _statusCallback = nullptr;
    std::function<void(const ConfigurationData &)> _configCallback = nullptr;

    friend class CommandManager;

    class CircularBuffer
    {
    public:
        CircularBuffer();
        bool add(uint8_t byte);
        bool read(uint8_t &byte);
        void clear();
        bool isEmpty() const;

    private:
        uint8_t _buffer[LD2410_BUFFER_SIZE];
        uint16_t _head;
        uint16_t _tail;
    };

    class FrameProcessor
    {
    public:
        FrameProcessor();

        bool processFrame(CircularBuffer &buffer);
        bool isAckFrame() const;
        const uint8_t *getFrameData() const;
        uint16_t getFrameLength() const;
        void reset();
        void parseDataFrame(BasicData &data, EngineeringData &engData);

    private:
        uint8_t _frame[LD2410_MAX_FRAME_LENGTH];
        uint16_t _position;
        bool _started;
        bool _isAckFrame;

        bool findFrameStart(CircularBuffer &buffer);
        bool validateFrame() const;
    };

    class CommandManager
    {
    public:
        CommandManager();
        void init(LD2410 *parent);
        void sendCommand(HardwareSerial &serial, const uint8_t *cmd, size_t length);
        bool isInConfigMode() const;
        bool enterConfigMode(HardwareSerial &serial);
        bool exitConfigMode(HardwareSerial &serial);
        bool waitForAck(HardwareSerial &serial, uint16_t expectedCommand, uint8_t *response = nullptr);

    private:
        LD2410 *_parent; // Pointer to the parent LD2410 instance
        bool _inConfigMode;
        bool validateResponse(const uint8_t *response, uint16_t expectedCommand) const;
    };

    //===========================================
    // UART Interface Management
    //===========================================
    struct UARTInterface
    {
        HardwareSerial *serial;
        bool initialized;
        bool isEngineeringMode;
        CircularBuffer buffer;
        FrameProcessor frameProcessor;
        CommandManager commandManager;

        void init(LD2410 *parent)
        {
            serial = nullptr;
            initialized = false;
            isEngineeringMode = false;
            commandManager.init(parent);
        }
    } _uart;

    //===========================================
    // Output Pin Management
    //===========================================
    struct OutputObservation
    {
        uint8_t pin;
        void (*callback)(bool);
        bool started;
        volatile bool lastState;
    } _outputObservation;

    static void IRAM_ATTR digitalOutputInterrupt(void *arg);

    //===========================================
    // Internal State & Data
    //===========================================
    Stream *_debug_serial;
    Error _lastError;
    BasicData _basicData;
    EngineeringData _engineeringData;
    ConfigurationData _currentConfig;

    //===========================================
    // Internal Utility Functions
    //===========================================
    void debugPrint(const char *message);
    void debugPrintln(const char *message);
    bool validateGate(uint8_t gate) const;
    bool validateSensitivity(uint8_t sensitivity) const;
    void setError(Error error);
    void debugPrintHex(const char *prefix, const uint8_t *data, uint16_t length);
};

#endif // LD2410_H
