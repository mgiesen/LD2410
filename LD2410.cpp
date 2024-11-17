/*
 * LD2410.cpp
 *
 * Author: Maximilian Giesen
 * Version: UNRELEASED
 * Repository: https://github.com/mgiesen/ld2410
 *
 * A lightweight library for the HiLink HLK-LD2410 sensor, enabling easy UART communication
 * and efficient monitoring of sensor output with minimal overhead.
 */

#include "LD2410.h"

LD2410::LD2410() : _debug_serial(nullptr),
                   _lastError(Error::NONE)
{
    _uart.init();
    memset(&_currentData, 0, sizeof(_currentData));
    memset(&_engineeringData, 0, sizeof(_engineeringData));
}

//=====================================================================================================================
// DEBUGGING
//=====================================================================================================================

void LD2410::useDebug(Stream &debugSerial)
{
    _debug_serial = &debugSerial;
    debugPrintln("Debug mode enabled");
}

void LD2410::debugPrint(const char *message)
{
    if (_debug_serial)
    {
        _debug_serial->print(message);
    }
}

void LD2410::debugPrintln(const char *message)
{
    if (_debug_serial)
    {
        _debug_serial->println(message);
    }
}

//=====================================================================================================================
// OUTPUT PIN OBSERVATION
//=====================================================================================================================

bool LD2410::beginOutputObservation(uint8_t pin, void (*callback)(bool), uint8_t pin_mode_option)
{
    if (callback == nullptr)
    {
        debugPrintln("[LD2410] Callback can't be null. Output observation not started");
        return false;
    }

    // Disable existing interrupt if any
    if (_outputObservation.started)
    {
        debugPrintln("[LD2410] Output observation already started. Reassigning callback...");
        detachInterrupt(digitalPinToInterrupt(_outputObservation.pin));
    }

    // Configure new output observation
    _outputObservation.pin = pin;
    _outputObservation.callback = callback;
    _outputObservation.started = false;
    _outputObservation.lastState = false;

    // Setup pin
    pinMode(pin, pin_mode_option);
    _outputObservation.lastState = digitalRead(pin) == HIGH;

    // Attach interrupt
    attachInterruptArg(digitalPinToInterrupt(pin), digitalOutputInterrupt, this, CHANGE);

    _outputObservation.started = true;
    debugPrintln("[LD2410] Output observation started successfully");
    return true;
}

void IRAM_ATTR LD2410::digitalOutputInterrupt(void *arg)
{
    LD2410 *instance = static_cast<LD2410 *>(arg);
    if (instance && instance->_outputObservation.started)
    {
        bool currentState = digitalRead(instance->_outputObservation.pin) == HIGH;
        if (currentState != instance->_outputObservation.lastState)
        {
            instance->_outputObservation.lastState = currentState;
            instance->_outputObservation.callback(currentState);
        }
    }
}

//=====================================================================================================================
// UART INTERFACE
//=====================================================================================================================

bool LD2410::beginUART(uint8_t esp32_rx_pin, uint8_t esp32_tx_pin, HardwareSerial &serial, unsigned long baud)
{
    _uart.serial = &serial;
    _uart.serial->begin(baud, SERIAL_8N1, esp32_rx_pin, esp32_tx_pin);

    // Wartezeit zum Stabilisieren der seriellen Verbindung
    delay(500);

    // Überprüfe, ob Daten verfügbar sind, um die Verbindung zu verifizieren
    unsigned long start = millis();
    while (millis() - start < 1000)
    {
        if (_uart.serial->available())
        {
            debugPrintln("[LD2410] UART initialized successfully");
            _uart.initialized = true;
            return true;
        }
        delay(10);
    }

    debugPrintln("[LD2410] Failed to initialize UART");
    _uart.initialized = false;
    return false;
}

void LD2410::processUART()
{
    if (!_uart.initialized)
    {
        return;
    }

    const uint8_t maxBytesPerLoop = 32; // Prevent blocking too long
    uint8_t bytesProcessed = 0;

    while (_uart.serial->available() && bytesProcessed < maxBytesPerLoop)
    {
        uint8_t byte = _uart.serial->read();
        if (!addToBuffer(byte))
        {
            setError(Error::BUFFER_OVERFLOW);
            return;
        }
        bytesProcessed++;
    }

    if (readFrame())
    {
        if (!_uart.isAckFrame)
        {
            parseDataFrame();
        }
    }
}

bool LD2410::addToBuffer(uint8_t byte)
{
    uint16_t nextHead = (_uart.bufferHead + 1) % LD2410_BUFFER_SIZE;
    if (nextHead == _uart.bufferTail)
    {
        return false; // Buffer full
    }
    _uart.buffer[_uart.bufferHead] = byte;
    _uart.bufferHead = nextHead;
    return true;
}

bool LD2410::readFromBuffer(uint8_t &byte)
{
    if (_uart.bufferHead == _uart.bufferTail)
    {
        return false;
    }
    byte = _uart.buffer[_uart.bufferTail];
    _uart.bufferTail = (_uart.bufferTail + 1) % LD2410_BUFFER_SIZE;
    return true;
}

void LD2410::clearBuffer()
{
    _uart.bufferHead = _uart.bufferTail = 0;
    _uart.framePosition = 0;
    _uart.frameStarted = false;

    while (_uart.serial && _uart.serial->available())
    {
        _uart.serial->read();
    }
}

bool LD2410::findFrameStart()
{
    uint8_t byte;
    while (readFromBuffer(byte))
    {
        if (byte == 0xF4 || byte == 0xFD)
        {
            _uart.frame[0] = byte;
            _uart.framePosition = 1;
            _uart.frameStarted = true;
            _uart.isAckFrame = (byte == 0xFD);
            return true;
        }
    }
    return false;
}

bool LD2410::readFrame()
{
    if (!_uart.frameStarted && !findFrameStart())
    {
        return false;
    }

    uint8_t byte;
    while (readFromBuffer(byte))
    {
        _uart.frame[_uart.framePosition++] = byte;

        // Check frame length
        if (_uart.framePosition == 8)
        {
            uint16_t frameLength = _uart.frame[4] | (_uart.frame[5] << 8);
            if (frameLength + 10 > LD2410_MAX_FRAME_LENGTH)
            {
                setError(Error::INVALID_FRAME);
                _uart.frameStarted = false;
                _uart.framePosition = 0;
                return false;
            }
        }

        // Validate frame end
        if (_uart.framePosition >= 8)
        {
            if (_uart.isAckFrame)
            {
                if (_uart.frame[_uart.framePosition - 4] == 0x04 &&
                    _uart.frame[_uart.framePosition - 3] == 0x03 &&
                    _uart.frame[_uart.framePosition - 2] == 0x02 &&
                    _uart.frame[_uart.framePosition - 1] == 0x01)
                {
                    _uart.frameStarted = false;
                    return validateFrame();
                }
            }
            else
            {
                if (_uart.frame[_uart.framePosition - 4] == 0xF8 &&
                    _uart.frame[_uart.framePosition - 3] == 0xF7 &&
                    _uart.frame[_uart.framePosition - 2] == 0xF6 &&
                    _uart.frame[_uart.framePosition - 1] == 0xF5)
                {
                    _uart.frameStarted = false;
                    return validateFrame();
                }
            }
        }

        if (_uart.framePosition >= LD2410_MAX_FRAME_LENGTH)
        {
            setError(Error::INVALID_FRAME);
            _uart.frameStarted = false;
            _uart.framePosition = 0;
            return false;
        }
    }

    return false;
}

bool LD2410::validateFrame()
{
    if (_uart.framePosition < 8)
    {
        setError(Error::INVALID_FRAME);
        return false;
    }
    return true;

    // To-Do: Maybe a checksum validation could be added here
}

void LD2410::parseDataFrame()
{
    if (_uart.frame[6] != 0x01 && _uart.frame[6] != 0x02)
    {
        return;
    }

    _uart.isEngineeringMode = (_uart.frame[6] == 0x01);

    if (_uart.frame[7] == 0xAA)
    {
        // Parse basic data
        _currentData.targetState = static_cast<TargetState>(_uart.frame[8]);
        _currentData.movingTargetDistance = _uart.frame[9] | (_uart.frame[10] << 8);
        _currentData.movingTargetEnergy = _uart.frame[11];
        _currentData.stationaryTargetDistance = _uart.frame[12] | (_uart.frame[13] << 8);
        _currentData.stationaryTargetEnergy = _uart.frame[14];
        _currentData.detectionDistance = _uart.frame[15] | (_uart.frame[16] << 8);

        if (_uart.isEngineeringMode)
        {
            // Copy basic data fields manually
            _engineeringData.targetState = _currentData.targetState;
            _engineeringData.movingTargetDistance = _currentData.movingTargetDistance;
            _engineeringData.movingTargetEnergy = _currentData.movingTargetEnergy;
            _engineeringData.stationaryTargetDistance = _currentData.stationaryTargetDistance;
            _engineeringData.stationaryTargetEnergy = _currentData.stationaryTargetEnergy;
            _engineeringData.detectionDistance = _currentData.detectionDistance;
            _engineeringData.lightSensorValue = _currentData.lightSensorValue;
            _engineeringData.outPinState = _currentData.outPinState;

            // Parse additional engineering data
            _engineeringData.maxMovingGate = _uart.frame[17];
            _engineeringData.maxStationaryGate = _uart.frame[18];

            for (int i = 0; i < LD2410_MAX_GATES; i++)
            {
                _engineeringData.movingEnergyGates[i] = _uart.frame[19 + i];
                _engineeringData.stationaryEnergyGates[i] = _uart.frame[28 + i];
            }

            _engineeringData.lightSensorValue = _uart.frame[37];
            _engineeringData.outPinState = _uart.frame[38] != 0;
        }
    }
}

bool LD2410::sendCommand(const uint8_t *cmd, size_t length)
{
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return false;
    }

    clearBuffer();
    waitFor(LD2410_COMMAND_DELAY);

    // Send command frame
    const uint8_t header[] = {0xFD, 0xFC, 0xFB, 0xFA};
    const uint8_t footer[] = {0x04, 0x03, 0x02, 0x01};

    _uart.serial->write(header, 4);
    _uart.serial->write(cmd, length);
    _uart.serial->write(footer, 4);
    _uart.serial->flush();

    // Extract expected command
    uint16_t command = (cmd[3] << 8) | cmd[2];
    uint16_t expectedAck = command | 0x0100;

    return waitForAck(expectedAck);
}

bool LD2410::waitForAck(uint16_t expectedCommand)
{
    unsigned long start = millis();

    while (millis() - start < LD2410_COMMAND_TIMEOUT)
    {
        if (_uart.serial->available())
        {
            if (!addToBuffer(_uart.serial->read()))
            {
                setError(Error::BUFFER_OVERFLOW);
                return false;
            }
        }

        if (readFrame() && _uart.isAckFrame)
        {
            return parseCommandFrame(expectedCommand);
        }
    }

    setError(Error::TIMEOUT);
    debugPrintln("Command timeout");
    return false;
}

bool LD2410::parseCommandFrame(uint16_t expectedCommand)
{
    uint16_t receivedCommand = (_uart.frame[7] << 8) | _uart.frame[6];
    if (receivedCommand != expectedCommand)
    {
        setError(Error::COMMAND_FAILED);
        return false;
    }

    uint16_t status = (_uart.frame[9] << 8) | _uart.frame[8];
    return status == 0;
}

//=====================================================================================================================
// CONFIGURATION COMMANDS
//=====================================================================================================================

const uint8_t CMD_ENABLE_CONFIG[] = {0x04, 0x00, 0xFF, 0x00, 0x01, 0x00};
const uint8_t CMD_EXIT_CONFIG[] = {0x02, 0x00, 0xFE, 0x00};
const uint8_t CMD_ENABLE_ENGINEERING[] = {0x02, 0x00, 0x62, 0x00};
const uint8_t CMD_DISABLE_ENGINEERING[] = {0x02, 0x00, 0x63, 0x00};
const uint8_t CMD_RESTART[] = {0x02, 0x00, 0xA3, 0x00};
const uint8_t CMD_FACTORY_RESET[] = {0x02, 0x00, 0xA2, 0x00};

bool LD2410::enterConfigMode()
{
    if (_uart.commandMode)
    {
        return true;
    }

    if (!sendCommand(CMD_ENABLE_CONFIG, sizeof(CMD_ENABLE_CONFIG)))
    {
        return false;
    }

    _uart.commandMode = true;
    waitFor(LD2410_COMMAND_DELAY);
    return true;
}

bool LD2410::exitConfigMode()
{
    if (!_uart.commandMode)
    {
        return true;
    }

    if (!sendCommand(CMD_EXIT_CONFIG, sizeof(CMD_EXIT_CONFIG)))
    {
        return false;
    }

    _uart.commandMode = false;
    waitFor(LD2410_COMMAND_DELAY);
    return true;
}

bool LD2410::enableEngineeringMode()
{
    if (!enterConfigMode())
    {
        return false;
    }

    bool success = sendCommand(CMD_ENABLE_ENGINEERING, sizeof(CMD_ENABLE_ENGINEERING));

    if (!exitConfigMode())
    {
        return false;
    }

    return success;
}

bool LD2410::disableEngineeringMode()
{
    if (!enterConfigMode())
    {
        return false;
    }

    bool success = sendCommand(CMD_DISABLE_ENGINEERING, sizeof(CMD_DISABLE_ENGINEERING));

    if (!exitConfigMode())
    {
        return false;
    }

    return success;
}

bool LD2410::setMaxValues(uint8_t movingGate, uint8_t stationaryGate, uint16_t timeout)
{
    if (!validateGate(movingGate) || !validateGate(stationaryGate))
    {
        setError(Error::INVALID_PARAMETER);
        return false;
    }

    if (!enterConfigMode())
    {
        return false;
    }

    uint8_t cmd[] = {
        0x14, 0x00, // Command length
        0x60, 0x00, // Command type
        0x00, 0x00, // Moving gate command
        0x00, 0x00, // Moving gate value
        0x00, 0x00, // Spacer
        0x01, 0x00, // Stationary gate command
        0x00, 0x00, // Stationary gate value
        0x00, 0x00, // Spacer
        0x02, 0x00, // Timeout command
        0x00, 0x00, // Timeout value
        0x00, 0x00  // Spacer
    };

    cmd[6] = movingGate;
    cmd[12] = stationaryGate;
    cmd[18] = timeout & 0xFF;
    cmd[19] = (timeout >> 8) & 0xFF;

    bool success = sendCommand(cmd, sizeof(cmd));

    if (!exitConfigMode())
    {
        return false;
    }

    return success;
}

bool LD2410::setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary)
{
    if (!validateGate(gate) ||
        !validateSensitivity(moving) ||
        !validateSensitivity(stationary))
    {
        setError(Error::INVALID_PARAMETER);
        return false;
    }

    if (!enterConfigMode())
    {
        return false;
    }

    uint8_t cmd[] = {
        0x14, 0x00, // Command length
        0x64, 0x00, // Command type
        0x00, 0x00, // Gate command
        0x00, 0x00, // Gate number
        0x00, 0x00, // Spacer
        0x01, 0x00, // Moving sensitivity command
        0x00, 0x00, // Moving sensitivity value
        0x00, 0x00, // Spacer
        0x02, 0x00, // Stationary sensitivity command
        0x00, 0x00, // Stationary sensitivity value
        0x00, 0x00  // Spacer
    };

    cmd[6] = gate;
    cmd[12] = moving;
    cmd[18] = stationary;

    bool success = sendCommand(cmd, sizeof(cmd));

    if (!exitConfigMode())
    {
        return false;
    }

    return success;
}

bool LD2410::restart()
{
    if (!enterConfigMode())
    {
        return false;
    }

    return sendCommand(CMD_RESTART, sizeof(CMD_RESTART));

    // No need to exit config mode as device will restart
}

bool LD2410::factoryReset()
{
    if (!enterConfigMode())
    {
        return false;
    }

    bool success = sendCommand(CMD_FACTORY_RESET, sizeof(CMD_FACTORY_RESET));

    if (!exitConfigMode())
    {
        return false;
    }

    return success;
}

//=====================================================================================================================
// PUBLIC UTILITY FUNCTIONS
//=====================================================================================================================

void LD2410::prettyPrintData(Stream &output)
{
    output.println();
    output.println("--------------------------------------------------");
    output.println("LD2410 Sensor Data");
    output.println("--------------------------------------------------");

    output.print("Target State: ");
    switch (_currentData.targetState)
    {
    case TargetState::NO_TARGET:
        output.println("No Target");
        break;
    case TargetState::MOVING:
        output.println("Moving Target");
        break;
    case TargetState::STATIONARY:
        output.println("Stationary Target");
        break;
    case TargetState::MOVING_AND_STATIONARY:
        output.println("Moving & Stationary Target");
        break;
    }

    output.print("Moving Target - Distance: ");
    output.print(_currentData.movingTargetDistance);
    output.print(" cm, Energy: ");
    output.println(_currentData.movingTargetEnergy);

    output.print("Stationary Target - Distance: ");
    output.print(_currentData.stationaryTargetDistance);
    output.print(" cm, Energy: ");
    output.println(_currentData.stationaryTargetEnergy);

    output.print("Detection Distance: ");
    output.print(_currentData.detectionDistance);
    output.println(" cm");

    if (_uart.isEngineeringMode)
    {
        output.println("\nEngineering Mode Data:");
        output.println("Moving Energy Gates:");
        for (int i = 0; i < LD2410_MAX_GATES; i++)
        {
            output.print("Gate ");
            output.print(i);
            output.print(": ");
            output.println(_engineeringData.movingEnergyGates[i]);
        }

        output.println("\nStationary Energy Gates:");
        for (int i = 0; i < LD2410_MAX_GATES; i++)
        {
            output.print("Gate ");
            output.print(i);
            output.print(": ");
            output.println(_engineeringData.stationaryEnergyGates[i]);
        }

        output.print("\nLight Sensor Value: ");
        output.println(_engineeringData.lightSensorValue);

        output.print("OUT Pin State: ");
        output.println(_engineeringData.outPinState ? "Occupied" : "Unoccupied");
    }

    output.println("--------------------------------------------------");
    output.println();
}

const char *LD2410::getErrorString() const
{
    switch (_lastError)
    {
    case Error::NONE:
        return "No error";
    case Error::BUFFER_OVERFLOW:
        return "Buffer overflow";
    case Error::INVALID_FRAME:
        return "Invalid frame";
    case Error::COMMAND_FAILED:
        return "Command failed";
    case Error::INVALID_PARAMETER:
        return "Invalid parameter";
    case Error::TIMEOUT:
        return "Timeout";
    case Error::NOT_INITIALIZED:
        return "Not initialized";
    default:
        return "Unknown error";
    }
}

//=====================================================================================================================
// PRIVATE UTILITY FUNCTIONS
//=====================================================================================================================

void LD2410::waitFor(unsigned long ms)
{
    unsigned long start = millis();
    while (millis() - start < ms)
    {
        loop();  // Continue processing data while waiting
        yield(); // Allow other tasks to run
    }
}

bool LD2410::validateGate(uint8_t gate) const
{
    return gate <= LD2410_MAX_GATES;
}

bool LD2410::validateSensitivity(uint8_t sensitivity) const
{
    return sensitivity <= LD2410_MAX_SENSITIVITY;
}

void LD2410::setError(Error error)
{
    _lastError = error;
    if (_debug_serial)
    {
        debugPrint("Error: ");
        debugPrintln(getErrorString());
    }
}
