/*
 * LD2410.cpp
 *
 * Author: Maximilian Giesen
 * Version: UNRELEASED
 * Repository: https://github.com/mgiesen/ld2410
 *
 * A streamlined library for the LD2410 sensor, providing a high-level interface for UART communication and output observation, with minimal user code required.
 */

#include "LD2410.h"

//===========================================
// CircularBuffer Implementation
//===========================================

LD2410::CircularBuffer::CircularBuffer() : _head(0), _tail(0) {}

bool LD2410::CircularBuffer::add(uint8_t byte)
{
    uint16_t nextHead = (_head + 1) % LD2410_BUFFER_SIZE;
    if (nextHead == _tail)
    {
        return false; // Buffer full
    }
    _buffer[_head] = byte;
    _head = nextHead;
    return true;
}

bool LD2410::CircularBuffer::read(uint8_t &byte)
{
    if (_head == _tail)
    {
        return false; // Buffer empty
    }
    byte = _buffer[_tail];
    _tail = (_tail + 1) % LD2410_BUFFER_SIZE;
    return true;
}

void LD2410::CircularBuffer::clear()
{
    _head = _tail = 0;
}

bool LD2410::CircularBuffer::isEmpty() const
{
    return _head == _tail;
}

//===========================================
// FrameProcessor
//===========================================

LD2410::FrameProcessor::FrameProcessor() : _position(0), _started(false), _isAckFrame(false) {}

bool LD2410::FrameProcessor::findFrameStart(CircularBuffer &buffer)
{
    uint8_t byte;
    while (buffer.read(byte))
    {
        if (byte == 0xF4 || byte == 0xFD)
        {
            _frame[0] = byte;
            _position = 1;
            _started = true;
            _isAckFrame = (byte == 0xFD);
            return true;
        }
    }
    return false;
}

bool LD2410::FrameProcessor::processFrame(CircularBuffer &buffer)
{
    if (!_started && !findFrameStart(buffer))
    {
        return false;
    }

    uint8_t byte;
    while (buffer.read(byte))
    {
        _frame[_position++] = byte;

        // Check frame length
        if (_position == 8)
        {
            uint16_t frameLength = _frame[4] | (_frame[5] << 8);
            if (frameLength + 10 > LD2410_MAX_FRAME_LENGTH)
            {
                reset();
                return false;
            }
        }

        // Validate frame end
        if (_position >= 8)
        {
            if (_isAckFrame)
            {
                if (_frame[_position - 4] == 0x04 &&
                    _frame[_position - 3] == 0x03 &&
                    _frame[_position - 2] == 0x02 &&
                    _frame[_position - 1] == 0x01)
                {
                    _started = false;
                    return validateFrame();
                }
            }
            else
            {
                if (_frame[_position - 4] == 0xF8 &&
                    _frame[_position - 3] == 0xF7 &&
                    _frame[_position - 2] == 0xF6 &&
                    _frame[_position - 1] == 0xF5)
                {
                    _started = false;
                    return validateFrame();
                }
            }
        }

        if (_position >= LD2410_MAX_FRAME_LENGTH)
        {
            reset();
            return false;
        }
    }

    return false;
}

void LD2410::FrameProcessor::reset()
{
    _position = 0;
    _started = false;
}

bool LD2410::FrameProcessor::validateFrame() const
{
    return _position >= 8;
}

bool LD2410::FrameProcessor::isAckFrame() const
{
    return _isAckFrame;
}

const uint8_t *LD2410::FrameProcessor::getFrameData() const
{
    return _frame;
}

uint16_t LD2410::FrameProcessor::getFrameLength() const
{
    return _position;
}

void LD2410::FrameProcessor::parseDataFrame(BasicData &data, EngineeringData &engData)
{
    if (_frame[6] != 0x01 && _frame[6] != 0x02)
    {
        return;
    }

    bool isEngineeringMode = (_frame[6] == 0x01);

    if (_frame[7] == 0xAA)
    {
        // Parse basic data
        data.targetState = static_cast<TargetState>(_frame[8]);
        data.movingTargetDistance = _frame[9] | (_frame[10] << 8);
        data.movingTargetEnergy = _frame[11];
        data.stationaryTargetDistance = _frame[12] | (_frame[13] << 8);
        data.stationaryTargetEnergy = _frame[14];
        data.detectionDistance = _frame[15] | (_frame[16] << 8);
        data.lastSensorDataUpdate = millis();

        if (isEngineeringMode)
        {
            // Parse additional engineering data
            engData.maxMovingGate = _frame[17];
            engData.maxStationaryGate = _frame[18];

            for (int i = 0; i < LD2410_MAX_GATES; i++)
            {
                engData.movingEnergyGates[i] = _frame[19 + i];
                engData.stationaryEnergyGates[i] = _frame[28 + i];
            }

            engData.lastEngineeringDataUpdate = millis();
        }
    }
}

//===========================================
// CommandManager
//===========================================

LD2410::CommandManager::CommandManager()
    : _parent(nullptr), _inConfigMode(false)
{
}

void LD2410::CommandManager::init(LD2410 *parent)
{
    _parent = parent;
}

bool LD2410::CommandManager::isInConfigMode() const
{
    return _inConfigMode;
}

void LD2410::CommandManager::sendCommand(HardwareSerial &serial, const uint8_t *cmd, size_t length)
{
    serial.write(cmd, length);
    serial.flush();
}

bool LD2410::CommandManager::enterConfigMode(HardwareSerial &serial)
{
    if (_inConfigMode)
        return true;

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x04, 0x00,
        0xFF, 0x00,
        0x01, 0x00,
        0x04, 0x03, 0x02, 0x01};

    sendCommand(serial, cmd, sizeof(cmd));

    // Wait for ACK and check if successful
    bool success = waitForAck(serial, 0xFF);

    if (success)
    {
        _inConfigMode = true;
        delay(LD2410_CONFIG_DELAY); // Delay still needed after successful config mode entry

        // Clear any pending data
        while (serial.available())
        {
            serial.read();
        }
    }

    return success;
}

bool LD2410::CommandManager::exitConfigMode(HardwareSerial &serial)
{
    if (!_inConfigMode)
        return true;

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0xFE, 0x00,
        0x04, 0x03, 0x02, 0x01};

    sendCommand(serial, cmd, sizeof(cmd));

    // Wait for ACK and check if successful
    bool success = waitForAck(serial, 0xFE);

    if (success)
    {
        _inConfigMode = false;
    }

    return success;
}

bool LD2410::CommandManager::validateResponse(const uint8_t *response, uint16_t expectedCommand) const
{
    // Check if header is correct
    if (response[0] != 0xFD || response[1] != 0xFC || response[2] != 0xFB || response[3] != 0xFA)
    {
        return false;
    }

    // Get frame length from response
    uint16_t frameLength = response[4] | (response[5] << 8);

    // Check minimum length (header + length + command + status + tail = 12 bytes)
    if (frameLength < 2 || frameLength > LD2410_MAX_FRAME_LENGTH - 10)
    {
        return false;
    }

    // Check if the command matches the expected command
    if (response[6] != (expectedCommand & 0xFF))
    {
        return false;
    }

    // Check if tail is at expected position based on frame length
    size_t tailPos = frameLength + 6;
    if (response[tailPos] != 0x04 || response[tailPos + 1] != 0x03 || response[tailPos + 2] != 0x02 || response[tailPos + 3] != 0x01)
    {
        return false;
    }

    return true;
}

bool LD2410::CommandManager::waitForAck(HardwareSerial &serial, uint16_t expectedCommand, uint8_t *response)
{
    unsigned long start = millis();
    uint8_t buffer[LD2410_MAX_FRAME_LENGTH];
    size_t pos = 0;
    bool headerFound = false;

    while (millis() - start < LD2410_COMMAND_TIMEOUT)
    {
        if (serial.available())
        {
            uint8_t byte = serial.read();

            // Search for the header (FD FC FB FA)
            if (!headerFound && byte == 0xFD)
            {
                buffer[pos++] = byte;
                headerFound = true;
                continue;
            }

            // After header is found, keep adding bytes
            if (headerFound && pos < LD2410_MAX_FRAME_LENGTH)
            {
                buffer[pos++] = byte;

                // Once we have length bytes (6 bytes including header)
                if (pos >= 6)
                {
                    uint16_t expectedLength = buffer[4] | (buffer[5] << 8);
                    size_t expectedTotalLength = expectedLength + 10; // Frame length + header(4) + length(2) + tail(4)

                    // Check if we have received the complete frame
                    if (pos >= expectedTotalLength)
                    {
                        if (validateResponse(buffer, expectedCommand))
                        {
                            // If caller wants the response and it's valid, copy it
                            if (response)
                            {
                                memcpy(response, buffer, expectedTotalLength);
                            }

                            // Return true if the ACK Status is successful
                            return buffer[8] == 0x00;
                        }
                        else
                        {
                            if (_parent)
                            {
                                _parent->debugPrintHex("[LD2410 DEBUGGER] Invalid ACK-Response: ", buffer, pos);
                            }
                            return false;
                        }
                    }
                }
            }
        }

        if (pos >= LD2410_MAX_FRAME_LENGTH)
        {
            break;
        }

        yield();
    }

    if (_parent && _parent->_debug_serial)
    {
        _parent->debugPrintHex("[LD2410 DEBUGGER] Timeout or invalid ACK-Response: ", buffer, pos);
    }

    return false;
}

//===========================================
// General
//===========================================

LD2410::LD2410() : _debug_serial(nullptr), _lastError(Error::NONE)
{
    _uart.init(this);
    memset(&_basicData, 0, sizeof(_basicData));
    memset(&_engineeringData, 0, sizeof(_engineeringData));
    memset(&_outputObservation, 0, sizeof(_outputObservation));
}

bool LD2410::beginUART(uint8_t mcu_rx_pin, uint8_t mcu_tx_pin, HardwareSerial &serial, unsigned long baud)
{
    _uart.serial = &serial;
    _uart.serial->begin(baud, SERIAL_8N1, mcu_rx_pin, mcu_tx_pin);

    delay(500); // Wait for serial connection to stabilize

    unsigned long start = millis();
    while (millis() - start < 1000)
    {
        if (_uart.serial->available())
        {
            debugPrintln("[LD2410 DEBUGGER] UART initialized successfully");
            _uart.initialized = true;
            return true;
        }
        delay(10);
    }

    debugPrintln("[LD2410 DEBUGGER] Failed to initialize UART");
    _uart.initialized = false;
    return false;
}

bool LD2410::beginOutputObservation(uint8_t pin, void (*callback)(bool), uint8_t pin_mode_option)
{
    if (callback == nullptr)
    {
        debugPrintln("[LD2410 DEBUGGER] Callback can't be null. Output observation not started");
        return false;
    }

    // Disable existing interrupt if any
    if (_outputObservation.started)
    {
        debugPrintln("[LD2410 DEBUGGER] Output observation already started. Reassigning callback...");
        detachInterrupt(digitalPinToInterrupt(_outputObservation.pin));
    }

    _outputObservation.pin = pin;
    _outputObservation.callback = callback;
    _outputObservation.started = false;
    _outputObservation.lastState = false;

    pinMode(pin, pin_mode_option);
    _outputObservation.lastState = digitalRead(pin) == HIGH;

    attachInterruptArg(digitalPinToInterrupt(pin), digitalOutputInterrupt, this, CHANGE);

    _outputObservation.started = true;
    debugPrintln("[LD2410 DEBUGGER] Output observation started successfully");
    return true;
}

void LD2410::readSensorData(uint8_t maxBytesPerLoop)
{
    if (!_uart.initialized)
    {
        return;
    }

    uint8_t bytesProcessed = 0;

    // Process incoming data
    while (_uart.serial->available() && bytesProcessed < maxBytesPerLoop)
    {
        uint8_t byte = _uart.serial->read();
        if (!_uart.buffer.add(byte))
        {
            setError(Error::BUFFER_OVERFLOW);
            return;
        }
        bytesProcessed++;
    }

    // Process buffered data
    if (_uart.frameProcessor.processFrame(_uart.buffer))
    {
        if (_uart.frameProcessor.isAckFrame())
        {
            debugPrintHex("[LD2410 DEBUGGER] Got unhandled ACK Frame: ", _uart.frameProcessor.getFrameData(), _uart.frameProcessor.getFrameLength());
        }
        else
        {
            _uart.frameProcessor.parseDataFrame(_basicData, _engineeringData);
            _uart.isEngineeringMode = (_uart.frameProcessor.getFrameData()[6] == 0x01);
        }
    }
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

void LD2410::setError(Error error)
{
    _lastError = error;
    if (_debug_serial)
    {
        debugPrint("Error: ");
        debugPrintln(getLastErrorString());
    }
}

void LD2410::useDebug(Stream &debugSerial)
{
    _debug_serial = &debugSerial;
    debugPrintln("Debug mode enabled");
}

const char *LD2410::getLastErrorString() const
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

//===========================================
// LD2410 Commands
//===========================================

bool LD2410::setMaxValues(uint8_t movingGate, uint8_t stationaryGate, uint16_t timeout)
{
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return false;
    }

    if (!validateGate(movingGate) || !validateGate(stationaryGate))
    {
        setError(Error::INVALID_PARAMETER);
        return false;
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        setError(Error::COMMAND_FAILED);
        return false;
    }

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x14, 0x00,
        0x60, 0x00,
        0x00, 0x00,
        movingGate, 0x00,
        0x00, 0x00,
        0x01, 0x00,
        stationaryGate, 0x00,
        0x00, 0x00,
        0x02, 0x00,
        (uint8_t)(timeout & 0xFF), (uint8_t)((timeout >> 8) & 0xFF),
        0x00, 0x00,
        0x04, 0x03, 0x02, 0x01};

    _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    bool success = _uart.commandManager.waitForAck(*_uart.serial, 0x60, nullptr);
    _uart.commandManager.exitConfigMode(*_uart.serial);

    if (!success)
    {
        setError(Error::COMMAND_FAILED);
    }

    return success;
}

bool LD2410::setGateSensitivityThreshold(uint8_t gate, uint8_t moving, uint8_t stationary)
{
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return false;
    }

    if (!validateGate(gate) || !validateSensitivity(moving) || !validateSensitivity(stationary))
    {
        setError(Error::INVALID_PARAMETER);
        return false;
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        setError(Error::COMMAND_FAILED);
        return false;
    }

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x14, 0x00,
        0x64, 0x00,
        0x00, 0x00,
        gate, 0x00,
        0x00, 0x00,
        0x01, 0x00,
        moving, 0x00,
        0x00, 0x00,
        0x02, 0x00,
        stationary, 0x00,
        0x00, 0x00,
        0x04, 0x03, 0x02, 0x01};

    _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    bool success = _uart.commandManager.waitForAck(*_uart.serial, 0x64, nullptr);
    _uart.commandManager.exitConfigMode(*_uart.serial);

    if (!success)
    {
        setError(Error::COMMAND_FAILED);
    }

    return success;
}

bool LD2410::enableEngineeringMode()
{
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return false;
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        setError(Error::COMMAND_FAILED);
        return false;
    }

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0x62, 0x00,
        0x04, 0x03, 0x02, 0x01};

    _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    bool success = _uart.commandManager.waitForAck(*_uart.serial, 0x62, nullptr);

    _uart.commandManager.exitConfigMode(*_uart.serial);

    if (success)
    {
        _uart.isEngineeringMode = true;
        debugPrintln("[LD2410 DEBUGGER] Engineering mode enabled");
    }
    else
    {
        setError(Error::COMMAND_FAILED);
    }

    return success;
}

bool LD2410::disableEngineeringMode()
{
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return false;
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        setError(Error::COMMAND_FAILED);
        return false;
    }

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0x63, 0x00,
        0x04, 0x03, 0x02, 0x01};

    _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    bool success = _uart.commandManager.waitForAck(*_uart.serial, 0x63, nullptr);

    _uart.commandManager.exitConfigMode(*_uart.serial);

    if (success)
    {
        _uart.isEngineeringMode = false;
        debugPrintln("[LD2410 DEBUGGER] Engineering mode disabled");
    }
    else
    {
        setError(Error::COMMAND_FAILED);
    }

    return success;
}

bool LD2410::setBaudRate(unsigned long baudRate)
{
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return false;
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        setError(Error::COMMAND_FAILED);
        return false;
    }

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x04, 0x00,
        0xA1, 0x00,
        (uint8_t)(baudRate & 0xFF),
        (uint8_t)((baudRate >> 8) & 0xFF),
        0x04, 0x03, 0x02, 0x01};

    _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    bool success = _uart.commandManager.waitForAck(*_uart.serial, 0xA1, nullptr);
    _uart.commandManager.exitConfigMode(*_uart.serial);

    if (!success)
    {
        setError(Error::COMMAND_FAILED);
    }

    return success;
}

bool LD2410::setDistanceResolution(bool use020mResolution)
{
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return false;
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        setError(Error::COMMAND_FAILED);
        return false;
    }

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0xAA, 0x00,
        (uint8_t)(use020mResolution ? 0x01 : 0x00),
        0x04, 0x03, 0x02, 0x01};

    _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    bool success = _uart.commandManager.waitForAck(*_uart.serial, 0xAA, nullptr);
    _uart.commandManager.exitConfigMode(*_uart.serial);

    if (!success)
    {
        setError(Error::COMMAND_FAILED);
    }

    return success;
}

bool LD2410::factoryReset()
{
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return false;
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        setError(Error::COMMAND_FAILED);
        return false;
    }

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0xA2, 0x00,
        0x04, 0x03, 0x02, 0x01};

    _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    bool success = _uart.commandManager.waitForAck(*_uart.serial, 0xA2, nullptr);
    _uart.commandManager.exitConfigMode(*_uart.serial);

    if (!success)
    {
        setError(Error::COMMAND_FAILED);
    }

    return success;
}

bool LD2410::restart()
{
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return false;
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        setError(Error::COMMAND_FAILED);
        return false;
    }

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0xA3, 0x00,
        0x04, 0x03, 0x02, 0x01};

    _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    bool success = _uart.commandManager.waitForAck(*_uart.serial, 0xA3, nullptr);

    if (!success)
    {
        setError(Error::COMMAND_FAILED);
    }

    return success;
    // No exitConfigMode needed as device will restart
}

bool LD2410::readConfiguration()
{
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return false;
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        setError(Error::COMMAND_FAILED);
        return false;
    }

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0x61, 0x00,
        0x04, 0x03, 0x02, 0x01};

    _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    uint8_t response[64];
    if (_uart.commandManager.waitForAck(*_uart.serial, 0x61, response))
    {
        if (response[10] == 0xAA)
        {
            _currentConfig.maxDistanceGate = response[11];
            _currentConfig.configuredMaxMotionGate = response[12];
            _currentConfig.configuredMaxStationaryGate = response[13];

            for (int i = 0; i < 9; i++)
            {
                _currentConfig.motionSensitivity[i] = response[14 + i];
                _currentConfig.stationarySensitivity[i] = response[23 + i];
            }

            _currentConfig.noOccupancyDuration = response[32] | (response[33] << 8);
            _currentConfig.lastConfigurationgDataUpdate = millis();

            _uart.commandManager.exitConfigMode(*_uart.serial);
            return true;
        }
    }

    _uart.commandManager.exitConfigMode(*_uart.serial);
    setError(Error::COMMAND_FAILED);
    return false;
}

String LD2410::getMacAddress()
{
    /*
     * Returns a 6-byte identifier in MAC address format (XX:XX:XX:XX:XX:XX)
     *
     * Note: There are some inconsistencies regarding this function:
     * 1. The manufacturer's documentation mentions a 3-byte address in big endian,
     *    but the actual response contains 6 bytes
     * 2. The returned bytes often contain patterns similar to the frame end bytes
     * 3. It's unclear if this is actually a MAC address or just a device identifier
     *
     * This implementation follows the community standard (as used in albertnis/ld2410-configurator) taking bytes 10-15 from the response.
     */
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return "Invalid";
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        setError(Error::COMMAND_FAILED);
        return "Invalid";
    }

    const uint8_t macCmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x04, 0x00,
        0xA5, 0x00,
        0x01, 0x00,
        0x04, 0x03, 0x02, 0x01};

    _uart.commandManager.sendCommand(*_uart.serial, macCmd, sizeof(macCmd));

    uint8_t response[32];
    if (_uart.commandManager.waitForAck(*_uart.serial, 0xA5, response))
    {
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 response[10], response[11], response[12],
                 response[13], response[14], response[15]);

        _uart.commandManager.exitConfigMode(*_uart.serial);
        return String(macStr);
    }

    _uart.commandManager.exitConfigMode(*_uart.serial);
    setError(Error::TIMEOUT);
    return "Invalid";
}

String LD2410::getFirmwareVersion()
{
    if (!_uart.initialized)
    {
        setError(Error::NOT_INITIALIZED);
        return "Invalid";
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        setError(Error::COMMAND_FAILED);
        return "Invalid";
    }

    const uint8_t cmd[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0xA0, 0x00,
        0x04, 0x03, 0x02, 0x01};

    _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    uint8_t response[32];
    if (_uart.commandManager.waitForAck(*_uart.serial, 0xA0, response))
    {
        uint8_t majorVersion = response[13];
        uint8_t minorVersion = response[12];
        uint8_t buildHigh = response[14];
        uint8_t buildLow = response[15];

        char versionStr[32];
        snprintf(versionStr, sizeof(versionStr), "V%d.%02d.%d%02d",
                 majorVersion,
                 minorVersion,
                 buildHigh,
                 buildLow);

        _uart.commandManager.exitConfigMode(*_uart.serial);
        return String(versionStr);
    }

    _uart.commandManager.exitConfigMode(*_uart.serial);
    setError(Error::TIMEOUT);
    return "Invalid";
}

//===========================================
// Utility Functions
//===========================================

bool LD2410::validateGate(uint8_t gate) const
{
    return gate <= LD2410_MAX_GATES;
}

bool LD2410::validateSensitivity(uint8_t sensitivity) const
{
    return sensitivity <= LD2410_MAX_SENSITIVITY;
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

void LD2410::debugPrintHex(const char *prefix, const uint8_t *data, uint16_t length)
{
    if (_debug_serial)
    {
        _debug_serial->print(prefix);
        for (uint16_t i = 0; i < length; i++)
        {
            if (data[i] < 0x10)
                _debug_serial->print('0');
            _debug_serial->print(data[i], HEX);
            _debug_serial->print(' ');
        }
        _debug_serial->println();
    }
}