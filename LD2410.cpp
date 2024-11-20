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
// FrameProcessor Implementation
//===========================================
LD2410::FrameProcessor::FrameProcessor()
    : _position(0), _started(false), _isAckFrame(false) {}

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
            // Copy basic data fields to engineering data
            engData.targetState = data.targetState;
            engData.movingTargetDistance = data.movingTargetDistance;
            engData.movingTargetEnergy = data.movingTargetEnergy;
            engData.stationaryTargetDistance = data.stationaryTargetDistance;
            engData.stationaryTargetEnergy = data.stationaryTargetEnergy;
            engData.detectionDistance = data.detectionDistance;

            // Parse additional engineering data
            engData.maxMovingGate = _frame[17];
            engData.maxStationaryGate = _frame[18];

            for (int i = 0; i < LD2410_MAX_GATES; i++)
            {
                engData.movingEnergyGates[i] = _frame[19 + i];
                engData.stationaryEnergyGates[i] = _frame[28 + i];
            }

            engData.lightSensorValue = _frame[37];
            engData.outPinState = _frame[38] != 0;
            engData.lastEngineeringDataUpdate = millis();
        }
    }
}

bool LD2410::FrameProcessor::parseCommandFrame(uint16_t expectedCommand)
{
    uint16_t receivedCommand = (_frame[7] << 8) | _frame[6];
    if (receivedCommand != expectedCommand)
    {
        return false;
    }

    uint16_t status = (_frame[9] << 8) | _frame[8];
    return status == 0;
}

//===========================================
// CommandManager Implementation
//===========================================
LD2410::CommandManager::CommandManager() : _inConfigMode(false) {}

bool LD2410::CommandManager::sendCommand(HardwareSerial &serial, const uint8_t *cmd, size_t length)
{
    const uint8_t header[] = {0xFD, 0xFC, 0xFB, 0xFA};
    const uint8_t footer[] = {0x04, 0x03, 0x02, 0x01};

    serial.write(header, 4);
    serial.write(cmd, length);
    serial.write(footer, 4);
    serial.flush();

    return true; // For now: Asume, that the command was sent successfully
}

// To-Do: Implement function into sendCommand, to validate sucess
bool LD2410::CommandManager::waitForAck(HardwareSerial &serial, CircularBuffer &buffer, FrameProcessor &processor, uint16_t expectedCommand)
{
    unsigned long start = millis();

    while (millis() - start < LD2410_COMMAND_TIMEOUT)
    {
        if (serial.available())
        {
            if (!buffer.add(serial.read()))
            {
                return false;
            }
        }

        if (processor.processFrame(buffer) && processor.isAckFrame())
        {
            return processor.parseCommandFrame(expectedCommand);
        }
    }

    return false;
}

bool LD2410::CommandManager::enterConfigMode(HardwareSerial &serial)
{
    if (_inConfigMode)
    {
        return true;
    }

    const uint8_t cmd[] = {0x04, 0x00, 0xFF, 0x00, 0x01, 0x00};
    if (!sendCommand(serial, cmd, sizeof(cmd)))
    {
        return false;
    }

    _inConfigMode = true;
    delay(LD2410_COMMAND_DELAY);
    return true;
}

bool LD2410::CommandManager::exitConfigMode(HardwareSerial &serial)
{
    if (!_inConfigMode)
    {
        return true;
    }

    const uint8_t cmd[] = {0x02, 0x00, 0xFE, 0x00};
    if (!sendCommand(serial, cmd, sizeof(cmd)))
    {
        return false;
    }

    _inConfigMode = false;
    delay(LD2410_COMMAND_DELAY);
    return true;
}

//===========================================
// LD2410 Main Class Implementation
//===========================================

LD2410::LD2410() : _debug_serial(nullptr), _lastError(Error::NONE)
{
    _uart.init();
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

    _outputObservation.pin = pin;
    _outputObservation.callback = callback;
    _outputObservation.started = false;
    _outputObservation.lastState = false;

    pinMode(pin, pin_mode_option);
    _outputObservation.lastState = digitalRead(pin) == HIGH;

    attachInterruptArg(digitalPinToInterrupt(pin), digitalOutputInterrupt, this, CHANGE);

    _outputObservation.started = true;
    debugPrintln("[LD2410] Output observation started successfully");
    return true;
}

void LD2410::processUART(uint8_t maxBytesPerLoop)
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
            // debugPrintHex("ACK FRAME: ", _uart.frameProcessor.getFrameData(), _uart.frameProcessor.getFrameLength());
        }
        else
        {
            _uart.frameProcessor.parseDataFrame(_basicData, _engineeringData);
            _uart.isEngineeringMode = (_uart.frameProcessor.getFrameData()[6] == 0x01);
        }
    }
}

bool LD2410::setMaxValues(uint8_t movingGate, uint8_t stationaryGate, uint16_t timeout)
{
    if (!validateGate(movingGate) || !validateGate(stationaryGate))
    {
        setError(Error::INVALID_PARAMETER);
        return false;
    }

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        return false;
    }

    uint8_t cmd[] = {
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
        0x00, 0x00};

    bool success = _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    if (!_uart.commandManager.exitConfigMode(*_uart.serial))
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

    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        return false;
    }

    uint8_t cmd[] = {
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
        0x00, 0x00};

    bool success = _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    if (!_uart.commandManager.exitConfigMode(*_uart.serial))
    {
        return false;
    }

    return success;
}

bool LD2410::enableEngineeringMode()
{
    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        return false;
    }

    const uint8_t cmd[] = {0x02, 0x00, 0x62, 0x00};
    bool success = _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    if (!_uart.commandManager.exitConfigMode(*_uart.serial))
    {
        return false;
    }

    if (success)
    {
        _uart.isEngineeringMode = true;
        debugPrintln("[LD2410] Engineering mode enabled");
    }

    return success;
}

bool LD2410::disableEngineeringMode()
{
    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        return false;
    }

    const uint8_t cmd[] = {0x02, 0x00, 0x63, 0x00};
    bool success = _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    if (!_uart.commandManager.exitConfigMode(*_uart.serial))
    {
        return false;
    }

    if (success)
    {

        debugPrintln("[LD2410] Engineering mode disabled");
    }

    return success;
}

bool LD2410::setBaudRate(unsigned long baudRate)
{
    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        return false;
    }

    uint8_t cmd[] = {0x04, 0x00, 0xA1, 0x00,
                     (uint8_t)(baudRate & 0xFF),
                     (uint8_t)((baudRate >> 8) & 0xFF)};

    bool success = _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    if (!_uart.commandManager.exitConfigMode(*_uart.serial))
    {
        return false;
    }

    return success;
}

bool LD2410::setDistanceResolution(bool use020mResolution)
{
    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        return false;
    }

    uint8_t cmd[] = {0x02, 0x00, 0xA0, 0x00,
                     (uint8_t)(use020mResolution ? 0x01 : 0x00)};

    bool success = _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    if (!_uart.commandManager.exitConfigMode(*_uart.serial))
    {
        return false;
    }

    return success;
}

bool LD2410::restart()
{
    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        return false;
    }

    const uint8_t cmd[] = {0x02, 0x00, 0xA3, 0x00};
    return _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));
    // No need to exit config mode as device will restart
}

bool LD2410::factoryReset()
{
    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        return false;
    }

    const uint8_t cmd[] = {0x02, 0x00, 0xA2, 0x00};
    bool success = _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    if (!_uart.commandManager.exitConfigMode(*_uart.serial))
    {
        return false;
    }

    return success;
}

bool LD2410::readConfiguration()
{
    if (!_uart.commandManager.enterConfigMode(*_uart.serial))
    {
        return false;
    }

    const uint8_t cmd[] = {0x02, 0x00, 0x61, 0x00};
    bool success = _uart.commandManager.sendCommand(*_uart.serial, cmd, sizeof(cmd));

    if (!_uart.commandManager.exitConfigMode(*_uart.serial))
    {
        return false;
    }

    return success;
}

bool LD2410::getFirmwareVersion(uint8_t &major, uint8_t &minor, uint16_t &bugfix, uint16_t &build)
{
    if (!_uart.initialized)
    {
        return false;
    }

    uint8_t buffer[64] = {0};
    uint8_t bufferIndex = 0;

    debugPrintln("\n=== Starting Version Query with Endless Listen ===");

    // 1. Enter Config Mode
    const uint8_t configCmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    debugPrintHex("TX CONFIG: ", configCmd, sizeof(configCmd));
    _uart.serial->write(configCmd, sizeof(configCmd));
    _uart.serial->flush();

    debugPrintln("Listening for CONFIG response:");
    unsigned long lastByteTime = millis();
    bufferIndex = 0;

    // Listen bis 2 Sekunden lang keine Daten mehr kommen
    while (true)
    {
        if (_uart.serial->available())
        {
            buffer[bufferIndex++] = _uart.serial->read();
            lastByteTime = millis();

            // Print sofort jedes Byte
            if (_debug_serial)
            {
                _debug_serial->printf("%02X ", buffer[bufferIndex - 1]);
            }
        }

        // Wenn 2 Sekunden keine Daten mehr kamen, weiter zum nächsten Command
        if (millis() - lastByteTime > 2000)
        {
            debugPrintln("\nNo more data for 2 seconds...");
            break;
        }
        yield(); // ESP32 Watchdog füttern
    }

    // 2. Request Version
    delay(100); // Kurze Pause zwischen Commands
    const uint8_t versionCmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA0, 0x00, 0x04, 0x03, 0x02, 0x01};
    debugPrintHex("\nTX VERSION: ", versionCmd, sizeof(versionCmd));
    _uart.serial->write(versionCmd, sizeof(versionCmd));
    _uart.serial->flush();

    debugPrintln("Listening for VERSION response:");
    lastByteTime = millis();
    bufferIndex = 0;

    // Wieder endlos zuhören
    while (true)
    {
        if (_uart.serial->available())
        {
            buffer[bufferIndex++] = _uart.serial->read();
            lastByteTime = millis();

            // Print sofort jedes Byte
            if (_debug_serial)
            {
                _debug_serial->printf("%02X ", buffer[bufferIndex - 1]);
            }
        }

        if (millis() - lastByteTime > 2000)
        {
            debugPrintln("\nNo more data for 2 seconds...");
            break;
        }
        yield();
    }

    // 3. Exit Config Mode
    delay(100);
    const uint8_t exitCmd[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    debugPrintHex("\nTX EXIT: ", exitCmd, sizeof(exitCmd));
    _uart.serial->write(exitCmd, sizeof(exitCmd));
    _uart.serial->flush();

    debugPrintln("Listening for EXIT response:");
    lastByteTime = millis();
    bufferIndex = 0;

    // Ein letztes Mal endlos zuhören
    while (true)
    {
        if (_uart.serial->available())
        {
            buffer[bufferIndex++] = _uart.serial->read();
            lastByteTime = millis();

            // Print sofort jedes Byte
            if (_debug_serial)
            {
                _debug_serial->printf("%02X ", buffer[bufferIndex - 1]);
            }
        }

        if (millis() - lastByteTime > 2000)
        {
            debugPrintln("\nNo more data for 2 seconds...");
            break;
        }
        yield();
    }

    debugPrintln("\n=== End of Version Query ===\n");
    return false;
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

void LD2410::useDebug(Stream &debugSerial)
{
    _debug_serial = &debugSerial;
    debugPrintln("Debug mode enabled");
}

void LD2410::prettyPrintData(Stream &output)
{
    output.println("\n--------------------------------------------------");
    output.println("LD2410 Basic Data");
    output.println("--------------------------------------------------");

    output.print("Sensor Data Current: ");
    output.println(_basicData.isBasicDataCurrent() ? "Yes" : "No");

    output.print("Target State: ");
    switch (_basicData.targetState)
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
    output.print(_basicData.movingTargetDistance);
    output.print(" cm, Energy: ");
    output.println(_basicData.movingTargetEnergy);

    output.print("Stationary Target - Distance: ");
    output.print(_basicData.stationaryTargetDistance);
    output.print(" cm, Energy: ");
    output.println(_basicData.stationaryTargetEnergy);

    output.print("Detection Distance: ");
    output.print(_basicData.detectionDistance);
    output.println(" cm");

    if (_uart.isEngineeringMode)
    {
        output.println("\nEngineering Mode Data:");

        output.print("Engineering Data Current: ");
        output.println(_engineeringData.isEngineeringDataCurrent() ? "Yes" : "No");

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

    output.println("--------------------------------------------------\n");
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

void LD2410::waitFor(unsigned long ms)
{
    unsigned long start = millis();
    while (millis() - start < ms)
    {
        processUART(); // Continue processing data while waiting
        yield();       // Allow other tasks to run
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
        debugPrintln(getLastErrorString());
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
