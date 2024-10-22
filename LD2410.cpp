#include "LD2410.h"

LD2410 *LD2410::_instance = nullptr;

LD2410::LD2410() : _digitalOutputPin(255), _outputCallback(nullptr), _debugSerial(nullptr), _serial(nullptr), _ringBufferHead(0), _ringBufferTail(0)
{
    _instance = this;
}

//=====================================================================================================================
// DEBUGGING
//=====================================================================================================================

void LD2410::useDebug(Stream &debugSerial)
{
    _debugSerial = &debugSerial;
    debugPrintln("[LD2410] Debug mode enabled");
}

void LD2410::debugPrint(const char *message) const
{
    if (_debugSerial)
    {
        _debugSerial->print(message);
    }
}

void LD2410::debugPrintln(const char *message) const
{
    if (_debugSerial)
    {
        _debugSerial->println(message);
    }
}

void LD2410::debugPrintBuffer(const uint8_t *buffer, size_t length) const
{
    if (_debugSerial)
    {
        _debugSerial->print("[LD2410] Serial Buffer: ");
        for (size_t i = 0; i < length; i++)
        {
            _debugSerial->print(buffer[i], HEX);
            _debugSerial->print(" ");
        }
        _debugSerial->println();
    }
}

//=====================================================================================================================
// OUTPUT PIN OBSERVATION
//=====================================================================================================================

bool LD2410::beginOutputObservation(uint8_t pin, void (*callback)(bool), uint8_t pinModeOption)
{
    _digitalOutputPin = pin;
    _outputCallback = callback;

    if (_outputCallback == nullptr)
    {
        debugPrintln("[LD2410] Callback can't be null. Output observation not started");
        return false;
    }

    pinMode(_digitalOutputPin, pinModeOption);
    attachInterruptArg(digitalPinToInterrupt(_digitalOutputPin), digitalOutputInterrupt, this, CHANGE);
    debugPrintln("[LD2410] Output observation started successfully");

    return true;
}

void IRAM_ATTR LD2410::digitalOutputInterrupt(void *arg)
{
    LD2410 *instance = static_cast<LD2410 *>(arg);
    if (instance && instance->_outputCallback)
    {
        bool state = digitalRead(instance->_digitalOutputPin) == HIGH;
        instance->_outputCallback(state);
    }
}
//=====================================================================================================================
// UART COMMUNICATION
//=====================================================================================================================

bool LD2410::beginUART(int rxPin, int txPin, HardwareSerial &serial, unsigned long baud)
{
    if (baud != 9600 && baud != 19200 && baud != 38400 && baud != 57600 && baud != 115200 && baud != 230400 && baud != 256000 && baud != 460800)
    {
        debugPrintln("[LD2410] Baud rate is not supported. UART initialization failed");
        return false;
    }

    _serial = &serial;
    _serial->begin(baud, SERIAL_8N1, rxPin, txPin);

    delay(500);

    if (_serial->available())
    {
        debugPrintln("[LD2410] UART initialized successfully");
        return true;
    }
    else
    {
        debugPrintln("[LD2410] Failed to initialize UART");
        return false;
    }
}

void LD2410::readUART()
{
    // Read all available bytes from the serial buffer
    while (_serial->available() && ringBufferAvailable() > 0)
    {
        ringBufferWrite(_serial->read());
    }

    // Parse all available messages
    while (findMessageStart())
    {
        size_t messageLength;
        if (findMessageEnd(messageLength))
        {
            uint8_t message[messageLength];
            for (size_t i = 0; i < messageLength; i++)
            {
                message[i] = ringBufferRead();
            }
            parseMessage(message, messageLength);
        }
        else
        {
            break; // Incomplete message, wait for more data
        }
    }
}

bool LD2410::findMessageStart()
{
    while (ringBufferAvailable() >= 4)
    {
        const bool dataMessage = ringBufferRead() == 0xFD && ringBufferRead() == 0xFC && ringBufferRead() == 0xFB && ringBufferRead() == 0xFA;
        const bool ackMessage = ringBufferRead() == 0xF4 && ringBufferRead() == 0xF3 && ringBufferRead() == 0xF2 && ringBufferRead() == 0xF1;

        if (dataMessage || ackMessage)
        {
            _isDataFrame = (ringBufferRead() == 0xF4);
            return true;
        }
    }
    return false;
}

bool LD2410::findMessageEnd(size_t &messageLength)
{
    if (ringBufferAvailable() < 6) // 2 for length, 4 for footer
    {
        return false;
    }

    uint16_t length = ringBufferRead() | (ringBufferRead() << 8);

    if (ringBufferAvailable() < length + 4) // 4 bytes for end frame
    {
        return false;
    }

    size_t endIndex = (_ringBufferTail + length) % RING_BUFFER_SIZE;

    const bool dataEnd = _ringBuffer[endIndex] == 0xF8 &&
                         _ringBuffer[(endIndex + 1) % RING_BUFFER_SIZE] == 0xF7 &&
                         _ringBuffer[(endIndex + 2) % RING_BUFFER_SIZE] == 0xF6 &&
                         _ringBuffer[(endIndex + 3) % RING_BUFFER_SIZE] == 0xF5;

    const bool ackEnd = _ringBuffer[endIndex] == 0x04 &&
                        _ringBuffer[(endIndex + 1) % RING_BUFFER_SIZE] == 0x03 &&
                        _ringBuffer[(endIndex + 2) % RING_BUFFER_SIZE] == 0x02 &&
                        _ringBuffer[(endIndex + 3) % RING_BUFFER_SIZE] == 0x01;

    if ((dataEnd && _isDataFrame) || (ackEnd && !_isDataFrame))
    {
        messageLength = length + 10; // 4 start + 2 length + length + 4 end
        return true;
    }

    return false;
}

void LD2410::parseMessage(const uint8_t *message, size_t length)
{
    debugPrintBuffer(message, length);
    // TODO: Implement protocol-specific parsing here
}

void LD2410::ringBufferWrite(uint8_t data)
{
    _ringBuffer[_ringBufferHead] = data;
    _ringBufferHead = (_ringBufferHead + 1) % RING_BUFFER_SIZE;
    if (_ringBufferHead == _ringBufferTail)
    {
        _ringBufferTail = (_ringBufferTail + 1) % RING_BUFFER_SIZE;
    }
}

uint8_t LD2410::ringBufferRead()
{
    if (_ringBufferHead == _ringBufferTail)
    {
        return 0; // Buffer is empty
    }
    uint8_t data = _ringBuffer[_ringBufferTail];
    _ringBufferTail = (_ringBufferTail + 1) % RING_BUFFER_SIZE;
    return data;
}

size_t LD2410::ringBufferAvailable() const
{
    return (_ringBufferHead - _ringBufferTail + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
}

void LD2410::ringBufferClear()
{
    _ringBufferHead = _ringBufferTail = 0;
}
