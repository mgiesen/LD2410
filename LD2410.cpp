#include "LD2410.h"

LD2410 *LD2410::_instance = nullptr;

LD2410::LD2410() : _digital_output_pin(255), _output_callback(nullptr), _debug_serial(nullptr), _serial(nullptr)
{
    _instance = this;
}

//=====================================================================================================================
// DEBUGGING
//=====================================================================================================================

void LD2410::useDebug(Stream &debug_serial)
{
    _debug_serial = &debug_serial;
    debugPrintln("[LD2410] Debug mode enabled");
}

void LD2410::debugPrint(const char *message) const
{
    if (_debug_serial)
    {
        _debug_serial->print(message);
    }
}

void LD2410::debugPrintln(const char *message) const
{
    if (_debug_serial)
    {
        _debug_serial->println(message);
    }
}

void LD2410::debugPrintBuffer(const char *messageHeader, const uint8_t *buffer, size_t length) const
{
    if (_debug_serial)
    {
        _debug_serial->print(messageHeader);
        for (size_t i = 0; i < length; i++)
        {
            _debug_serial->print(buffer[i], HEX);
            _debug_serial->print(" ");
        }
        _debug_serial->println();
    }
}

//=====================================================================================================================
// OUTPUT PIN OBSERVATION
//=====================================================================================================================

bool LD2410::beginOutputObservation(uint8_t pin, void (*callback)(bool), uint8_t pin_mode_option)
{
    _digital_output_pin = pin;
    _output_callback = callback;

    if (_output_callback == nullptr)
    {
        debugPrintln("[LD2410] Callback can't be null. Output observation not started");
        return false;
    }

    pinMode(_digital_output_pin, pin_mode_option);
    attachInterruptArg(digitalPinToInterrupt(_digital_output_pin), digitalOutputInterrupt, this, CHANGE);
    debugPrintln("[LD2410] Output observation started successfully");

    return true;
}

void IRAM_ATTR LD2410::digitalOutputInterrupt(void *arg)
{
    LD2410 *instance = static_cast<LD2410 *>(arg);
    if (instance && instance->_output_callback)
    {
        bool state = digitalRead(instance->_digital_output_pin) == HIGH;
        instance->_output_callback(state);
    }
}

//=====================================================================================================================
// UART COMMUNICATION
//=====================================================================================================================

bool LD2410::beginUART(uint8_t ld2410_rx_pin, uint8_t ld2410_tx_pin, HardwareSerial &serial, unsigned long baud)
{
    if (baud != 9600 && baud != 19200 && baud != 38400 && baud != 57600 &&
        baud != 115200 && baud != 230400 && baud != 256000 && baud != 460800)
    {
        debugPrintln("[LD2410] Baud rate is not supported. UART initialization failed");
        return false;
    }

    _serial = &serial;
    _serial->begin(baud, SERIAL_8N1, ld2410_tx_pin, ld2410_rx_pin);

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

void LD2410::printSerialMessage()
{
    bool new_data = false;

    while (_serial->available())
    {
        addToBuffer(_serial->read());
        new_data = true;
    }

    if (new_data)
    {
        if (readFrame())
        {
            debugPrintBuffer("[LD2410] Received frame: ", _radar_data_frame, _radar_data_frame_position);
        }
    }
}

void LD2410::processAckFrame()
{
    uint16_t command = _radar_data_frame[6] | (_radar_data_frame[7] << 8);
    uint16_t status = _radar_data_frame[9] | (_radar_data_frame[10] << 8);

    switch (command)
    {
    }
}

void LD2410::processSensorDataFrame()
{
    // 0x01 --> Engineering mode
    // 0x02 --> Basic target information

    if (_radar_data_frame[6] == 0x02 || _radar_data_frame[6] == 0x01)
    {
        // Check if the data header of the intra-frame data is valid
        if (_radar_data_frame[7] == 0xAA)
        {
            // Basic target information
            _target_state = _radar_data_frame[8];
            _moving_target_distance = _radar_data_frame[9] | (_radar_data_frame[10] << 8);
            _moving_target_energy = _radar_data_frame[11];
            _stationary_target_distance = _radar_data_frame[12] | (_radar_data_frame[13] << 8);
            _stationary_target_energy = _radar_data_frame[14];
            _detection_distance = _radar_data_frame[15] | (_radar_data_frame[16] << 8);

            // Engineering mode additional data
            if (_radar_data_frame[6] == 0x01)
            {
                _max_moving_gate = _radar_data_frame[17];
                _max_stationary_gate = _radar_data_frame[18];

                // Process moving target energy gates
                for (int i = 0; i < 9; i++)
                {
                    _moving_energy_gates[i] = _radar_data_frame[19 + i];
                }

                // Process stationary target energy gates
                for (int i = 0; i < 9; i++)
                {
                    _stationary_energy_gates[i] = _radar_data_frame[28 + i];
                }

                _light_sensor_value = _radar_data_frame[37];
                _out_pin_state = _radar_data_frame[38] == 0x01;
            }
        }
    }
}

void LD2410::proessSerialMessages()
{
    bool new_data = false;

    while (_serial->available())
    {
        addToBuffer(_serial->read());
        new_data = true;
    }

    if (new_data)
    {
        if (readFrame())
        {
            if (_ack_frame)
            {
                processAckFrame();
            }
            else
            {
                processSensorDataFrame();
            }
        }
    }
}

void LD2410::addToBuffer(uint8_t byte)
{
    _circular_buffer[_buffer_head] = byte;
    _buffer_head = (_buffer_head + 1) % LD2410_BUFFER_SIZE;

    if (_buffer_head == _buffer_tail)
    {
        _buffer_tail = (_buffer_tail + 1) % LD2410_BUFFER_SIZE;
    }
}

bool LD2410::readFromBuffer(uint8_t &byte)
{
    if (_buffer_head == _buffer_tail)
    {
        return false;
    }
    else
    {
        byte = _circular_buffer[_buffer_tail];
        _buffer_tail = (_buffer_tail + 1) % LD2410_BUFFER_SIZE;
        return true;
    }
}

bool LD2410::findFrameStart()
{
    uint8_t byte;
    while (readFromBuffer(byte))
    {
        if (byte == 0xF4 || byte == 0xFD)
        {
            _radar_data_frame[0] = byte;
            _radar_data_frame_position = 1;
            _frame_started = true;
            _ack_frame = (byte == 0xFD);
            return true;
        }
    }
    return false;
}

bool LD2410::checkFrameEnd()
{
    if (_ack_frame)
    {
        return (_radar_data_frame[0] == 0xFD &&
                _radar_data_frame[1] == 0xFC &&
                _radar_data_frame[2] == 0xFB &&
                _radar_data_frame[3] == 0xFA &&
                _radar_data_frame[_radar_data_frame_position - 4] == 0x04 &&
                _radar_data_frame[_radar_data_frame_position - 3] == 0x03 &&
                _radar_data_frame[_radar_data_frame_position - 2] == 0x02 &&
                _radar_data_frame[_radar_data_frame_position - 1] == 0x01);
    }
    else
    {
        return (_radar_data_frame[0] == 0xF4 &&
                _radar_data_frame[1] == 0xF3 &&
                _radar_data_frame[2] == 0xF2 &&
                _radar_data_frame[3] == 0xF1 &&
                _radar_data_frame[_radar_data_frame_position - 4] == 0xF8 &&
                _radar_data_frame[_radar_data_frame_position - 3] == 0xF7 &&
                _radar_data_frame[_radar_data_frame_position - 2] == 0xF6 &&
                _radar_data_frame[_radar_data_frame_position - 1] == 0xF5);
    }
}

bool LD2410::readFrame()
{
    uint8_t byte_read;
    while (readFromBuffer(byte_read))
    {
        if (!_frame_started)
        {
            if (byte_read == 0xF4 || byte_read == 0xFD)
            {
                _radar_data_frame[0] = byte_read;
                _radar_data_frame_position = 1;
                _frame_started = true;
                _ack_frame = (byte_read == 0xFD);
            }
        }
        else
        {
            _radar_data_frame[_radar_data_frame_position++] = byte_read;

            if (_radar_data_frame_position == 8)
            {
                uint16_t intra_frame_data_length = _radar_data_frame[4] | (_radar_data_frame[5] << 8);

                if (intra_frame_data_length + 10 > LD2410_MAX_FRAME_LENGTH)
                {
                    _frame_started = false;
                    _radar_data_frame_position = 0;
                    continue;
                }
            }

            if (_radar_data_frame_position >= 8 && checkFrameEnd())
            {
                _frame_started = false;
                return true;
            }
        }
    }
    return false;
}
