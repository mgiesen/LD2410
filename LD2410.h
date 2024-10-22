#ifndef LD2410_H
#define LD2410_H

#include <Arduino.h>

#define LD2410_BUFFER_SIZE 1024
#define LD2410_MAX_FRAME_LENGTH 1024

class LD2410
{
public:
    LD2410();
    void useDebug(Stream &debug_serial);
    bool beginUART(uint8_t ld2410_rx_pin, uint8_t ld2410_tx_pin, HardwareSerial &serial, unsigned long baud = 256000);
    bool beginOutputObservation(uint8_t pin, void (*callback)(bool), uint8_t pin_mode_option = INPUT);
    void proessSerialMessages();
    void printSerialMessage();

private:
    //-------------------------------------------------------------------------------------------------
    // Singleton Setup
    //-------------------------------------------------------------------------------------------------
    static LD2410 *_instance;

    //-------------------------------------------------------------------------------------------------
    // UART
    //-------------------------------------------------------------------------------------------------
    HardwareSerial *_serial;

    uint8_t _circular_buffer[LD2410_BUFFER_SIZE];
    uint16_t _buffer_head = 0;
    uint16_t _buffer_tail = 0;

    uint8_t _radar_data_frame[LD2410_MAX_FRAME_LENGTH];
    uint16_t _radar_data_frame_position = 0;
    bool _frame_started = false;
    bool _ack_frame = false;

    void addToBuffer(uint8_t byte);
    bool readFromBuffer(uint8_t &byte);
    bool findFrameStart();
    bool checkFrameEnd();
    bool readFrame();

    //-------------------------------------------------------------------------------------------------
    // Debugging
    //-------------------------------------------------------------------------------------------------
    Stream *_debug_serial;
    void debugPrint(const char *message) const;
    void debugPrintln(const char *message) const;
    void debugPrintBuffer(const char *messageHeader, const uint8_t *buffer, size_t length) const;

    //-------------------------------------------------------------------------------------------------
    // Output observation
    //-------------------------------------------------------------------------------------------------
    uint8_t _digital_output_pin;
    void (*_output_callback)(bool);
    static void IRAM_ATTR digitalOutputInterrupt(void *arg);
};

#endif // LD2410_H