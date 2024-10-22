#ifndef LD2410_H
#define LD2410_H

#include <Arduino.h>

class LD2410
{
public:
    LD2410();
    void useDebug(Stream &debugSerial);
    bool beginUART(int rxPin, int txPin, HardwareSerial &serial, unsigned long baud = 256000);
    bool beginOutputObservation(uint8_t pin, void (*callback)(bool), uint8_t pinModeOption = INPUT);
    void readUART();

private:
    static const size_t RING_BUFFER_SIZE = 256;
    uint8_t _ringBuffer[RING_BUFFER_SIZE];
    size_t _ringBufferHead;
    size_t _ringBufferTail;

    Stream *_debugSerial;
    HardwareSerial *_serial;
    uint8_t _digitalOutputPin;
    void (*_outputCallback)(bool);
    static LD2410 *_instance;
    bool _isDataFrame;

    static void IRAM_ATTR digitalOutputInterrupt(void *arg);

    bool findMessageStart();
    bool findMessageEnd(size_t &messageLength);
    void parseMessage(const uint8_t *message, size_t length);

    void debugPrint(const char *message) const;
    void debugPrintln(const char *message) const;
    void debugPrintBuffer(const uint8_t *buffer, size_t length) const;

    // Ring buffer methods
    void ringBufferWrite(uint8_t data);
    uint8_t ringBufferRead();
    size_t ringBufferAvailable() const;
    void ringBufferClear();
};

#endif // LD2410_H