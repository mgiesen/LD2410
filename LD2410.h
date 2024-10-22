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

    bool beginOutputObservation(uint8_t pin, void (*callback)(bool), uint8_t pin_mode_option = INPUT);

    bool beginUART(uint8_t ld2410_rx_pin, uint8_t ld2410_tx_pin, HardwareSerial &serial, unsigned long baud = 256000);
    void proessSerialMessages();
    void printSerialMessage();

    bool isEngineeringMode() const { return _is_engineering_mode; }

    // Basic target information
    uint8_t getTargetState() const { return _target_state; }
    uint16_t getMovingTargetDistance() const { return _moving_target_distance; }
    uint8_t getMovingTargetEnergy() const { return _moving_target_energy; }
    uint16_t getStationaryTargetDistance() const { return _stationary_target_distance; }
    uint8_t getStationaryTargetEnergy() const { return _stationary_target_energy; }
    uint16_t getDetectionDistance() const { return _detection_distance; }

    // Engineering mode additional data
    uint8_t getMaxMovingGate() const { return _max_moving_gate; }
    uint8_t getMaxStationaryGate() const { return _max_stationary_gate; }
    uint8_t getMovingEnergyGate(uint8_t gate) const { return gate < 9 ? _moving_energy_gates[gate] : 0; }
    uint8_t getStationaryEnergyGate(uint8_t gate) const { return gate < 9 ? _stationary_energy_gates[gate] : 0; }
    uint8_t getLightSensorValue() const { return _light_sensor_value; }
    bool getOutPinState() const { return _out_pin_state; }

private:
    //-------------------------------------------------------------------------------------------------
    // Singleton Setup
    //-------------------------------------------------------------------------------------------------

    static LD2410 *_instance;

    //-------------------------------------------------------------------------------------------------
    // Sensor Data
    //-------------------------------------------------------------------------------------------------

    uint8_t _target_state;
    uint16_t _moving_target_distance;
    uint8_t _moving_target_energy;
    uint16_t _stationary_target_distance;
    uint8_t _stationary_target_energy;
    uint16_t _detection_distance;
    bool _is_engineering_mode;
    uint8_t _max_moving_gate;
    uint8_t _max_stationary_gate;
    uint8_t _moving_energy_gates[9];     // 0-8 gates
    uint8_t _stationary_energy_gates[9]; // 0-8 gates
    uint8_t _light_sensor_value;
    bool _out_pin_state;

    //-------------------------------------------------------------------------------------------------
    // Frame Processing
    //-------------------------------------------------------------------------------------------------

    void processAckFrame();
    void processSensorDataFrame();

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
