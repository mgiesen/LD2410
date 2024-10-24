#ifndef LD2410_H
#define LD2410_H

#include <Arduino.h>

#define LD2410_BUFFER_SIZE 1024
#define LD2410_MAX_FRAME_LENGTH 1024

class LD2410
{
public:
    LD2410();

    // Setup functions
    void useDebug(Stream &debug_serial);
    bool beginOutputObservation(uint8_t pin, void (*callback)(bool), uint8_t pin_mode_option = INPUT);
    bool beginUART(uint8_t ld2410_rx_pin, uint8_t ld2410_tx_pin, HardwareSerial &serial, unsigned long baud = 256000);

    // Loop functions
    void processUART();

    // Basic target information Getter
    uint8_t getTargetState() const { return _target_state; }
    uint16_t getMovingTargetDistance() const { return _moving_target_distance; }
    uint8_t getMovingTargetEnergy() const { return _moving_target_energy; }
    uint16_t getStationaryTargetDistance() const { return _stationary_target_distance; }
    uint8_t getStationaryTargetEnergy() const { return _stationary_target_energy; }
    uint16_t getDetectionDistance() const { return _detection_distance; }

    // Engineering mode additional data Getter
    uint8_t getMaxMovingGate() const { return _max_moving_gate; }
    uint8_t getMaxStationaryGate() const { return _max_stationary_gate; }
    uint8_t getMovingEnergyGate(uint8_t gate) const { return gate < 9 ? _moving_energy_gates[gate] : 0; }
    uint8_t getStationaryEnergyGate(uint8_t gate) const { return gate < 9 ? _stationary_energy_gates[gate] : 0; }
    uint8_t getLightSensorValue() const { return _light_sensor_value; }
    bool getOutPinState() const { return _out_pin_state; }
    bool isEngineeringMode() const { return _is_engineering_mode; }

    // Debugging
    void prettyPrintData(Stream &output);

    // Commands Setter and Getter
    bool enableEngineeringMode();
    bool disableEngineeringMode();

    // TO-DO: Basic Configuration Commands
    // bool setMaxDistanceGateAndDuration(uint8_t maxMovingGate, uint8_t maxStationaryGate, uint16_t unoccupiedDuration); // 0x0060
    // bool readParameters();                                                                                             // 0x0061
    // bool setDistanceGateSensitivity(uint8_t gate, uint8_t movingSensitivity, uint8_t stationarySensitivity);           // 0x0064

    // TO-DO: System Commands
    // bool readFirmwareVersion();                // 0x00A0
    // bool setSerialBaudRate(uint32_t baudRate); // 0x00A1
    // bool restoreFactorySettings();             // 0x00A2
    // bool restart();                            // 0x00A3

    // TO-DO: Bluetooth Related Commands
    // bool setBluetoothState(bool enabled);            // 0x00A4
    // bool getMacAddress();                            // 0x00A5
    // bool getBluetoothPermissions();                  // 0x00A8
    // bool setBluetoothPassword(const char *password); // 0x00A9

    // TO-DO: Resolution & Control Commands
    // bool activatePreciseDistanceMode(); // Switches LD2410 to 0,2m resolution (0x01)
    // bool activateExtendedRangeMode(); // Switches LD2410 to 0,75m resolution (0x00)
    // bool queryDistanceResolution();                                             // 0x00AB
    // bool setAuxiliaryControl(uint8_t mode, uint8_t threshold, bool defaultLow); // 0x00AD
    // bool queryAuxiliaryControl();                                               // 0x00AE

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
    uint32_t _radar_uart_last_command_ = 0;

    void addToBuffer(uint8_t byte);
    bool readFromBuffer(uint8_t &byte);
    bool findFrameStart();
    bool checkFrameEnd();
    bool readFrame();
    void parseSensorDataFromFrame();

    //-------------------------------------------------------------------------------------------------
    // Debugging
    //-------------------------------------------------------------------------------------------------

    Stream *_debug_serial;
    void debugPrint(const char *message) const;
    void debugPrintln(const char *message) const;

    //-------------------------------------------------------------------------------------------------
    // Output observation
    //-------------------------------------------------------------------------------------------------

    uint8_t _digital_output_pin;
    void (*_output_callback)(bool);
    static void IRAM_ATTR digitalOutputInterrupt(void *arg);

    //-------------------------------------------------------------------------------------------------
    // Commands
    //-------------------------------------------------------------------------------------------------

    bool sendCommand(const uint8_t *cmd, size_t length);
    bool enterConfigMode();
    bool exitConfigMode();
};

#endif // LD2410_H