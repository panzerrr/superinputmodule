#ifndef MODBUS_HANDLER_H
#define MODBUS_HANDLER_H

#include <ModbusRTU.h>

// Modbus configuration
#define SLAVE_ID 0x01       // Device Address
#define BAUDRATE 19200      // Serial Bit Rate
#define PARITY SERIAL_8E1   // 8 data bits, Even parity, 1 stop bit
#define MODBUS_TX_PIN 17    // GPIO 17 for Modbus TX
#define MODBUS_RX_PIN 16    // GPIO 16 for Modbus RX
#define TXEN_PIN -1         // Not used in RS-232 or USB-Serial

// Data type enumeration
enum DataType {
    TYPE_U64,
    TYPE_FLOAT,
    TYPE_INT16
}; // Construct our data type, as I checked the excel only found these 3

// Modbus instance
extern ModbusRTU mb;

// Current slave ID (can be changed dynamically)
extern uint8_t currentSlaveID;

// System mode tracking
enum SystemMode {
    MODE_ANALOG,    // Analog output mode (default)
    MODE_MODBUS     // Modbus slave mode
};

extern SystemMode currentMode;

// Store previous analog values when entering modbus mode
extern float previousVoltageValues[3];
extern float previousCurrentValues[3];
extern char previousSignalModes[3];
extern bool valuesStored;

// Utility functions
uint16_t lowWord(uint32_t dword);
uint16_t highWord(uint32_t dword);

// Initialize Modbus
void initModbus();

// Mode management
void enterModbusMode(uint8_t slaveID);
void exitModbusMode();
bool isModbusModeActive();

// Set slave ID
void setSlaveID(uint8_t slaveID);


// Process measurement values in sequence: flow, consumption, reverse consumption, flow direction
void processMeasurementValues(float flow, uint32_t consumption, uint32_t reverseConsumption, uint32_t flowDirection);

// Process different data types
void processU64(uint16_t regn, uint64_t data);
void processUint32(uint16_t regn, uint32_t data);
void processFloat(uint16_t regn, float data);
void processInt16(uint16_t regn, int16_t data);

// Process measurement channels
void setFlowValue(float flow);
void setConsumptionValue(uint32_t consumption);
void setReverseConsumptionValue(uint32_t reverseConsumption);
void setFlowDirectionValue(uint32_t direction);

// Set all DAC outputs to zero when Modbus is activated
void setAllDACsToZero();

// Turn off all relays to isolate outputs when Modbus is activated
void turnOffAllRelays();

// Store current analog values before entering modbus mode
void storeAnalogValues();

// Restore analog values when exiting modbus mode
void restoreAnalogValues();

#endif // MODBUS_HANDLER_H
