#include "modbus_handler.h"
#include "dac_controller.h"
#include "sine_wave_generator.h"
#include "relay_controller.h"
#include "command_handler.h"
#include "utils.h"

// Modbus instance
ModbusRTU mb;

// Current slave ID (can be changed dynamically)
uint8_t currentSlaveID = SLAVE_ID;

// System mode tracking
SystemMode currentMode = MODE_ANALOG;

// Store previous analog values when entering modbus mode
float previousVoltageValues[3] = {0.0f, 0.0f, 0.0f};
float previousCurrentValues[3] = {0.0f, 0.0f, 0.0f};
char previousSignalModes[3] = {'v', 'v', 'v'};
bool valuesStored = false;

uint16_t lowWord(uint32_t dword) {
    return (uint16_t)(dword & 0xFFFF);
}

uint16_t highWord(uint32_t dword) {
    return (uint16_t)(dword >> 16);
}

void initModbus() {
    // Initialize Serial1 with explicit pin configuration (like working code)
    Serial1.begin(BAUDRATE, PARITY, MODBUS_RX_PIN, MODBUS_TX_PIN);
    delay(100); // Give Serial1 time to initialize
    
    // Initialize Modbus with Serial1 (like working code)
    mb.begin(&Serial1);
    mb.slave(currentSlaveID);
    
    Serial.printf("Modbus interface initialized: RX=GPIO%d, TX=GPIO%d, Baud=%d, Parity=8E1\n", 
                  MODBUS_RX_PIN, MODBUS_TX_PIN, BAUDRATE);
    Serial.println("System is in ANALOG mode by default.");
    Serial.println("Use 'modbus <slave_id>' to enter Modbus mode and disable analog outputs.");
}

void enterModbusMode(uint8_t slaveID) {
    if (slaveID >= 1 && slaveID <= 247) {
        currentMode = MODE_MODBUS;
        currentSlaveID = slaveID;
        mb.slave(currentSlaveID);
        
        // Clear the signal arrays in main.cpp to reset all analog inputs
        extern char signalModes[3];
        extern float signalValues[3];
        for (int i = 0; i < 3; i++) {
            signalModes[i] = 'v';  // Reset to default voltage mode
            signalValues[i] = 0.0f; // Reset to 0
        }
        
        // Turn off all analog outputs and isolate with relays
        setAllDACsToZero();
        turnOffAllRelays();
        
        Serial.println("=== MODBUS MODE ACTIVATED ===");
        Serial.printf("Slave ID: %d\n", currentSlaveID);
        Serial.println("All analog outputs have been disabled and isolated.");
        Serial.println("");
        Serial.println("Please input measurement data in the following order:");
        Serial.println("1. Flow value (float, resolution 0.1)");
        Serial.println("2. Consumption value (integer, resolution 1)");
        Serial.println("3. Reverse consumption value (integer, resolution 1)");
        Serial.println("4. Flow direction (0=same direction, 1=reverse direction)");
        Serial.println("");
        Serial.println("Use: measure <flow> <consumption> <reverse> <direction>");
        Serial.println("Example: measure 12.5 50000 2500 0");
        Serial.println("");
        Serial.println("Or use individual commands:");
        Serial.println("  flow <value>");
        Serial.println("  consumption <value>");
        Serial.println("  reverse <value>");
        Serial.println("  direction <0|1>");
        Serial.println("");
        Serial.println("Use 'exit_modbus' to return to analog mode.");
    } else {
        Serial.println("Invalid slave ID. Must be between 1 and 247.");
    }
}

void exitModbusMode() {
    if (currentMode == MODE_MODBUS) {
        currentMode = MODE_ANALOG;
        
        // Don't restore previous values - keep everything at 0
        // User can manually set new values if needed
        
        Serial.println("=== ANALOG MODE ACTIVATED ===");
        Serial.println("Modbus mode disabled. Analog outputs and relays are now available.");
        Serial.println("All analog outputs remain at 0. Set new values manually if needed.");
        Serial.println("Use 'modbus <slave_id>' to re-enter Modbus mode.");
    } else {
        Serial.println("System is already in Analog mode.");
        Serial.println("Use 'modbus <slave_id>' to enter Modbus mode.");
    }
}

bool isModbusModeActive() {
    return (currentMode == MODE_MODBUS);
}

void setSlaveID(uint8_t slaveID) {
    if (slaveID >= 1 && slaveID <= 247) {
        currentSlaveID = slaveID;
        mb.slave(currentSlaveID);
        Serial.printf("Slave ID changed to: %d\n", currentSlaveID);
    } else {
        Serial.println("Invalid slave ID. Must be between 1 and 247.");
    }
}

void processMeasurementValues(float flow, uint32_t consumption, uint32_t reverseConsumption, uint32_t flowDirection) {
    // Set all measurement values in sequence
    setFlowValue(flow);
    setConsumptionValue(consumption);
    setReverseConsumptionValue(reverseConsumption);
    setFlowDirectionValue(flowDirection);
    
    Serial.println("All measurement values updated:");
    Serial.printf("  Flow: %.1f (Register 6)\n", flow);
    Serial.printf("  Consumption: %u (Register 8)\n", consumption);
    Serial.printf("  Reverse Consumption: %u (Register 14)\n", reverseConsumption);
    Serial.printf("  Flow Direction: %u (Register 42)\n", flowDirection);
}


void processU64(uint16_t regn, uint64_t data) {
    mb.addHreg(regn, 0x00, 4);   // Add 4 registers for 64-bit data
    
    // For UINT64, we need to split into 4 16-bit registers
    // Following the 1-0-3-2 byte order pattern for each 32-bit half
    uint32_t low32 = (uint32_t)(data & 0xFFFFFFFF);
    uint32_t high32 = (uint32_t)((data >> 32) & 0xFFFFFFFF);
    
    // Process low 32-bit part with 1-0-3-2 byte order
    uint8_t byte0 = (low32 >> 0) & 0xFF;
    uint8_t byte1 = (low32 >> 8) & 0xFF;
    uint8_t byte2 = (low32 >> 16) & 0xFF;
    uint8_t byte3 = (low32 >> 24) & 0xFF;
    
    uint16_t reg1 = (byte1 << 8) | byte0;  // Byte 1-0
    uint16_t reg2 = (byte3 << 8) | byte2;  // Byte 3-2
    
    // Process high 32-bit part with 1-0-3-2 byte order
    uint8_t byte4 = (high32 >> 0) & 0xFF;
    uint8_t byte5 = (high32 >> 8) & 0xFF;
    uint8_t byte6 = (high32 >> 16) & 0xFF;
    uint8_t byte7 = (high32 >> 24) & 0xFF;
    
    uint16_t reg3 = (byte5 << 8) | byte4;  // Byte 1-0
    uint16_t reg4 = (byte7 << 8) | byte6;  // Byte 3-2
    
    mb.Hreg(regn, reg1);
    mb.Hreg(regn + 1, reg2);
    mb.Hreg(regn + 2, reg3);
    mb.Hreg(regn + 3, reg4);
}

void processUint32(uint16_t regn, uint32_t data) {
    mb.addHreg(regn, 0x00, 2);   // Add 2 registers for 32-bit data
    
    // UINT32 byte order: 1-0-3-2
    // Original: Byte 0, Byte 1, Byte 2, Byte 3
    // Device:   Byte 1, Byte 0, Byte 3, Byte 2
    uint8_t byte0 = (data >> 0) & 0xFF;
    uint8_t byte1 = (data >> 8) & 0xFF;
    uint8_t byte2 = (data >> 16) & 0xFF;
    uint8_t byte3 = (data >> 24) & 0xFF;
    
    // Reorder to 1-0-3-2
    uint16_t reg1 = (byte1 << 8) | byte0;  // Byte 1-0
    uint16_t reg2 = (byte3 << 8) | byte2;  // Byte 3-2
    
    mb.Hreg(regn, reg1);
    mb.Hreg(regn + 1, reg2);
}

void processFloat(uint16_t regn, float data) {
    uint32_t asInt = *(uint32_t*)&data;
    mb.addHreg(regn, 0x00, 2);
    
    // FLOAT byte order: 1-0-3-2
    // Original: Byte 0, Byte 1, Byte 2, Byte 3
    // Device:   Byte 1, Byte 0, Byte 3, Byte 2
    uint8_t byte0 = (asInt >> 0) & 0xFF;
    uint8_t byte1 = (asInt >> 8) & 0xFF;
    uint8_t byte2 = (asInt >> 16) & 0xFF;
    uint8_t byte3 = (asInt >> 24) & 0xFF;
    
    // Reorder to 1-0-3-2
    uint16_t reg1 = (byte1 << 8) | byte0;  // Byte 1-0
    uint16_t reg2 = (byte3 << 8) | byte2;  // Byte 3-2
    
    mb.Hreg(regn, reg1);
    mb.Hreg(regn + 1, reg2);
}

void processInt16(uint16_t regn, int16_t data) {
    mb.addHreg(regn, 0x00, 1);   // Only need 1 register for 16-bit data
    
    // UINT16/INT16 byte order: 1-0
    // Original: Byte 0, Byte 1
    // Device:   Byte 1, Byte 0
    uint8_t byte0 = (uint16_t)data & 0xFF;
    uint8_t byte1 = ((uint16_t)data >> 8) & 0xFF;
    
    // Reorder to 1-0 (Byte 1 MSB, Byte 0 LSB)
    uint16_t reg1 = (byte1 << 8) | byte0;
    
    mb.Hreg(regn, reg1);
}

/**
 * Set all DAC outputs to zero when Modbus is activated
 */
void setAllDACsToZero() {
    // Stop all sine wave generation
    stopSineWave(0); // Stop all channels
    
    // Set all voltage DACs to 0
    gp8413_1.setVoltage(0.0, 0); // SIG1 voltage channel
    gp8413_1.setVoltage(0.0, 1); // SIG2 voltage channel
    gp8413_2.setVoltage(0.0, 0); // SIG3 voltage channel
    
    // Set all current DACs to 0
    gp8313_1.setDACOutElectricCurrent(0); // SIG1 current channel
    gp8313_2.setDACOutElectricCurrent(0); // SIG2 current channel
    gp8313_3.setDACOutElectricCurrent(0); // SIG3 current channel
    
    Serial.println("All DAC outputs set to 0V/0mA");
}

/**
 * Turn off all relays to isolate outputs when Modbus is activated
 */
void turnOffAllRelays() {
    // Turn off all 6 relays (HIGH = OFF for these relays)
    setRelay(1, false); // SIG1 current relay
    setRelay(2, false); // SIG1 voltage relay
    setRelay(3, false); // SIG2 current relay
    setRelay(4, false); // SIG2 voltage relay
    setRelay(5, false); // SIG3 current relay
    setRelay(6, false); // SIG3 voltage relay
    
    Serial.println("All relays turned OFF - outputs isolated");
}

/**
 * Store current analog values before entering modbus mode
 */
void storeAnalogValues() {
    // Store current signal modes and values from main.cpp
    extern char signalModes[3];
    extern float signalValues[3];
    
    for (int i = 0; i < 3; i++) {
        previousSignalModes[i] = signalModes[i];
        if (signalModes[i] == 'v') {
            previousVoltageValues[i] = signalValues[i];
            previousCurrentValues[i] = 0.0f; // Not used in voltage mode
        } else if (signalModes[i] == 'c') {
            previousCurrentValues[i] = signalValues[i];
            previousVoltageValues[i] = 0.0f; // Not used in current mode
        }
        
        // Debug output
        const char* modeStr = (signalModes[i] == 'v') ? "voltage" : "current";
        const char* unit = (signalModes[i] == 'v') ? "V" : "mA";
        float value = (signalModes[i] == 'v') ? signalValues[i] : signalValues[i];
        Serial.printf("Storing SIG%d: %s mode, %.2f%s\n", i + 1, modeStr, value, unit);
    }
    valuesStored = true;
    Serial.println("Analog values stored before entering Modbus mode");
}

/**
 * Restore analog values when exiting modbus mode
 */
void restoreAnalogValues() {
    if (!valuesStored) {
        Serial.println("No stored analog values to restore");
        return;
    }
    
    // Restore signal modes and values to main.cpp
    extern char signalModes[3];
    extern float signalValues[3];
    extern SignalMap signalMap[3];
    
    for (int i = 0; i < 3; i++) {
        signalModes[i] = previousSignalModes[i];
        if (previousSignalModes[i] == 'v') {
            signalValues[i] = previousVoltageValues[i];
        } else if (previousSignalModes[i] == 'c') {
            signalValues[i] = previousCurrentValues[i];
        }
        
        // Restore relay mode and DAC output
        if (previousSignalModes[i] == 'v' || previousSignalModes[i] == 'c') {
            setRelayMode(i + 1, previousSignalModes[i]);
            
            // Restore DAC output
            if (previousSignalModes[i] == 'v') {
                signalMap[i].voltageDAC->setVoltage(previousVoltageValues[i], signalMap[i].voltageChannel);
            } else if (previousSignalModes[i] == 'c') {
                signalMap[i].currentDAC->setDACOutElectricCurrent(static_cast<uint16_t>(previousCurrentValues[i] * 1310.68));
            }
            
            const char* modeStr = (previousSignalModes[i] == 'v') ? "voltage" : "current";
            const char* unit = (previousSignalModes[i] == 'v') ? "V" : "mA";
            float restoredValue = (previousSignalModes[i] == 'v') ? previousVoltageValues[i] : previousCurrentValues[i];
            Serial.printf("Restored SIG%d: %s mode, %.2f%s\n", i + 1, modeStr, restoredValue, unit);
        }
    }
    
    valuesStored = false;
    Serial.println("Analog values restored from Modbus mode");
}

/**
 * Measurement channel functions - Set values for specific measurement channels
 * Based on the device manual specification
 */

// Flow measurement (Register 6, FLOAT, Resolution 0.1)
void setFlowValue(float flow) {
    processFloat(6, flow);
    Serial.printf("Flow set to %.1f (Register 6)\n", flow);
}

// Consumption measurement (Register 8, UNIT32, Resolution 1)
void setConsumptionValue(uint32_t consumption) {
    processUint32(8, consumption);
    Serial.printf("Consumption set to %u (Register 8)\n", consumption);
}

// Reverse consumption measurement (Register 14, UNIT32, Resolution 1)
void setReverseConsumptionValue(uint32_t reverseConsumption) {
    processUint32(14, reverseConsumption);
    Serial.printf("Reverse consumption set to %u (Register 14)\n", reverseConsumption);
}

// Flow direction indication (Register 42, UNIT32, Resolution 1)
// Value 0 = same direction, Value 1 = reverse direction
void setFlowDirectionValue(uint32_t direction) {
    processUint32(42, direction);
    const char* dirStr = (direction == 0) ? "same direction" : "reverse direction";
    Serial.printf("Flow direction set to %u (%s) (Register 42)\n", direction, dirStr);
}
