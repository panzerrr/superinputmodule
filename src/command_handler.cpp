#include "command_handler.h"
#include "dac_controller.h"
#include "relay_controller.h"
#include "modbus_handler.h"
#include "sine_wave_generator.h"
#include "device_id.h"
#include "rs485_command_handler.h"
#include "utils.h"

// Global variable declarations
extern char signalModes[3]; // Signal modes
extern float signalValues[3]; // Signal values
extern bool signalConfigured[3]; // Signal configuration status

// Global signal mapping table
SignalMap signalMap[3] = {
    {&gp8413_1, 0, &gp8313_1}, // SIG1
    {&gp8413_1, 1, &gp8313_2}, // SIG2
    {&gp8413_2, 0, &gp8313_3}  // SIG3
};

void parseModeCommand(String params) {
    int commaIndex = params.indexOf(',');
    if (commaIndex == -1 || params.length() <= commaIndex + 1) {
        Serial.println("Invalid mode command. Use 'MODE SIG,MODE' (case-insensitive).");
        return;
    }

    int sig = params.substring(0, commaIndex).toInt(); // Get signal number
    char mode = toLowerCase(params.substring(commaIndex + 1).charAt(0)); // Get mode ('v' or 'c') - case insensitive

    if (sig < 1 || sig > 3 || (mode != 'v' && mode != 'c')) {
        Serial.println("Invalid mode. Use 'v' or 'c' (case-insensitive).");
        return;
    }

    // Execute protection operation
    if (mode == 'v') {
        signalMap[sig - 1].currentDAC->setDACOutElectricCurrent(0);
        Serial.printf("SIG%d: Current set to 0mA for protection.\n", sig);
    } else if (mode == 'c') {
        signalMap[sig - 1].voltageDAC->setVoltage(0.0, signalMap[sig - 1].voltageChannel);
        Serial.printf("SIG%d: Voltage set to 0V for protection.\n", sig);
    }

    // Update mode status and set relay
    signalModes[sig - 1] = mode;
    setRelayMode(sig, mode);
    Serial.printf("Mode set: SIG%d -> %c\n", sig, mode);
}

void parseValueCommand(String params) {
    int commaIndex = params.indexOf(',');
    if (commaIndex == -1 || params.length() <= commaIndex + 1) {
        Serial.println("Invalid value command. Use 'VALUE SIG,VALUE' (case-insensitive).");
        return;
    }

    int sig = params.substring(0, commaIndex).toInt();
    float value = params.substring(commaIndex + 1).toFloat();

    if (sig < 1 || sig > 3) {
        Serial.println("Invalid signal number. Use 1 to 3.");
        return;
    }

    char mode = signalModes[sig - 1];
    if (mode == 'v') {
        if (value < 0 || value > 10.0) {
            Serial.println("Invalid voltage value. Use 0-10V.");
            return;
        }
        signalMap[sig - 1].voltageDAC->setVoltage(value, signalMap[sig - 1].voltageChannel);
        Serial.printf("Voltage set: SIG%d -> %.2f V\n", sig, value);
    } else if (mode == 'c') {
        if (value < 0 || value > 25.0) {
            Serial.println("Invalid current value. Use 0-25mA.");
            return;
        }
        // Convert mA to DAC data: Rset=2kΩ, 25mA = 32767 (15-bit), so 1mA = 1310.68
        signalMap[sig - 1].currentDAC->setDACOutElectricCurrent(static_cast<uint16_t>(value * 1310.68));
        Serial.printf("Current set: SIG%d -> %.2f mA\n", sig, value);
    } else {
        Serial.printf("Unknown mode '%c' for SIG%d.\n", mode, sig);
    }
}

/**
 * Main command processor - routes commands to appropriate handlers
 */
void processCommand(String command) {
    // Convert to lowercase for most commands, but keep modbus commands case-sensitive
    String lowerCommand = command;
    lowerCommand.toLowerCase();
    
    // Check current mode and handle commands accordingly
    if (isModbusModeActive()) {
        // In Modbus mode - only allow modbus-related commands
        if (lowerCommand.startsWith("help") || lowerCommand.startsWith("status") || 
            lowerCommand.startsWith("exit_modbus") || lowerCommand.startsWith("modbus") ||
            lowerCommand.startsWith("measure") || lowerCommand.startsWith("flow") ||
            lowerCommand.startsWith("consumption") || lowerCommand.startsWith("reverse") ||
            lowerCommand.startsWith("direction") || lowerCommand.startsWith("slave")) {
            processModbusCommand(command);
        } else {
            Serial.println("Command blocked: System is in Modbus mode.");
            Serial.println("Only Modbus commands are available. Use 'exit_modbus' to return to analog mode.");
        }
    } else {
        // In Analog mode - block modbus commands but allow analog commands
        if (lowerCommand.startsWith("exit_modbus")) {
            processModbusCommand(command);
        } else if (lowerCommand.startsWith("measure") || lowerCommand.startsWith("flow") ||
            lowerCommand.startsWith("consumption") || lowerCommand.startsWith("reverse") ||
            lowerCommand.startsWith("direction") || lowerCommand.startsWith("slave")) {
            Serial.println("Command blocked: System is in Analog mode.");
            Serial.println("Use 'modbus <slave_id>' to enter Modbus mode first.");
        } else if (command.indexOf(',') > 0) {
            processAnalogCommand(command);
        } else if (lowerCommand.startsWith("help") || lowerCommand.startsWith("status")) {
            processSystemCommand(command);
        } else if (lowerCommand.startsWith("modbus_test") || lowerCommand.startsWith("serial_test") || 
                   lowerCommand.startsWith("send_modbus")) {
            processTestCommand(command);
        } else if (lowerCommand.startsWith("modbus")) {
            processModbusCommand(command);
        } else if (lowerCommand.startsWith("sine") || lowerCommand.startsWith("ping") || 
                   lowerCommand.startsWith("test485") || lowerCommand.startsWith("voltage") || 
                   lowerCommand.startsWith("current") || lowerCommand.startsWith("stop")) {
            processSystemCommand(command);
        } else if (command.length() > 0) {
            Serial.println("Unknown command. Type 'help' for available commands.");
        }
    }
}

/**
 * Process analog channel commands (channel,mode,value format)
 */
void processAnalogCommand(String command) {
    // This is an analog channel command: channel,mode,value
    if (!isModbusModeActive()) {
        // Process analog channel command
        int comma1 = command.indexOf(',');
        int comma2 = command.indexOf(',', comma1 + 1);
        
        if (comma1 > 0 && comma2 > 0) {
            int channel = command.substring(0, comma1).toInt();
            char mode = command.charAt(comma1 + 1);
            float value = command.substring(comma2 + 1).toFloat();
            
            if (channel >= 1 && channel <= 3) {
                if (mode == 'v' || mode == 'c') {
                    // Update signal mode and mark as configured
                    signalModes[channel - 1] = mode;
                    signalConfigured[channel - 1] = true;
                    
                    // Set channel mode first
                    setRelayMode(channel, mode);
                    
                    // Then set value using signal mapping
                    if (mode == 'v') {
                        if (value >= 0 && value <= 10) {
                            // Use signal mapping to set correct DAC and channel
                            signalMap[channel - 1].voltageDAC->setVoltage(value, signalMap[channel - 1].voltageChannel);
                            signalValues[channel - 1] = value; // Update signal value
                            Serial.printf("Channel %d set to VOLTAGE mode, output %.2fV\n", channel, value);
                            // Trigger status report after successful voltage setting
                            extern void printStatusReport();
                            printStatusReport();
                        } else {
                            Serial.println("Invalid voltage value (0-10V)");
                        }
                    } else if (mode == 'c') {
                        if (value >= 0 && value <= 25) {
                            // Use signal mapping to set correct DAC
                            // Convert mA to DAC data: Rset=2kΩ, 25mA = 32767 (15-bit), so 1mA = 1310.68
                            signalMap[channel - 1].currentDAC->setDACOutElectricCurrent(static_cast<uint16_t>(value * 1310.68));
                            signalValues[channel - 1] = value; // Update signal value
                            Serial.printf("Channel %d set to CURRENT mode, output %.2fmA\n", channel, value);
                            // Trigger status report after successful current setting
                            extern void printStatusReport();
                            printStatusReport();
                        } else {
                            Serial.println("Invalid current value (0-25mA)");
                        }
                    }
                } else {
                    Serial.println("Invalid mode (v/c)");
                }
            } else {
                Serial.println("Invalid channel (1-3)");
            }
        } else {
            Serial.println("Usage: channel,mode,value (e.g., 3,v,2.0)");
        }
    } else {
        Serial.println("Analog channel commands only available in Analog mode.");
        Serial.println("Use 'exit_modbus' to return to analog mode.");
    }
}

/**
 * Process modbus-related commands
 */
void processModbusCommand(String command) {
    String lowerCommand = command;
    lowerCommand.toLowerCase();
    
    if (lowerCommand.startsWith("modbus")) {
        // Enter modbus mode: modbus <slave_id>
        uint8_t slaveID = command.substring(7).toInt();
        enterModbusMode(slaveID);
    }
    else if (lowerCommand.startsWith("exit_modbus")) {
        // Exit modbus mode
        exitModbusMode();
    }
    else if (lowerCommand.startsWith("slave")) {
        // Set slave ID: slave <id> (only in modbus mode)
        if (isModbusModeActive()) {
            uint8_t slaveID = command.substring(6).toInt();
            setSlaveID(slaveID);
        } else {
            Serial.println("Use 'modbus <slave_id>' to enter Modbus mode first.");
        }
    }
    else if (lowerCommand.startsWith("measure")) {
        // Set all measurements: measure <flow> <consumption> <reverse> <direction>
        String params = command.substring(8); // Remove "measure "
        int space1 = params.indexOf(' ');
        int space2 = params.indexOf(' ', space1 + 1);
        int space3 = params.indexOf(' ', space2 + 1);
        
        if (space1 > 0 && space2 > 0 && space3 > 0) {
            float flow = params.substring(0, space1).toFloat();
            uint32_t consumption = params.substring(space1 + 1, space2).toInt();
            uint32_t reverse = params.substring(space2 + 1, space3).toInt();
            uint32_t direction = params.substring(space3 + 1).toInt();
            
            if (direction == 0 || direction == 1) {
                processMeasurementValues(flow, consumption, reverse, direction);
            } else {
                Serial.println("Invalid direction. Use 0 (same) or 1 (reverse).");
            }
        } else {
            Serial.println("Usage: measure <flow> <consumption> <reverse> <direction>");
            Serial.println("Example: measure 12.5 50000 2500 0");
        }
    }
    else if (lowerCommand.startsWith("flow")) {
        // Set flow value: flow <value>
        float flow = command.substring(5).toFloat();
        setFlowValue(flow);
    }
    else if (lowerCommand.startsWith("consumption")) {
        // Set consumption value: consumption <value>
        uint32_t consumption = command.substring(12).toInt();
        setConsumptionValue(consumption);
    }
    else if (lowerCommand.startsWith("reverse")) {
        // Set reverse consumption value: reverse <value>
        uint32_t reverse = command.substring(8).toInt();
        setReverseConsumptionValue(reverse);
    }
    else if (lowerCommand.startsWith("direction")) {
        // Set flow direction: direction <0|1>
        uint32_t direction = command.substring(10).toInt();
        if (direction == 0 || direction == 1) {
            setFlowDirectionValue(direction);
        } else {
            Serial.println("Invalid direction. Use 0 (same) or 1 (reverse).");
        }
    }
    else if (lowerCommand.startsWith("help") || lowerCommand.startsWith("status")) {
        // These are handled by system command processor
        processSystemCommand(command);
    }
}

/**
 * Process system commands (help, status, sine, etc.)
 */
void processSystemCommand(String command) {
    // For now, delegate back to main.cpp's handleUSBSerialCommands
    // This will be implemented in the next phase
    extern void handleSystemCommand(String command);
    handleSystemCommand(command);
}

/**
 * Process test commands (modbus_test, serial_test, etc.)
 */
void processTestCommand(String command) {
    // For now, delegate back to main.cpp's handleUSBSerialCommands  
    // This will be implemented in the next phase
    extern void handleTestCommand(String command);
    handleTestCommand(command);
}
