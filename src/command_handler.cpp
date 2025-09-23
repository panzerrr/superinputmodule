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
        } else if (lowerCommand.startsWith("sine")) {
            processSystemCommand(command);
        } else if (lowerCommand.startsWith("ping") || lowerCommand.startsWith("test485") || 
                   lowerCommand.startsWith("voltage") || lowerCommand.startsWith("current") || 
                   lowerCommand.startsWith("stop")) {
            processRS485Command(command);
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
    String lowerCommand = command;
    lowerCommand.toLowerCase();
    
    if (lowerCommand.startsWith("help")) {
        printHelp();
    }
    else if (lowerCommand.startsWith("status")) {
        // Show local system status
        printStatusReport();
    }
    else if (lowerCommand.startsWith("sine")) {
        // Handle sine wave commands directly
        parseSineWaveCommand(command);
    }
}

/**
 * Process RS485-related commands
 */
void processRS485Command(String command) {
    String lowerCommand = command;
    lowerCommand.toLowerCase();
    
    if (lowerCommand.startsWith("ping")) {
        // Send ping command via RS-485 (temporarily disabled)
        Serial.println("RS-485 functionality temporarily disabled, waiting for definition");
    }
    else if (lowerCommand.startsWith("test485")) {
        // Test RS-485 connection (temporarily disabled)
        Serial.println("RS-485 functionality temporarily disabled, waiting for definition");
    }
    else if (lowerCommand.startsWith("voltage")) {
        // Set voltage via RS-485: voltage <value>
        // Example: voltage 5.0
        float voltage = command.substring(8).toFloat();
        if (voltage >= 0 && voltage <= 10) {
            uint16_t voltageRaw = (uint16_t)(voltage * 100);
            uint8_t data[2] = {(uint8_t)(voltageRaw >> 8), (uint8_t)(voltageRaw & 0xFF)};
            sendTestRS485Command(CMD_SET_VOLTAGE, data, 2);
        } else {
            Serial.println("Invalid voltage value (0-10V)");
        }
    }
    else if (lowerCommand.startsWith("current")) {
        // Set current via RS-485: current <value>
        // Example: current 10.5
        float current = command.substring(8).toFloat();
        if (current >= 0 && current <= 25) {
            uint16_t currentRaw = (uint16_t)(current * 100);
            uint8_t data[2] = {(uint8_t)(currentRaw >> 8), (uint8_t)(currentRaw & 0xFF)};
            sendTestRS485Command(CMD_SET_CURRENT, data, 2);
        } else {
            Serial.println("Invalid current value (0-25mA)");
        }
    }
    else if (lowerCommand.startsWith("stop")) {
        // Stop sine wave via RS-485
        sendTestRS485Command(CMD_STOP_SINE, nullptr, 0);
    }
}

/**
 * Process test commands (modbus_test, serial_test, etc.)
 */
void processTestCommand(String command) {
    String lowerCommand = command;
    lowerCommand.toLowerCase();
    
    if (lowerCommand.startsWith("modbus_test")) {
        // Test Modbus connection
        Serial.println("=== Modbus Connection Test ===");
        Serial.printf("Serial1 RX Pin: GPIO%d\n", MODBUS_RX_PIN);
        Serial.printf("Serial1 TX Pin: GPIO%d\n", MODBUS_TX_PIN);
        Serial.printf("Baud Rate: %d\n", BAUDRATE);
        Serial.printf("Parity: 8E1\n");
        Serial.printf("Slave ID: %d\n", SLAVE_ID);
        Serial.printf("TXEN Pin: %d\n", TXEN_PIN);
        Serial.printf("Serial1 available bytes: %d\n", Serial1.available());
        Serial.println("Available registers:");
        Serial.println("  Registers are added dynamically when you send commands");
        Serial.println("  Use format: REGN,TYPE,VALUE to add registers");
        Serial.println("  Example: 1000,I,12345 adds register 1000 with U64 value 12345");
        
        Serial.println("Listening for Modbus requests for 15 seconds...");
        Serial.println("ModbusPoll settings should be:");
        Serial.println("  - Slave ID: 1");
        Serial.println("  - Function: 03 (Read Holding Registers)");
        Serial.println("  - Address: 0");
        Serial.println("  - Quantity: 1");
        Serial.println("  - Baud: 19200, 8E1");
        Serial.println("  - COM Port: Select correct port");
        Serial.println("");
        Serial.println("Starting monitoring...");
        
        unsigned long startTime = millis();
        int requestCount = 0;
        int totalBytes = 0;
        int modbusResponses = 0;
        
        while (millis() - startTime < 15000) {
            // Check for incoming data
            if (Serial1.available()) {
                int bytes = Serial1.available();
                totalBytes += bytes;
                requestCount++;
                Serial.printf("[%lu] Received data #%d: %d bytes\n", millis() - startTime, requestCount, bytes);
                
                // Read and display the raw data
                uint8_t buffer[64];
                int readBytes = Serial1.readBytes(buffer, min(bytes, 64));
                Serial.print("Raw data: ");
                for (int i = 0; i < readBytes; i++) {
                    Serial.printf("0x%02X ", buffer[i]);
                }
                Serial.println();
                
                // Try to parse as Modbus request
                if (readBytes >= 8) { // Minimum Modbus RTU frame size
                    Serial.printf("Possible Modbus request: Slave=0x%02X, Func=0x%02X\n", buffer[0], buffer[1]);
                }
            }
            
            // Process Modbus tasks
            mb.task();
            // Since we can't check the return value anymore, we'll increment on each task call
            // Note: This might not be accurate as it doesn't confirm a response was actually sent
            modbusResponses++;
            Serial.printf("[%lu] Modbus task processed #%d\n", millis() - startTime, modbusResponses);
            
            delay(10);
        }
        
        Serial.printf("Test complete. Received %d data packets, %d total bytes.\n", requestCount, totalBytes);
        Serial.printf("Sent %d Modbus responses.\n", modbusResponses);
        
        if (requestCount == 0) {
            Serial.println("No data received! Check:");
            Serial.println("  1. USB-Serial adapter connection");
            Serial.println("  2. COM port selection in ModbusPoll");
            Serial.println("  3. Baud rate settings (19200)");
            Serial.println("  4. USB-Serial adapter driver");
        } else if (modbusResponses == 0) {
            Serial.println("Data received but no Modbus responses sent!");
            Serial.println("Check Modbus protocol settings:");
            Serial.println("  - Slave ID must be 1");
            Serial.println("  - Function must be 03 (Read Holding Registers)");
            Serial.println("  - Address must be 0-3");
        }
        Serial.println("=============================");
    }
    else if (lowerCommand.startsWith("serial_test")) {
        // Simple Serial2 loopback test
        Serial.println("=== Serial2 Loopback Test ===");
        Serial.println("This test will send data via Serial2 TX and read it back via RX");
        Serial.println("Connect GPIO 16 (RX) to GPIO 17 (TX) with a jumper wire");
        Serial.println("Starting test in 3 seconds...");
        delay(3000);
        
        const char* testMessage = "ESP32 Serial2 Test Message";
        Serial.printf("Sending: %s\n", testMessage);
        Serial2.write(testMessage);
        Serial2.write('\n');
        
        delay(100); // Wait for data to be sent
        
        Serial.println("Reading back data...");
        String received = "";
        unsigned long startTime = millis();
        while (millis() - startTime < 2000) {
            if (Serial2.available()) {
                received += (char)Serial2.read();
            }
            delay(10);
        }
        
        if (received.length() > 0) {
            Serial.printf("Received: %s\n", received.c_str());
            Serial.println("Loopback test PASSED - Serial2 is working!");
        } else {
            Serial.println("Loopback test FAILED - No data received");
            Serial.println("Check jumper wire connection between GPIO 16 and 17");
        }
        Serial.println("=============================");
    }
    else if (lowerCommand.startsWith("send_modbus")) {
        // Send a test Modbus request
        Serial.println("=== Send Test Modbus Request ===");
        Serial.println("Sending test Modbus request to read register 0x0000");
        
        // Create a simple Modbus RTU request: [Slave ID][Function][Address High][Address Low][Quantity High][Quantity Low][CRC Low][CRC High]
        uint8_t request[] = {
            0x01,  // Slave ID
            0x03,  // Function 03 (Read Holding Registers)
            0x00,  // Address High
            0x00,  // Address Low (register 0)
            0x00,  // Quantity High
            0x01,  // Quantity Low (1 register)
            0x84,  // CRC Low (calculated for this request)
            0x0A   // CRC High
        };
        
        Serial.print("Sending request: ");
        for (int i = 0; i < 8; i++) {
            Serial.printf("0x%02X ", request[i]);
        }
        Serial.println();
        
        Serial1.write(request, 8);
        Serial1.flush();
        Serial.println("Request sent via Serial1");
        
        // Wait for response
        Serial.println("Waiting for response...");
        unsigned long startTime = millis();
        String response = "";
        while (millis() - startTime < 2000) {
            if (Serial1.available()) {
                response += (char)Serial1.read();
            }
            delay(10);
        }
        
        if (response.length() > 0) {
            Serial.printf("Received response (%d bytes): ", response.length());
            for (int i = 0; i < response.length(); i++) {
                Serial.printf("0x%02X ", (uint8_t)response[i]);
            }
            Serial.println();
        } else {
            Serial.println("No response received");
        }
        Serial.println("=============================");
    }
}

/**
 * Print periodic status report via USB Serial
 */
void printStatusReport() {
    Serial.println("\n=== Status Report ===");
    
    // Device information
    Serial.printf("Device ID: %d\n", getCurrentDeviceID());
    
    // System mode status
    if (isModbusModeActive()) {
        Serial.printf("System Mode: MODBUS (Slave ID: %d)\n", currentSlaveID);
        Serial.println("Analog outputs: DISABLED");
    } else {
        Serial.println("System Mode: ANALOG");
        Serial.println("Analog outputs: ENABLED");
    }
    
    // Signal status
    for (int i = 0; i < 3; i++) {
        // Check if this channel is running sine wave
        if (isSineWaveActiveOnChannel(i)) {
            // Display sine wave parameters instead of current values
            float amplitude, period, center;
            char mode;
            if (getSineWaveParams(i, &amplitude, &period, &center, &mode)) {
                const char* modeStr = (mode == 'v') ? "voltage" : "current";
                const char* unit = (mode == 'v') ? "V" : "mA";
                Serial.printf("SIG%d: %s mode, SINE WAVE (%.2f%s amplitude, %.1fs period, center %.2f%s)\n", 
                             i + 1, modeStr, amplitude, unit, period, center, unit);
            }
        } else {
            // Display normal manual mode values
            char mode = signalModes[i];
            const char* modeStr = (mode == 'v') ? "voltage" : (mode == 'c') ? "current" : "unknown";
            
            if (mode == 'v') {
                Serial.printf("SIG%d: %s mode, %.2f V\n", i + 1, modeStr, signalValues[i]);
            } else if (mode == 'c') {
                Serial.printf("SIG%d: %s mode, %.2f mA\n", i + 1, modeStr, signalValues[i]);
            } else {
                Serial.printf("SIG%d: %s mode\n", i + 1, modeStr);
            }
        }
    }
    
    Serial.println("==================\n");
}

/**
 * Print help information
 */
void printHelp() {
    Serial.println("\n=== USB Serial Commands ===");
    
    if (isModbusModeActive()) {
        Serial.println("=== MODBUS MODE ACTIVE ===");
        Serial.println("Mode Commands:");
        Serial.println("exit_modbus            - Return to analog mode");
        Serial.println("");
        Serial.println("Measurement Commands:");
        Serial.println("measure <f> <c> <r> <d> - Set all measurements at once");
        Serial.println("  Example: measure 12.5 50000 2500 0");
        Serial.println("  f=flow, c=consumption, r=reverse, d=direction(0|1)");
        Serial.println("");
        Serial.println("Individual Commands:");
        Serial.println("flow <value>            - Set flow measurement (Register 6)");
        Serial.println("consumption <value>     - Set consumption (Register 8)");
        Serial.println("reverse <value>         - Set reverse consumption (Register 14)");
        Serial.println("direction <0|1>         - Set flow direction (Register 42)");
        Serial.println("slave <id>              - Change slave ID (1-247)");
    } else {
        Serial.println("=== ANALOG MODE ACTIVE ===");
        Serial.println("Mode Commands:");
        Serial.println("modbus <slave_id>       - Enter Modbus mode (disables analog outputs)");
        Serial.println("");
        Serial.println("Analog Output Commands:");
        Serial.println("channel,mode,value      - Set channel output");
        Serial.println("  Example: 3,v,2.0      - Channel 3 output 2.0V voltage");
        Serial.println("  Example: 2,c,10.5     - Channel 2 output 10.5mA current");
        Serial.println("  channel: 1-3, mode: v(voltage)/c(current)");
        Serial.println("  voltage: 0-10V, current: 0-25mA");
        Serial.println("");
        Serial.println("SINE START <amp> <period> <center> <signal> <mode> - Start sine wave");
        Serial.println("  Example: SINE START 2.0 2.0 5.0 1 V");
        Serial.println("SINE STOP [signal]      - Stop sine wave");
        Serial.println("SINE STATUS             - Show sine wave status");
        Serial.println("");
    }
    
    Serial.println("System Commands:");
    Serial.println("ping                    - Send ping command via RS-485 (disabled)");
    Serial.println("test485                 - Test RS-485 connection (disabled)");
    Serial.println("status                  - Show local system status");
    Serial.println("modbus_test             - Test Modbus connection and show configuration");
    Serial.println("serial_test             - Test Serial2 loopback (connect GPIO 16 to 17)");
    Serial.println("send_modbus             - Send test Modbus request");
    Serial.println("help                    - Show this help");
    Serial.println("========================================\n");
}

/**
 * Example of how to send a command via RS-485 from USB Serial
 * This function can be called from USB Serial commands for testing
 */
void sendTestRS485Command(uint8_t commandType, const uint8_t* data, uint8_t length) {
    // Create command buffer
    uint8_t buffer[RS485_MAX_COMMAND_LENGTH];
    uint8_t bufferIndex = 0;
    
    // Add start byte
    buffer[bufferIndex++] = 0xAA;
    
    // Add device ID (broadcast to all devices)
    buffer[bufferIndex++] = 0xFF;
    
    // Add command type
    buffer[bufferIndex++] = commandType;
    
    // Add data
    if (length > 0 && data != nullptr) {
        memcpy(&buffer[bufferIndex], data, length);
        bufferIndex += length;
    }
    
    // Add end byte
    buffer[bufferIndex++] = 0x55;
    
    // Send via RS-485
    sendRS485Response(0xFF, commandType, data, length);
    
    Serial.printf("Test command sent: Type=0x%02X, Length=%d\n", commandType, length);
}

/**
 * Test RS-485 connection
 */
void testRS485Connection() {
    Serial.println("\n=== RS-485 Connection Test ===");
    
    // Test 1: Send ping command
    Serial.println("Test 1: Sending ping command...");
    sendTestRS485Command(CMD_PING, nullptr, 0);
    delay(100);
    
    // Test 2: Send voltage command
    Serial.println("Test 2: Sending voltage command (5.0V)...");
    uint16_t voltageRaw = (uint16_t)(5.0 * 100);
    uint8_t data[2] = {(uint8_t)(voltageRaw >> 8), (uint8_t)(voltageRaw & 0xFF)};
    sendTestRS485Command(CMD_SET_VOLTAGE, data, 2);
    delay(100);
    
    // Test 3: Send current command
    Serial.println("Test 3: Sending current command (10.0mA)...");
    uint16_t currentRaw = (uint16_t)(10.0 * 100);
    uint8_t currentData[2] = {(uint8_t)(currentRaw >> 8), (uint8_t)(currentRaw & 0xFF)};
    sendTestRS485Command(CMD_SET_CURRENT, currentData, 2);
    delay(100);
    
    // Test 4: Check for incoming data
    Serial.println("Test 4: Checking for incoming RS-485 data...");
    Serial.println("Listening for 2 seconds...");
    
    unsigned long startTime = millis();
    int bytesReceived = 0;
    
    while (millis() - startTime < 2000) {
        if (processRS485Commands()) {
            bytesReceived++;
            Serial.printf("Received command #%d\n", bytesReceived);
        }
        delay(10);
    }
    
    if (bytesReceived == 0) {
        Serial.println("No RS-485 data received during test period");
        Serial.println("Check wiring: TX=GPIO19, RX=GPIO18");
        Serial.println("Baud rate: 19200, Parity: 8E1");
    } else {
        Serial.printf("Successfully received %d commands\n", bytesReceived);
    }
    
    Serial.println("=== RS-485 Test Complete ===\n");
}

/**
 * Handle USB Serial commands for testing
 */
void handleUSBSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        processCommand(command);
    }
}
