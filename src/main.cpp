#include <Arduino.h>
#include <Wire.h>
#include "dac_controller.h"
#include "relay_controller.h"
#include "sine_wave_generator.h"
#include "device_id.h"
#include "modbus_handler.h"
#include "command_handler.h"
#include "utils.h"

char signalModes[3] = {'v', 'v', 'v'};
float signalValues[3] = {0.0f, 0.0f, 0.0f}; // Track values for each signal

// Forward declarations - main functions only
// Command handling functions are now in command_handler.cpp

void setup() {
    // Initialize USB Serial for debugging
    Serial.begin(115200);
    Serial.println("=== ESP32 Input Module with RS-485 ===");
    
    // Initialize I2C communication
    Wire.begin(21, 22);  // SDA = GPIO21, SCL = GPIO22 (according to schematic)
    Serial.println("I2C initialized (SDA=GPIO21, SCL=GPIO22)");
    
    // Initialize device ID
    initDeviceIDPins();
    uint8_t deviceID = calculateDeviceID();
    Serial.printf("Device ID: %d\n", deviceID);
    
    // Initialize DAC controllers
    initDACControllers();
    Serial.println("DAC controllers initialized");
    
    // Initialize relay controller
    initRelayController();
    Serial.println("Relay controller initialized");

    // Default to three-channel voltage mode on startup
    setRelayMode(1, 'v');
    setRelayMode(2, 'v');
    setRelayMode(3, 'v');
    
    // Initialize sine wave generator
    initSineWaveGenerator();
    Serial.println("Sine wave generator initialized");
    
    
    // Initialize Modbus slave
    initModbus();
    
    Serial.println("System initialization complete");
    Serial.println("USB Serial: Debug output only");
    Serial.println("Modbus Slave: Interface (GPIO 17=TX, 16=RX)");
    Serial.println("Ready to receive commands...");
}

void loop() {
    // Process USB Serial commands
    handleUSBSerialCommands();
    
    
    // Handle Modbus slave tasks
    mb.task();
    
    // Update sine wave generator
    updateSineWave();
    
    
    // Small delay to prevent watchdog issues
    delay(10);
}