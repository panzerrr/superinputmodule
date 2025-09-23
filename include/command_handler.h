#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>

/**
 * Initialize command handler
 *
 * Initialize relays and DAC, set up I2C and pin states.
 */
void initCommandHandler();

/**
 * Main command processor - handles all USB serial commands
 * 
 * @param command Complete command string input by user
 */
void processCommand(String command);

/**
 * Process serial input commands
 *
 * Called in `loop()` to receive and process commands.
 * @param command Complete command string input by user
 */
void parseModeCommand(String params);
void parseValueCommand(String params);

// Command category processors
void processAnalogCommand(String command);
void processModbusCommand(String command);
void processSystemCommand(String command);
void processTestCommand(String command);
// Helper functions that were in main.cpp
void printStatusReport();
void printHelp();

// Main USB serial command handler (called from main.cpp loop)
void handleUSBSerialCommands();

#endif // COMMAND_HANDLER_H
