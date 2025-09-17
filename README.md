# ESP32 Stepper Motor Control with Adafruit TMC2209

This project provides a simple interface to control a stepper motor using the Adafruit TMC2209 driver with an ESP32. The motor can be controlled via serial commands to move up or down, and it can be stopped or started based on user input.

## Project Structure

```
esp32-stepper-tmc2209
├── src
│   ├── main.cpp          // Main entry point of the program
│   └── tmc2209_config.h  // Configuration settings for the TMC2209 module
├── lib
│   └── Adafruit_TMC2209
│       ├── Adafruit_TMC2209.h  // Header file for the TMC2209 class
│       └── Adafruit_TMC2209.cpp // Implementation of the TMC2209 class methods
├── platformio.ini       // PlatformIO configuration file
├── README.md            // Project documentation
└── .vscode
    └── settings.json     // VSCode settings for the project
```

## Installation

1. Clone this repository to your local machine.
2. Open the project in your preferred IDE (e.g., VSCode).
3. Ensure you have the PlatformIO extension installed.
4. Install the required libraries, including the Adafruit TMC2209 library.

## Usage

1. Connect the TMC2209 driver to the ESP32 using the following pin configuration:
   - DIR: GPIO 20
   - STEP: GPIO 21
   - EN: GPIO 0 (Enable pin)
   - DIAG: GPIO 35 (Diagnostic pin)

2. Upload the code to your ESP32 board.

3. Open the Serial Monitor in your IDE.

4. Use the following commands to control the stepper motor:
   - `+` : Move the motor up.
   - `-` : Move the motor down.
   - ` ` (space) : Stop or start the motor.

## Diagnostics

If there are any issues with the motor operation, diagnostic information will be output via the DIAG pin. Ensure to monitor this pin for any error messages.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.