# LuminaGrowX Project Context for Gemini

## 1. Project Overview
**Project Name:** LuminaGrowX_TMC_Lib
**Target Hardware:** ESP32-S3 (esp32-s3-devkitc-1-n16r8v)
**Framework:** PlatformIO / Arduino
**Goal:** Automated Growbox Controller ensuring optimal environmental conditions (VPD, Light, Airflow) for plant growth stages.

## 2. Tech Stack & Environment
- **Language:** C++ (Firmware), HTML/CSS/JS (Frontend).
- **Build System:** PlatformIO.
- **Filesystem:** LittleFS (mounted at `/littlefs`, partition label `littlefs`).
- **Web Server:** `ESPAsyncWebServer` + `AsyncTCP`.
- **Frontend Libs:** Pico CSS (`data/pico.min.css`).

## 3. Architecture & Code Structure
The project is highly modular, with core logic separated into the `lib/` directory.

### Key Directories
- **`src/main.cpp`**: Application entry point. Initializes hardware, sets up `PlantCtrl`, `WebCtrl`, and runs the main loop. **Avoid putting complex logic here.**
- **`lib/`**:
    - **`plant_ctrl`**: Core grow logic. Manages states (Seedling, Veg, Flower), Day/Night cycles, and target VPD/Dimming values.
    - **`stepper_ctrl`**: Control for the LED height adjustment using TMC2209 drivers.
    - **`TMCTiny`**: Custom lightweight TMC2209 UART library (fixes CRC issues).
    - **`web_ctrl`**: Handles HTTP requests, API endpoints, and serves the frontend.
    - **`vpd_calc`**: VPD (Vapor Pressure Deficit) calculation utilities.
    - **`sht41_ctrl`**: Driver for SHT41 Temp/Humidity sensors.
    - **`gp8211_ctrl`**: Driver for GP8211 DAC (LED dimming via 0-10V).
    - **`fan_ctrl`**: PWM Fan control.
    - **`tof_ctrl`**: VL53L0X Time-of-Flight sensor for distance measurement.
- **`include/`**:
    - **`lumina_config.h`**: **CENTRAL CONFIGURATION.** Pin definitions, default settings, constants. **Check here first for pin mappings.**
    - **`health.h`**: System health monitoring struct/logic.

## 4. Hardware Specifics & Known Issues (Memories)
### TMC2209 Stepper Driver
- **UART Communication:** Uses Single-Wire UART.
    - **Pins:** TX=17, RX=18.
    - **Wiring:** Requires a **1kΩ resistor Y-cable** connecting TX and RX to the driver's UART pin.
    - **Library:** Uses `TMCTiny` (custom) because the standard library had CRC calculation issues ("Echo but no reply"). The fix involved replacing the lookup table with a bitwise CRC implementation.
- **Motor Direction:**
    - **Issue:** Motor only moved in one direction (likely DIR pin 11 floating/disconnected).
    - **Workaround:** Configured `AXIS_UP_DIR_ = -1` in code to match logical expectations.
    - **Config:** `min_stall_mm` parameter filters initial mechanical noise during homing.

### I2C / Pin Conflicts
- **I2C2 Conflict:** I2C2 is assigned to Pin 20 in some configurations.
    - **Warning:** Pin 20 is part of the USB D+/D- interface on ESP32-S3. Usage can conflict with USB serial/upload.

### Sensors & Actuators
- **SHT41:** Two instances (`sht_in`, `sht_out`) on different I2C buses or addresses.
- **GP8211 (DAC):** Controls LED brightness (0-10V). I2C Address `0x58`.
- **ToF (VL53L0X):** Measures distance to plant canopy. Uses XSHUT pin for reset/addressing.

## 5. Development Workflow
- **Configuration:** Modify `include/lumina_config.h` for pin or parameter changes.
- **Build:** `pio run`
- **Upload Firmware:** `pio run -t upload`
- **Upload Filesystem:** `pio run -t uploadfs` (Required for Web UI updates in `data/`).
- **Serial Monitor:** `pio device monitor` (Baud: 115200).

## 6. Commands & Tools
- **Tarball:** `tools/make_tar.ps1` creates release packages.
- **OTA:** OTA updates are supported via the web interface or standard ArduinoOTA if enabled.

## 7. Interaction Guidelines
- **Sprache:** Code-Kommentare immer auf Deutsch verfassen.
- **Coding Style:** Follow existing patterns in `lib/`. Use `constexpr` for constants.
- **Safety:** Always check `lumina_config.h` before suggesting pin changes.
- **Frontend:** When editing HTML/CSS, remember to update the `data/` folder and remind the user to upload the filesystem.
