#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

class LedCtrl {
public:
    enum class Color {
        OFF,
        RED,
        GREEN,
        BLUE,
        WHITE,
        ORANGE,
        PURPLE,
        CYAN,
        YELLOW,
        MAGENTA,
        WARM_WHITE
    };

    enum class Mode {
        SOLID,
        BLINK,
        BREATHE,    // Smooth blinking / Fading
        RAINBOW
    };

    /**
     * @brief Construct a new Led Ctrl object
     * 
     * @param pin GPIO Pin connected to Data In of WS2812B
     * @param numLeds Number of chained LEDs (default 1)
     */
    LedCtrl(int pin, int numLeds = 1);

    void begin();
    
    /**
     * @brief Update the LED state. Must be called frequently in the main loop.
     */
    void update();

    // --- Configuration ---

    void setColor(Color color);
    void setCustomColor(uint8_t r, uint8_t g, uint8_t b);
    
    /**
     * @brief Set global brightness
     * @param brightness 0 (off) to 255 (max)
     */
    void setBrightness(uint8_t brightness);

    void setMode(Mode mode);
    
    /**
     * @brief Set the cycle duration for Blink or Breathe modes
     * @param ms Duration in milliseconds (e.g., 1000ms = 1s cycle)
     */
    void setCycleTime(unsigned long ms);

private:
    Adafruit_NeoPixel _pixels;
    int _pin;
    int _numLeds;
    
    // State
    uint32_t _targetColorPacked = 0; // Stored as 0x00RRGGBB
    Mode _currentMode = Mode::SOLID;
    uint8_t _globalBrightness = 255;

    // Timing
    unsigned long _lastUpdate = 0;
    unsigned long _cycleDuration = 1000; // default 1 second
    
    // Internal helpers
    uint32_t getColorPacked(Color c);
    uint32_t applyBrightness(uint32_t color, uint8_t brightness);
    void showColor(uint32_t color);
    
    // Rainbow state
    uint16_t _rainbowHue = 0;
};
