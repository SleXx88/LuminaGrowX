#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_timer.h>

/*
  TMCTiny - A lightweight library combining TMC2209 UART control
  and interrupt-based step generation for ESP32.
*/

namespace TMCTiny {

// TMC2209 Registers
enum Reg : uint8_t {
    GCONF      = 0x00,
    GSTAT      = 0x01,
    IFCNT      = 0x02,
    IOIN       = 0x06,
    IHOLD_IRUN = 0x10,
    TPOWERDOWN = 0x11,
    TPWMTHRS   = 0x13,
    TCOOLTHRS  = 0x14,
    VACTUAL    = 0x22,
    SGTHRS     = 0x40,
    SG_RESULT  = 0x41,
    CHOPCONF   = 0x6C,
    DRV_STATUS = 0x6F
};

class TMCTinyStepper {
public:
    TMCTinyStepper(uint8_t pinStep, uint8_t pinDir, uint8_t pinEn, HardwareSerial* serial, float r_sense = 0.11f, uint8_t addr = 0);
    ~TMCTinyStepper();

    // --- Setup ---
    void begin(uint32_t baud, int8_t pinRx, int8_t pinTx);
    void enable(bool en); // true = Output enabled (Low active on pin)
    void setAddress(uint8_t addr); // Set TMC2209 slave address (0-3)

    // --- TMC2209 Configuration ---
    void setMicrosteps(uint16_t msteps);
    void setCurrent(uint16_t mA, float holdMultiplier = 0.5f);
    void setStealthChop(bool enable); // true = StealthChop, false = SpreadCycle
    void setStallGuardThreshold(uint8_t threshold);
    uint16_t getStallGuardResult();
    void setCoolStep(uint32_t tcool_thrs, uint16_t semin = 5, uint16_t semax = 2);
    
    // Raw Register Access
    void writeRegister(uint8_t reg, uint32_t data);
    bool readRegister(uint8_t reg, uint32_t &val);

    // --- Motion Control (Interrupt Driven) ---
    void setMaxSpeed(float speedHz);
    void setAcceleration(float accelHz2);
    
    void move(long steps);           // Relative move
    void moveTo(long position);      // Absolute move
    void runForward();               // Continuous
    void runBackward();              // Continuous
    void stop();                     // Stop with deceleration
    void forceStopAndNewPosition(long newPos); // Immediate stop, reset pos

    long getCurrentPosition() const;
    void setCurrentPosition(long pos);
    bool isRunning() const;
    bool isStopping() const;

    // --- Internal ISR helper ---
    void _isrHandler();

private:
    // Pins & UART
    uint8_t _pinStep, _pinDir, _pinEn;
    HardwareSerial* _serial;
    float _rSense;
    uint8_t _addr;

    // Motion State
    volatile long _currentPos = 0;
    volatile long _targetPos = 0;
    volatile bool _isRunning = false;
    volatile bool _isContinuous = false;
    volatile int8_t _dir = 0; // 0=Stop, 1=Fwd, -1=Back
    volatile float _currentSpeed = 0.0f; // Hz
    
    // Ramp Config
    float _maxSpeed = 1000.0f;
    float _accel = 1000.0f;
    
    // Ramp Calculation State
    volatile unsigned long _stepIntervalUs = 0;
    volatile float _rampSpeed = 0.0f;
    float _c0 = 0.0f; // First step delay
    volatile long _accelSteps = 0;
    volatile long _decelSteps = 0;
    volatile long _totalStepsToMove = 0;
    
    // Timer
    esp_timer_handle_t _timer = nullptr;
    
    // Helpers
    void calcRamp();
    void startTimer();
    void stopTimer();
    void setDir(int8_t d);
    
    // TMC Helpers
    uint8_t calcCRC(uint8_t* dat, uint8_t len);
    void sendDatagram(uint8_t reg, uint32_t data, bool write);
};

} // namespace TMCTiny
