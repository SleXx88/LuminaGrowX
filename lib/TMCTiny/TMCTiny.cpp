#include "TMCTiny.h"

namespace TMCTiny {

// Global instance pointer for ISR context (simplification for single motor or static binding,
// but for multiple instances, we need to pass args to callback. esp_timer supports args.)
static void IRAM_ATTR timerCallback(void* arg) {
    static_cast<TMCTinyStepper*>(arg)->_isrHandler();
}

TMCTinyStepper::TMCTinyStepper(uint8_t pinStep, uint8_t pinDir, uint8_t pinEn, HardwareSerial* serial, float r_sense, uint8_t addr)
    : _pinStep(pinStep), _pinDir(pinDir), _pinEn(pinEn), _serial(serial), _rSense(r_sense), _addr(addr)
{
}

TMCTinyStepper::~TMCTinyStepper() {
    if (_timer) esp_timer_delete(_timer);
}

void TMCTinyStepper::begin(uint32_t baud, int8_t pinRx, int8_t pinTx) {
    pinMode(_pinStep, OUTPUT);
    pinMode(_pinDir, OUTPUT);
    digitalWrite(_pinDir, LOW); // Default Low
    pinMode(_pinEn, OUTPUT);
    digitalWrite(_pinEn, HIGH); // Disable by default
    digitalWrite(_pinStep, LOW);
    
    _serial->begin(baud, SERIAL_8N1, pinRx, pinTx);
    
    const esp_timer_create_args_t timer_args = {
        .callback = &timerCallback,
        .arg = this,
        .name = "tmc_tiny"
    };
    esp_timer_create(&timer_args, &_timer);
}

void TMCTinyStepper::enable(bool en) {
    digitalWrite(_pinEn, en ? LOW : HIGH);
}

void TMCTinyStepper::setAddress(uint8_t addr) {
    _addr = addr;
}

// --- TMC2209 Logic ---

uint8_t TMCTinyStepper::calcCRC(uint8_t* dat, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t currentByte = dat[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            currentByte = currentByte >> 1;
        }
    }
    return crc;
}

void TMCTinyStepper::sendDatagram(uint8_t reg, uint32_t data, bool write) {
    uint8_t packet[8];
    uint8_t len = 0;
    
    packet[len++] = 0x05; // Sync
    packet[len++] = _addr;
    packet[len++] = reg | (write ? 0x80 : 0x00);
    
    if (write) {
        packet[len++] = (data >> 24) & 0xFF;
        packet[len++] = (data >> 16) & 0xFF;
        packet[len++] = (data >> 8) & 0xFF;
        packet[len++] = data & 0xFF;
    }
    
    packet[len++] = calcCRC(packet, len);
    
    // Clear buffer (important to remove old noise/echoes)
    while (_serial->available() > 0) _serial->read();
    
    _serial->write(packet, len);
    _serial->flush(); // Wait for TX to complete
    
    // If read, wait for response
    if (!write) {
        // Wait loop handled by caller or basic delay here?
        // Caller readRegister handles the timeout and parsing.
        // But we add a tiny delay to allow switching from TX to RX.
        // delayMicroseconds(100); 
    }
}

void TMCTinyStepper::writeRegister(uint8_t reg, uint32_t data) {
    sendDatagram(reg, data, true);
}

bool TMCTinyStepper::readRegister(uint8_t reg, uint32_t &val) {
    sendDatagram(reg, 0, false);
    
    uint32_t start = millis();
    while ((millis() - start) < 20) { // Increased timeout to 20ms
        if (_serial->available() >= 8) {
             // Peek to see if we have a valid header (0x05, 0xFF)
             // If we have an echo (0x05, 0x00) in front, we need to skip it.
             
             // Simple state machine or sliding window?
             // Let's just consume until we find 0x05 0xFF or run out of data.
             
             int b = _serial->peek();
             if (b == 0x05) {
                 // Check next byte without consuming yet? Not easy with Arduino API.
                 // We read, check, if fail, we are misaligned.
                 // But wait, if it is Echo (05 00) and we read 05, next is 00.
                 // If we want 05 FF.
                 
                 // Let's read buffer into a temporary array and scan.
                 uint8_t buf[32]; // Max buffer
                 int avail = _serial->available();
                 if (avail > 32) avail = 32;
                 
                 // We only peek/read if we are sure? No, just read all and parse.
                 // But if we read all, we can't put back.
                 // Correct way: Wait for enough data, then parse.
             }
             break; // Go to parsing logic
        }
        delay(1);
    }
    
    // Naive robust approach: Read all available bytes (up to limit) and search for signature.
    int avail = _serial->available();
    if (avail < 8) return false; // Still nothing
    
    uint8_t buf[64];
    int readLen = _serial->readBytes(buf, (avail > 64 ? 64 : avail));
    
    for (int i = 0; i <= readLen - 8; i++) {
        if (buf[i] == 0x05 && buf[i+1] == 0xFF) {
            // Found Header!
            // buf[i+2] should be reg
            // buf[i+3]..buf[i+6] is data
            // buf[i+7] is CRC
            
            val = ((uint32_t)buf[i+3] << 24) | ((uint32_t)buf[i+4] << 16) | ((uint32_t)buf[i+5] << 8) | buf[i+6];
            return true;
        }
    }
    
    return false;
}

void TMCTinyStepper::setMicrosteps(uint16_t msteps) {
    uint8_t mres = 0;
    // 256=0, 128=1, 64=2, 32=3, 16=4, 8=5, 4=6, 2=7, 1=8
    switch(msteps) {
        case 256: mres=0; break;
        case 128: mres=1; break;
        case 64: mres=2; break;
        case 32: mres=3; break;
        case 16: mres=4; break;
        case 8: mres=5; break;
        case 4: mres=6; break;
        case 2: mres=7; break;
        case 1: mres=8; break;
        default: mres=4; break; // Default 16
    }
    uint32_t chopconf = 0;
    if (readRegister(CHOPCONF, chopconf)) {
        chopconf &= ~(0x0F << 24); // Clear MRES
        chopconf |= (mres << 24);
        writeRegister(CHOPCONF, chopconf);
    }
}

void TMCTinyStepper::setCurrent(uint16_t mA, float holdMultiplier) {
    // CS = 32 * 1.414 * I_rms * R_sense / 0.325
    float cs_f = 32.0f * 1.41421f * (float)mA / 1000.0f * _rSense / 0.325f;
    uint8_t cs = (uint8_t)constrain(cs_f, 0.0f, 31.0f);
    
    uint8_t ihold = (uint8_t)(cs * holdMultiplier);
    uint8_t irun = cs;
    uint8_t iholddelay = 10; 
    
    uint32_t val = (iholddelay << 16) | (irun << 8) | ihold;
    writeRegister(IHOLD_IRUN, val);
}

void TMCTinyStepper::setStealthChop(bool enable) {
    uint32_t gconf = 0;
    if (!readRegister(GCONF, gconf)) return; // Read failed, cannot modify safely

    if (enable) 
        gconf &= ~(1<<2);
    else 
        gconf |= (1<<2);
    writeRegister(GCONF, gconf);
}

void TMCTinyStepper::setStallGuardThreshold(uint8_t threshold) {
    writeRegister(SGTHRS, threshold);
}

void TMCTinyStepper::setCoolStep(uint32_t tcool_thrs, uint16_t semin, uint16_t semax) {
    writeRegister(TCOOLTHRS, tcool_thrs);
}

// --- Motion Control ---

void TMCTinyStepper::setMaxSpeed(float speedHz) {
    _maxSpeed = speedHz;
}

void TMCTinyStepper::setAcceleration(float accelHz2) {
    _accel = accelHz2;
}

long TMCTinyStepper::getCurrentPosition() const {
    return _currentPos;
}

void TMCTinyStepper::setCurrentPosition(long pos) {
    _currentPos = pos;
}

bool TMCTinyStepper::isRunning() const {
    return _isRunning;
}

bool TMCTinyStepper::isStopping() const {
    return _isRunning && !_isContinuous && (_targetPos == _currentPos); // Actually needs better state check
}

void TMCTinyStepper::setDir(int8_t d) {
    if (d > 0) digitalWrite(_pinDir, HIGH); // INVERTED: High = Forward
    else digitalWrite(_pinDir, LOW);
    _dir = d;
}

void TMCTinyStepper::forceStopAndNewPosition(long newPos) {
    stopTimer();
    _isRunning = false;
    _currentPos = newPos;
    _targetPos = newPos;
}

void TMCTinyStepper::stop() {
    if (!_isRunning) return;
    if (_isContinuous) {
        // Switch to positioning mode to decel
        // Calculate stopping distance: s = v^2 / (2*a)
        float v = _rampSpeed; // Current speed
        long stepsToStop = (long)((v * v) / (2.0f * _accel));
        _targetPos = _currentPos + (_dir * stepsToStop);
        _isContinuous = false;
        
        // Recalculate ramp state for deceleration
        _decelSteps = stepsToStop;
        _totalStepsToMove = stepsToStop; // Treating as if this is the end
        // This is a simplification. A proper ramp generator handles "Stop" by changing state to DECEL.
        // For "Tiny" implementation: 
        // We just let the ISR handle the "target reached" logic which includes deceleration.
        // But we need to ensure we are "past" the accel phase if we are fast.
    }
}

void TMCTinyStepper::move(long steps) {
    moveTo(_currentPos + steps);
}

void TMCTinyStepper::runForward() {
    if (_isRunning && _isContinuous && _dir == 1) return;
    forceStopAndNewPosition(_currentPos); // Brutal stop if changing dir
    _isContinuous = true;
    _dir = 1;
    setDir(1);
    
    // Logic: Target is infinite.
    // We set a huge target.
    _targetPos = _currentPos + 1000000000; 
    
    startTimer();
}

void TMCTinyStepper::runBackward() {
    if (_isRunning && _isContinuous && _dir == -1) return;
    forceStopAndNewPosition(_currentPos);
    _isContinuous = true;
    _dir = -1;
    setDir(-1);
    
    _targetPos = _currentPos - 1000000000;
    
    startTimer();
}

void TMCTinyStepper::moveTo(long position) {
    if (position == _currentPos) return;
    forceStopAndNewPosition(_currentPos); // Simple approach: stop before new move
    
    _isContinuous = false;
    _targetPos = position;
    _dir = (_targetPos > _currentPos) ? 1 : -1;
    setDir(_dir);
    
    startTimer();
}

// --- Stepper Core (Linear Ramping) ---

void TMCTinyStepper::startTimer() {
    _isRunning = true;
    _rampSpeed = 50.0f; // Start speed (min)
    if (_rampSpeed > _maxSpeed) _rampSpeed = _maxSpeed;
    
    // c0 = 1000000 / sqrt(2*a) * 0.676? No, simpler: 
    // First step delay. 
    // Let's use speed-based tracking.
    // Initial delay for first step:
    _stepIntervalUs = (unsigned long)(1000000.0f / _rampSpeed);
    
    long delta = labs(_targetPos - _currentPos);
    _totalStepsToMove = delta;
    
    // Calculate acceleration ramp
    // Steps to reach max speed: n = v^2 / (2*a)
    _accelSteps = (long)((_maxSpeed * _maxSpeed) / (2.0f * _accel));
    if (_accelSteps > delta / 2) _accelSteps = delta / 2;
    _decelSteps = _accelSteps; 
    
    // If continuous, we just accel until max.
    if (_isContinuous) {
        _accelSteps = (long)((_maxSpeed * _maxSpeed) / (2.0f * _accel));
        _decelSteps = 0;
    }
    
    esp_timer_start_once(_timer, _stepIntervalUs);
}

void TMCTinyStepper::stopTimer() {
    esp_timer_stop(_timer);
    _isRunning = false;
}

void IRAM_ATTR TMCTinyStepper::_isrHandler() {
    if (!_isRunning) return;
    
    // Pulse Step
    digitalWrite(_pinStep, HIGH);
    // Short delay? ESP32 is fast. 1-2us is enough.
    // With overhead, it's likely fine. For robustness:
    for(volatile int i=0; i<5; i++); 
    digitalWrite(_pinStep, LOW);
    
    // Update Position
    if (_dir > 0) _currentPos++;
    else _currentPos--;
    
    // Check Target
    long distToGo = labs(_targetPos - _currentPos);
    
    if (!_isContinuous && distToGo == 0) {
        _isRunning = false;
        return; // Done
    }
    
    // Calculate Next Speed (Trapezoidal)
    // Steps performed so far?
    // We don't store "steps performed", we use distToGo for Decel and Speed for Accel?
    // Let's use current Speed.
    
    // Current distance from start? Need to know steps taken.
    // Let's use a simpler state approach.
    long stepsDone = (_isContinuous) ? 1000000 : (_totalStepsToMove - distToGo); 
    // (If continuous, we always accel up to max)
    
    if (_isContinuous) {
        float dt = (float)_stepIntervalUs / 1000000.0f;
        if (_rampSpeed < _maxSpeed) {
            _rampSpeed += _accel * dt;
            if (_rampSpeed > _maxSpeed) _rampSpeed = _maxSpeed;
        } else if (_rampSpeed > _maxSpeed) {
            _rampSpeed -= _accel * dt;
            if (_rampSpeed < _maxSpeed) _rampSpeed = _maxSpeed;
        }
    } else {
        // Discrete Move
        if (distToGo <= _decelSteps) {
            // Decelerate
            float dt = (float)_stepIntervalUs / 1000000.0f;
            _rampSpeed -= _accel * dt;
            if (_rampSpeed < 50.0f) _rampSpeed = 50.0f; // Min speed
        } else if (_rampSpeed < _maxSpeed) {
            // Accelerate
            float dt = (float)_stepIntervalUs / 1000000.0f;
            _rampSpeed += _accel * dt;
            if (_rampSpeed > _maxSpeed) _rampSpeed = _maxSpeed;
        }
    }
    
    _stepIntervalUs = (unsigned long)(1000000.0f / _rampSpeed);
    esp_timer_start_once(_timer, _stepIntervalUs);
}

} // namespace TMCTiny
