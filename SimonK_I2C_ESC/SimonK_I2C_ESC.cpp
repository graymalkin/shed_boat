#include "SimonK_I2C_ESC.h"

namespace {
    char _buffer[9];
}

SimonK_I2C_ESC::SimonK_I2C_ESC(I2C &i2c, char address, char poleCount) : _i2c(i2c){
    _address = address << 1;
    _poleCount = poleCount;
    // A timer is needed to calculate the RPM.
    mbed_rpm_timer.start();
    _rpmTimer = mbed_rpm_timer.read_ms();
}

// Read the incoming data buffer from an ESC
void SimonK_I2C_ESC::readBuffer(char buffer[]) {
    char readStartAddress = 0x02;
    _i2c.write(_address,&readStartAddress,1,false);
    _i2c.read(_address,buffer,9);
}

// Send motor speed command to ESC
void SimonK_I2C_ESC::set(short throttle) {  
    char    throttleData[3];
    throttleData[0]   = 0x00; // Throttle Start Address
    throttleData[1]   = throttle>>8;
    throttleData[2]   = throttle;
    _i2c.write(_address, throttleData, 3, false);
}

// Send motor speed command to ESC
void SimonK_I2C_ESC::setPWM(short pwm) {  
    set((pwm - 1100) * (32767 - -32767) / (1900 - 1100) + -32767);
}

void SimonK_I2C_ESC::update() {  
    _buffer[8] = 0x00; // Reset last byte so we can check for alive
    readBuffer(_buffer);
    _rpm = (_buffer[0] << 8) | _buffer[1];
    _voltage_raw = (_buffer[2] << 8) | _buffer[3];
    _temp_raw = (_buffer[4] << 8) | _buffer[5];
    _current_raw = (_buffer[6] << 8) | _buffer[7];
    _identifier = _buffer[8];
    _rpm = float(_rpm) / (abs( mbed_rpm_timer.read_ms() -_rpmTimer)/1000.0f)*60/ float(_poleCount);
    _rpmTimer = mbed_rpm_timer.read_ms();
}

bool SimonK_I2C_ESC::isAlive() {
    return (_identifier == 0xab);
}

float SimonK_I2C_ESC::voltage() {
    return float(_voltage_raw)/65536.0f*5.0f*6.45f;
}

float SimonK_I2C_ESC::current() {
    return (float(_current_raw)-32767)/65535.0f*5.0f*14.706f;
}

float SimonK_I2C_ESC::temperature() {
  // This code was taken from an Adafruit
    float resistance = SERIESRESISTOR/(65535/float(_temp_raw)-1);

    float steinhart;
    steinhart = resistance / THERMISTORNOMINAL;  // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += float(1.0) / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = float (1.0) / steinhart;                 // Invert
    steinhart -= float(273.15);                         // convert to C

    return steinhart;
}

short SimonK_I2C_ESC::rpm() {
  return _rpm;
}