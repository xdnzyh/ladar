#include "utils.h"

// function:set the state of a pin
// Pin: the index of pin
// Value: 1 is High, 0 is low
void setPin(uint8_t Pin, uint8_t Value){
      Wire.beginTransmission(0x60);
      Wire.write(LED0_ON_L+4*Pin);
      Wire.write(Value*4096);
      Wire.write(Value*4096>>8);
      Wire.write(0);
      Wire.write(0);
      Wire.endTransmission();
}

// function:set PWM duty cycle of a pin
// Pin: the index of pin
// Value: 0 ~ 4095
void setPWM(uint8_t Pin, uint16_t Value){
      if (Value > 4095) {
        Value = 4095;
      }

      Wire.beginTransmission(0x60);
      Wire.write(LED0_ON_L+4*Pin);

      if (Value == 0) {
        // Full OFF
        Wire.write(0);
        Wire.write(0);
        Wire.write(0);
        Wire.write(0x10);
      }
      else {
        // ON at count 0, OFF at count Value
        Wire.write(0);
        Wire.write(0);
        Wire.write(Value & 0xFF);
        Wire.write(Value >> 8);
      }

      Wire.endTransmission();
}

void PCA9685_Init(){
  write8(PCA9685_MODE1, 0x0);
  SetPWMFreq(1600);
}

uint8_t read8(uint8_t addr){
    Wire.beginTransmission(I2CADDR);
    Wire.write(addr);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)I2CADDR, (uint8_t)1);
    return Wire.read();
}

void write8(uint8_t addr, uint8_t d){
    Wire.beginTransmission(I2CADDR);
    Wire.write(addr);
    Wire.write(d);
    Wire.endTransmission();
}

void SetPWMFreq(float freq){
  freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).

  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;

  uint8_t prescale = floor(prescaleval + 0.5);
  
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                          // This is why the beginTransmission below was not working.
  
}
