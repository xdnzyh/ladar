#ifndef UTILS_H
#define UTILS_H

#include <Wire.h>
#include <math.h>
#include <Arduino.h>

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE
#define I2CADDR 0x60
#define LED0_ON_L 0x6

void setPin(uint8_t Pin, uint8_t Value);

void setPWM(uint8_t Pin, uint16_t Value);

void PCA9685_Init(void);

uint8_t read8(uint8_t addr);

void write8(uint8_t addr, uint8_t d);

void SetPWMFreq(float freq=1600);

#endif
