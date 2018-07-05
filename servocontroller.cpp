#include "Servocontroller.h"

#ifndef DEBUG
#include <Arduino.h>
#endif
//setzt adresse des Servo Controller -> default = 0x40 und erzeugt TwoWire Objekt zur Übertragung
Servocontroller::Servocontroller(uint8_t addr)
: i2caddr(addr)
    #ifndef DEBUG
    ,i2c{TwoWire {}}
    #endif
{
  begin();
  setPWMFreq(50);
}


//setzt default Übertragungspins
void Servocontroller::begin() {
    #ifndef DEBUG
    i2c.begin();
    #endif
    reset();
}

//resettet Controller
void Servocontroller::reset() {
    write8(PCA9685_MODE1, 0x80);
    #ifndef DEBUG
    delay(10);
    #endif
}

//Setzt die Frequenz der PWM Signale
void Servocontroller::setPWMFreq(float freq) {

  freq *= 0.9;
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  #ifndef DEBUG
  uint8_t prescale = floor(prescaleval + 0.5);
  #else
  uint8_t prescale = 0;
  #endif
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  #ifndef DEBUG
  delay(5);
  #endif
  write8(PCA9685_MODE1, oldmode | 0xa0);  //  This sets the MODE1 register to turn on auto increment.
}

//Erzeugt ein PWM Signal am pin pinNum mit dem übergebenem on und off wert im Zyklus
void Servocontroller::setPWM(uint8_t pinNum, uint16_t on, uint16_t off) {
    #ifndef DEBUG
    i2c.beginTransmission(i2caddr);
    i2c.write(LED0_ON_L+4*pinNum);
    i2c.write(on);
    i2c.write(on>>8);
    i2c.write(off);
    i2c.write(off>>8);
    i2c.endTransmission();
    #endif
}

/*******************************************************************************************/
//Hilfsfunktionen
uint8_t Servocontroller::read8(uint8_t addr) {
    #ifndef DEBUG
    i2c.beginTransmission(i2caddr);
    i2c.write(addr);
    i2c.endTransmission();
    i2c.requestFrom((uint8_t)i2caddr, (uint8_t)1);
    return i2c.read();
    #endif
    return 0;
}

void Servocontroller::write8(uint8_t addr, uint8_t d) {
    #ifndef DEBUG
    i2c.beginTransmission(i2caddr);
    i2c.write(addr);
    i2c.write(d);
    i2c.endTransmission();
    #endif
}
