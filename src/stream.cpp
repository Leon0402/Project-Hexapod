#include "Stream.h"

#ifndef X86_64
 #include <avr/io.h>
 #include <stdio.h>
 #include <stdlib.h>
#endif


namespace avr {
  Stream cout {};
  Stream err {};
}

Stream::Stream() {
  #ifndef X86_64
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    #if USE_2X
      UCSR0A |= (1 << U2X0);
    #else
      UCSR0A &= ~(1 << U2X0);
    #endif
  #endif
}

void Stream::write(int8_t data) {
  #ifndef X86_64
    while (!(UCSR0A & (1<<UDRE0)));

    UDR0 = data;
  #endif
}

void Stream::write(const int8_t* data, uint8_t size) {
  for(uint8_t i = 0; i < size; ++i) {
    this->write(*data);
    ++data;
  }
}

void Stream::print(char data) {
  this->write(data);
}

void Stream::print(const char* data) {
  while(*data) {
    write(*data);
    ++data;
  }
}

void Stream::print(int data) {
  char buffer[11];
  this->print(ltoa(data, buffer, 10));

}

void Stream::print(unsigned int data) {
  char buffer[10];
  this->print(ultoa(data, buffer, 10));
}

void Stream::print(float data) {
  char buffer[7];
  this->print(dtostrf(data, 6, 2, buffer));
}
