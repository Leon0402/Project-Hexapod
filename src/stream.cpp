#include "Stream.h"

#ifndef X86_64
 #include <avr/io.h>
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

    UCSR0B |= (1<<RXCIE0);
  #endif
}

void Stream::write(char data) {
  #ifndef X86_64
    while (!(UCSR0A & (1<<UDRE0)));

    UDR0 = data;
  #endif
}

void Stream::write(const char* data) {
  while(*data) {
    write(*data);
    ++data;
  }
}

char Stream::read() {
  while (!(UCSR0A & (1<<RXC0)));
  return UDR0;
}

void Stream::read(char* buffer, uint8_t size) {
  uint8_t nextChar = this->read();

  for(uint8_t i = 0; i < size - 1; ++i) {
    if(nextChar == '\n' || nextChar == '\r') {
      break;
    }
    *buffer++ = nextChar;
    nextChar = this->read();
  }
  *buffer = '\0';
}
