#include "Hexapod.h"

#ifndef DEBUG
#include <avr/interrupt.h>
#include <time.h>
#include <util/delay.h>
#endif

Hexapod hexapod {};

int main() {
  #ifndef DEBUG
  sei();
  #endif

  #ifndef DEBUG
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22);
  OCR2A = 252;
  TIMSK2 |= (1 << OCIE2A);
  #endif

  #ifndef DEBUG
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01) || (1 << CS00);
  OCR0A = 252;
  TIMSK0 |= (1 << OCIE0A);
  #endif

  while(1);
}

#ifndef DEBUG
ISR(TIMER2_COMPA_vect){
  system_tick();
}
#endif

#ifndef DEBUG
ISR(TIMER0_COMPA_vect){
  hexapod.update(time(nullptr));
}
#endif
