#include "Hexapod.h"

#ifndef DEBUG
  #include <avr/interrupt.h>
  #include <time.h>
#endif

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

  Hexapod hexapod {};

  hexapod.test();

  while(1);
}

#ifndef DEBUG
ISR(TIMER2_COMPA_vect){
  system_tick();
}
#endif
