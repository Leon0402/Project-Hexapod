#include "Hexapod.h"

#ifndef X86_64
  #include <avr/interrupt.h>
  #include <util/delay.h>
  #include <time.h>
#endif

namespace {
  Hexapod hexapod {};
}

int main() {
  #ifndef X86_64
  //enable global interrupts
  sei();

  //configure Timer2 to be used as a timer (systen time is updated every 1ms)
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22);
  OCR2A = 252;
  TIMSK2 |= (1 << OCIE2A);

  //configure Timer0 to update all servos if they are active (~60ms btw. 3 servos every ~20ms)
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);
  OCR1A = 25;
  TIMSK1 |= (1 << OCIE1A);
  #endif

  //hexapod.moveLinear(0, true);
  hexapod.startPosition_test();
  _delay_ms(2000);
  //hexapod.pitch(20.0f);
  //hexapod.yaw(20.0f);
/*
  while(1) {
    char read = avr::cout.read();

    avr::cout.write(read);

    switch(read) {
      case 't': hexapod.test(); break;
      default: avr::cout.write("a");
    }
  }*/

  while(1);
}

#ifndef X86_64
//update system time
ISR(TIMER2_COMPA_vect){
  system_tick();
}
//update servos
ISR(TIMER1_COMPA_vect){
  hexapod.update(time(nullptr));
}
#endif
