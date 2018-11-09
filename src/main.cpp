  #include "Hexapod.h"

#ifndef X86_64
  #include <avr/interrupt.h>
  #include <time.h>
  #include <util/delay.h>
  #include <string.h>
#endif

namespace {
  Hexapod hexapod {};
}

void executeFunction(char functionName);

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

  //hexapod.bodyIk_test();
  //hexapod.moveForward_test();

  while(1);
}

void executeFunction(char functionName) {
  char buffer[10];
  avr::cout.read(buffer, sizeof buffer / sizeof buffer[0]);
  switch(functionName) {
    case 'b': avr::cout << strtok (buffer, ",") << '\n'; break;
  }
  avr::cout << "Test" << '\n';
/*
  char * argument;
	argument = strtok (command, ",");
	while(argument) {
    while(*argument) {
      avr::cout << *argument;
      argument++;
    }
		argument = strtok (NULL, ",");
	}*/
}

#ifndef X86_64
//update system time
ISR(TIMER2_COMPA_vect) {
  system_tick();
}

//update servos
ISR(TIMER1_COMPA_vect) {
  hexapod.update(time(nullptr));
}

//UART read commands from the esp / serial console
ISR(USART_RX_vect) {
  executeFunction(avr::cout.read());
}
#endif
