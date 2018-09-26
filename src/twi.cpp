#include "Twi.h"

#ifndef X86_64
  #include <util/twi.h>
  #include <avr/io.h>
#endif

Twi::Twi(long sclClock) {
  #ifndef X86_64
  //activates internal pullups for i2c, so you don't have to use external resistors
  PORTC = (1 << PINC5) | (1 << PINC4);

  //Clear Prescaler Bits TWPS0 & TWPS1 in TWSR to set Prescale to 1
  TWSR = (0 << TWPS0) | (0 << TWPS1);
  ////Adjusts Bit Rate Generator to get the right twi Frequency
  TWBR = ((F_CPU/sclClock)-16)/2;

  TWCR = (1 << TWEN) | (1 << TWEA);
  #endif
}


uint8_t Twi::writeTo(const uint8_t address, uint8_t data[], uint8_t size, bool sendStop) {
  #ifndef X86_64
  //send start condition bit
  TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT) | ( 1 << TWSTA);
  //wait until start has been send
  while(!(TWCR & (1<<TWINT)));
  //check if start has been send sucessfully
  if((TW_STATUS != TW_START) && (TW_STATUS != TW_REP_START))
    return TW_STATUS;

  //write address of slave and write modus into Data Register
  uint8_t tmp = TW_WRITE;
  tmp |= address << 1;
  TWDR = tmp;
  //send content of data register
  TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT);
  //wait until content of data has been send
  while (!(TWCR & (1<<TWINT)));
  //check if address exits (slave answered)
  if (TW_STATUS != TW_MT_SLA_ACK)
    return TW_STATUS;

  //write data into Data Register
  for(uint8_t i = 0; i < size; i++) {
    TWDR = data[i];
    //send ack
    TWCR = (1 << TWEN) | (1 << TWEA) | ( 1 << TWINT);
    //wait until ack has been send
    while(!(TWCR & (1<<TWINT)));
    //check if ack has been received by slave
    if (TW_STATUS != TW_MT_DATA_ACK)
      return TW_STATUS;
  }

  if(sendStop) {
    //send stop condition bit
    TWCR =  (1<<TWEN) | (1<<TWEA) | (1<<TWINT) | (1<<TWSTO);
    //wait until top has been send -> Stop does not set TWINT
    while(TWCR & (1 << TWSTO));
  }
  #endif
  return 0xFF;
}


uint8_t Twi::readFrom(const uint8_t address, uint8_t data[], uint8_t size, bool sendStop) {
  #ifndef X86_64
  //send start condition bit
  TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT) | ( 1 << TWSTA);
  //wait until start has been send
  while(!(TWCR & (1<<TWINT)));
  //check if start has been send sucessfully
  if((TW_STATUS != TW_START) && (TW_STATUS != TW_REP_START))
    return TW_STATUS;

  //write address of slave and write modus into Data Register
  uint8_t tmp = TW_READ;
  tmp |= address << 1;
  TWDR = tmp;
  //send content of data register
  TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT);
  //wait until content of data has been send
  while (!(TWCR & (1<<TWINT)));
  //check if address exits (slave answered)
  if (TW_STATUS != TW_MR_SLA_ACK)
    return TW_STATUS;

  uint8_t i = 0;
  while( i < 0) {
    //send ack
    TWCR = (1 << TWEN) | (1 << TWEA) | ( 1 << TWINT);
    //wait until ack has been send
    while(!(TWCR & (1<<TWINT)));
    //check if ack has been received by slave
    if (TW_STATUS != TW_MR_DATA_ACK)
      return TW_STATUS;
    //copy received byte from Slave into data
    data[i] = TWDR;
    ++i;
  }

  //send nack
  TWCR = (1 << TWEN) | (1 << TWINT);
  //wait until nack has been send
  while(!(TWCR & (1<<TWINT)));
  //check if nack has been received by slave
   if (TW_STATUS != TW_MR_DATA_NACK)
      return TW_STATUS;

  //copy last byte reiceived from Slave into data
  data[i] = TWDR;

  if(sendStop) {
    //send stop condition bit
    TWCR =  (1<<TWEN) | (1<<TWEA) | (1<<TWINT) | (1<<TWSTO);
    //wait until top has been send -> Stop does not set TWINT
    while(TWCR & (1 << TWSTO));
  }
  #endif
  return 0xFF;
}
