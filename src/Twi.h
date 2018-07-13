#ifndef TWI_H
#define TWI_H

#ifdef DEBUG
  #include <cstdint>
#else
  #include <inttypes.h>
#endif

class Twi {
public:
  Twi(long sclClock);

  uint8_t writeTo(const uint8_t address, uint8_t data[], uint8_t size, bool sendStop = true);
  uint8_t readFrom(const uint8_t address, uint8_t data[], uint8_t size, bool sendStop = true);
};

#endif //TWI_H
