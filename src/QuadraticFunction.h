#ifndef QUADRATICFUNCTION_H
#define QUADRATICFUNCTION_H

#ifndef X86_64
  #include <inttypes.h>
#else
  #include <cstdint>
#endif

class QuadraticFunction {

public:
  QuadraticFunction(float a = 0.0f, float b = 0.0f, float c = 0.0f);

  uint8_t getSlope(float x) const;
  int16_t getY(int16_t y) const;

  float a;
  float b;
  float c;
};
#endif //QUADRATICFUNCTION_H
