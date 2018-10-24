#ifndef LINEARFUNCTION_H
#define LINEARFUNCTION_H

#include "Point.h"

#ifndef X86_64
  #include <inttypes.h>
#else
  #include <cstdint>
#endif

class LinearFunction {

public:
  LinearFunction(float slope = 0, float yIntercept = 0);
  LinearFunction(float slope, Pointf point1);
  LinearFunction(Pointf point1, Pointf point2);

  bool getIntersectionWith(const LinearFunction& linearFunction, Pointf& intersection) const;
  void getIntersectionWith(const Pointf& circleCenter, uint8_t radius, Pointf intersections[2]) const;
  void rotateXY(float angle);

  float slope;
  float yIntercept;
};
#endif //LINEARFUNCTION_H
