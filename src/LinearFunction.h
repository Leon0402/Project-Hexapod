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
  LinearFunction(float slope = 0.0f, int16_t yIntercept = 0);
  LinearFunction(float slope, Point<int16_t> point1);
  LinearFunction(Point<int16_t> point1, Point<int16_t> point2);

  int16_t getY(int16_t x) const;
  bool getIntersectionWith(const LinearFunction& linearFunction, Point<int16_t>& intersection) const;
  void getIntersectionWith(const Point<int16_t>& circleCenter, uint8_t radius, Point<int16_t> intersections[2]) const;
  void rotateZ(uint16_t angle);

  float slope;
  int16_t yIntercept;
};
#endif //LINEARFUNCTION_H
