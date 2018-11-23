#include "QuadraticFunction.h"

#include "Stream.h"

QuadraticFunction::QuadraticFunction(float a, float b, float c)
: a {a}, b {b}, c {c} {}

uint8_t QuadraticFunction::getSlope(float x) const {
  //calculate the first derivation and calculate y to get the slope
  return 2*a*x + b;
}

int16_t QuadraticFunction::getY(int16_t x) const {
  //set x into the function to get y
  return a*(x - b)*(x - b) + c;
}
