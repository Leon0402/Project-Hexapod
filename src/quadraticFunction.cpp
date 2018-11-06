#include "QuadraticFunction.h"

QuadraticFunction::QuadraticFunction(float a, float b, float c)
: a {a}, b {b}, c {c} {}

float QuadraticFunction::getY(float x) const{
  return a*(x - b)*(x - b) + c;
}
