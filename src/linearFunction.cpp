#include "LinearFunction.h"

#ifndef X86_64
  #include <math.h>
#else
  #include <cmath>
#endif

LinearFunction::LinearFunction(Pointf point1, Pointf point2)
: slope {(point1.x - point2.x)/(point1.y - point2.y)},
  yIntercept {point1.y - slope*point2.x} {}

bool LinearFunction::getIntersectionWith(const LinearFunction& function, Pointf& intersection) const {
  if(this->slope == function.slope) {
    return false;
  }
  intersection.x = (yIntercept - function.yIntercept)/(slope - function.slope);
  intersection.y = slope*intersection.x + yIntercept;
  return true;
}

void LinearFunction::getIntersectionWith(const Pointf& circleCenter, uint8_t radius, Pointf intersections[2]) const {
  //Mx2*m - m*n + Mx1
  float beforeRoot = circleCenter.y*slope - slope*yIntercept + circleCenter.x;
  //2*Mx2*m*Mx1 - 2*Mx1*m*y - Mx2² - y² +  2*Mx2*y + r² - Mx1²m² + r²m²
  float root = 2*circleCenter.y*slope*circleCenter.x - 2*circleCenter.x*slope*yIntercept
               - circleCenter.y*circleCenter.y - yIntercept*yIntercept + 2*circleCenter.y*yIntercept
               + radius*radius - circleCenter.x*circleCenter.x*slope*slope + radius*radius*slope*slope;
  //(Mx2*m - m*n + Mx1 + sqrt(2*Mx2*m*Mx1 - 2*Mx1*m*y - Mx2² - y² +  2*Mx2*y + r² - Mx1²m² + r²m²))/(1 + m²)
  intersections[0] = (beforeRoot + sqrt(root))/(1+slope*slope);
  //(Mx2*m - m*n + Mx1 - sqrt(2*Mx2*m*Mx1 - 2*Mx1*m*y - Mx2² - y² +  2*Mx2*y + r² - Mx1²m² + r²m²))/(1 + m²)
  intersections[1] = (beforeRoot - sqrt(root))/(1+slope*slope);
}

void LinearFunction::rotateXY(float angle) {
  Pointf point1 {1, this->slope*1 + this->yIntercept};
  point1.rotateXY(angle);

  Pointf point2 {-1, this->slope*-1 + this->yIntercept };
  point2.rotateXY(angle);

  this->slope = (point1.x - point2.x)/(point1.y - point2.y);
  this->yIntercept = point1.y - slope*point2.x;
}
