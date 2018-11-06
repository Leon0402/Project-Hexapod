#include "LinearFunction.h"

#ifndef X86_64
  #include <math.h>
#else
  #include <cmath>
#endif

LinearFunction::LinearFunction(float slope, float yIntercept)
: slope {slope}, yIntercept {yIntercept} {}

LinearFunction::LinearFunction(float slope, Pointf point1)
: slope {slope}, yIntercept {point1.y - slope*point1.x} {}

LinearFunction::LinearFunction(Pointf point1, Pointf point2)
: slope {(point1.y - point2.y)/(point1.x - point2.x)},
  yIntercept {point1.y - slope*point1.x} {}

float LinearFunction::getY(float x) const {
  return this->slope*x + this->yIntercept;
}

bool LinearFunction::getIntersectionWith(const LinearFunction& function, Pointf& intersection) const {
  if(this->slope == function.slope) {
    return false;
  }
  intersection.x = (yIntercept - function.yIntercept)/(function.slope - slope);
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
  intersections[0].x = (beforeRoot + sqrt(root))/(1+slope*slope);
  intersections[0].y = slope*intersections[0].x + yIntercept;
  //(Mx2*m - m*n + Mx1 - sqrt(2*Mx2*m*Mx1 - 2*Mx1*m*y - Mx2² - y² +  2*Mx2*y + r² - Mx1²m² + r²m²))/(1 + m²)
  intersections[1].x = (beforeRoot - sqrt(root))/(1+slope*slope);
  intersections[1].y = slope*intersections[1].x + yIntercept;
}

void LinearFunction::rotateXY(float angle) {
  Pointf point1 {1, this->slope*1 + this->yIntercept};
  point1.rotateZ(angle);

  Pointf point2 {-1, this->slope*-1 + this->yIntercept };
  point2.rotateZ(angle);

  if(point1.x == point2.x) {
    this->slope = 255;
  } else {
    this->slope = (point1.y - point2.y)/(point1.x - point2.x);
  }

  this->yIntercept = point1.y - slope*point1.x;
}
