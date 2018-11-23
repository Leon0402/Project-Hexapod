#include "LinearFunction.h"

#ifndef X86_64
  #include <math.h>
#else
  #include <cmath>
#endif

LinearFunction::LinearFunction(float slope, int16_t yIntercept)
: slope {slope}, yIntercept {yIntercept} {}

LinearFunction::LinearFunction(float slope, Point<int16_t> point1)
: slope {slope}, yIntercept {static_cast<int16_t>(round(point1.y - slope*point1.x))} {}

LinearFunction::LinearFunction(Point<int16_t> point1, Point<int16_t> point2)
: slope {(point1.y - point2.y)/static_cast<float>(point1.x - point2.x)},
  yIntercept {static_cast<int16_t>(point1.y - slope*point1.x)} {}

int16_t LinearFunction::getY(int16_t x) const {
  return this->slope*x + this->yIntercept;
}

bool LinearFunction::getIntersectionWith(const LinearFunction& function, Point<int16_t>& intersection) const {
  if(this->slope == function.slope) {
    return false;
  }
  intersection.x = (yIntercept - function.yIntercept)/(function.slope - slope);
  intersection.y = slope*intersection.x + yIntercept;
  return true;
}

void LinearFunction::getIntersectionWith(const Point<int16_t>& circleCenter, uint8_t radius, Point<int16_t> intersections[2]) const {
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

void LinearFunction::rotateZ(uint16_t angle) {
  Point<int16_t> point {100, static_cast<int16_t>(round(this->slope*1 + this->yIntercept))};
  point.rotateZ(angle);

  //Thorugh rotation it can happen that the Linear Function is parallel to the y-axis
  if(point.x == 0) {
    this->slope = 255;
  } else {
    this->slope = point.y/static_cast<float>(point.x);
  }

  this->yIntercept = round(point.y - slope*point.x);
}
