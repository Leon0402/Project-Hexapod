#include "Point.h"

Pointf::Pointf(float x, float y, float z)
: x {x}, y {y}, z {z} {}

void Pointf::rotateXY(float angle) {
  float xAlt = x;
  this->x = this->x*cos(angle*M_PI/180.0f) - this->y*sin(angle*M_PI/180.0f);
  this->y = xAlt*sin(angle*M_PI/180.0f) + this->y*cos(angle*M_PI/180.0f);
}

uint8_t Pointf::distanceTo(const Pointf& point) const {
  return sqrt((this->x - point.x)*(this->x - point.x)
            + (this->y - point.y)*(this->y - point.y)
            + (this->z - point.z)*(this->z - point.z));
}
