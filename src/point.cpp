#include "Point.h"

Pointf::Pointf(float x, float y, float z)
: x {x}, y {y}, z {z} {}

void Pointf::rotateX(float angle) {
  float y0 = this->y;
  float z0 = this->z;
  this->y = y0*cos(angle*M_PI/180.0f) - z0*sin(angle*M_PI/180.0f);
  this->z = y0*sin(angle*M_PI/180.0f) + z0*cos(angle*M_PI/180.0f);
}

void Pointf::rotateY(float angle) {
  float x0 = this->x;
  float z0 = this->z;
  this->x = x0*cos(angle*M_PI/180.0f)  + z0*sin(angle*M_PI/180.0f);
  this->z = x0*-sin(angle*M_PI/180.0f) + z0*cos(angle*M_PI/180.0f);
}

void Pointf::rotateZ(float angle) {
  float x0 = this->x;
  float y0 = this->y;
  this->x = x0*cos(angle*M_PI/180.0f) - y0*sin(angle*M_PI/180.0f);
  this->y = x0*sin(angle*M_PI/180.0f) + y0*cos(angle*M_PI/180.0f);
}

void Pointf::rotateXYZ(float yawAngle, float pitchAngle, float rollAngle) {
  float x0 = this->x;
  float y0 = this->y;
  float z0 = this->z;
  this->x = x0*cos(yawAngle*M_PI/180.0f)*cos(pitchAngle*M_PI/180.0f)
          + y0*(cos(yawAngle*M_PI/180.0f)*sin(pitchAngle*M_PI/180.0f)*sin(rollAngle*M_PI/180.0f) - sin(yawAngle*M_PI/180.0f)*cos(rollAngle*M_PI/180.0f))
          + z0*(cos(yawAngle*M_PI/180.0f)*sin(pitchAngle*M_PI/180.0f)*cos(rollAngle*M_PI/180.0f) + sin(yawAngle*M_PI/180.0f)*sin(rollAngle*M_PI/180.0f));
  this->y = x0*sin(yawAngle*M_PI/180.0f)*cos(pitchAngle*M_PI/180.0f)
          + y0*(sin(yawAngle*M_PI/180.0f)*sin(pitchAngle*M_PI/180.0f)*sin(rollAngle*M_PI/180.0f) + cos(yawAngle*M_PI/180.0f)*cos(rollAngle*M_PI/180.0f))
          + z0*(sin(yawAngle*M_PI/180.0f)*sin(pitchAngle*M_PI/180.0f)*cos(rollAngle*M_PI/180.0f) - cos(yawAngle*M_PI/180.0f)*sin(rollAngle*M_PI/180.0f));
  this->z = x0*-sin(pitchAngle*M_PI/180.0f)
          + y0*cos(pitchAngle*M_PI/180.0f)*sin(rollAngle*M_PI/180.0f)
          + z0*cos(pitchAngle*M_PI/180.0f)*cos(rollAngle*M_PI/180.0f);
}

uint8_t Pointf::distanceTo(const Pointf& point) const {
  return sqrt((this->x - point.x)*(this->x - point.x)
            + (this->y - point.y)*(this->y - point.y)
            + (this->z - point.z)*(this->z - point.z));
}
