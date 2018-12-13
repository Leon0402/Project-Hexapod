#ifndef POINT_INL
#define POINT_INL

#ifndef X86_64
  #include <math.h>
#else
  #include <cmath>
#endif

template <typename T>
Point<T>::Point(T x, T y , T z)
: x {x}, y {y}, z {z} {}

template <typename T>
void Point<T>::rotateX(T angle) {
  T y0 = this->y;
  T z0 = this->z;
  this->y = y0*cos(angle*M_PI/180.0f) - z0*sin(angle*M_PI/180.0f);
  this->z = y0*sin(angle*M_PI/180.0f) + z0*cos(angle*M_PI/180.0f);
}

template <typename T>
void Point<T>::rotateY(T angle) {
  T x0 = this->x;
  T z0 = this->z;
  this->x = x0*cos(angle*M_PI/180.0f)  + z0*sin(angle*M_PI/180.0f);
  this->z = x0*-sin(angle*M_PI/180.0f) + z0*cos(angle*M_PI/180.0f);
}

template <typename T>
void Point<T>::rotateZ(T angle) {
  T x0 = this->x;
  T y0 = this->y;
  this->x = x0*cos(angle*M_PI/180.0f) - y0*sin(angle*M_PI/180.0f);
  this->y = x0*sin(angle*M_PI/180.0f) + y0*cos(angle*M_PI/180.0f);
}

template <typename T>
void Point<T>::rotateXYZ(T yawAngle, T pitchAngle, T rollAngle) {
  T x0 = this->x;
  T y0 = this->y;
  T z0 = this->z;
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

template <typename T>
T Point<T>::distanceTo(const Point<T>& point) const {
  return sqrt((this->x - point.x)*(this->x - point.x)
            + (this->y - point.y)*(this->y - point.y)
            + (this->z - point.z)*(this->z - point.z));
}

#endif //POINT_INL
