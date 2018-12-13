#ifndef POINT_H
#define POINT_H

#include "Stream.h"

template <typename T>
class Point {

public:
	Point(T x = 0, T y = 0, T z = 0);

	void rotateX(T angle);
  void rotateY(T angle);
  void rotateZ(T angle);
  void rotateXYZ(T yawAngle, T pitchAngle, T rollAngle);
  T distanceTo(const Point<T>& point) const;

  friend bool operator==(const Point& point1, const Point& point2) {
    return point1.x == point2.x && point1.y == point2.y && point1.z == point2.z;
  }

	T x;
	T y;
	T z;
};

#include "Point.inl"

template <typename T>
inline Stream& operator<<(Stream& stream, const Point<T>& point) {
	stream << '(' << point.x << '/' << point.y << '/' << point.z << ')';
	return stream;
}

#endif //POINT_H
