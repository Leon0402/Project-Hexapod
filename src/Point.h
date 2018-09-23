#ifndef POINT_H
#define POINT_H

#ifdef DEBUG
  #include <cmath>
  #include <ostream>
#else
  #include <math.h>
#endif

class Pointf {
public:
	Pointf(float x1 = 0.0f, float x2 = 0.0f, float x3 = 0.0f) :
			x(x1), y(x2), z(x3) {
	}

	void rotateXY(float angle) {
		float xAlt = x;
		x = x*cos(angle * M_PI / 180.0f) - y * sin(angle * M_PI / 180.0f);
		y = xAlt * sin(angle * M_PI / 180.0f) + y * cos(angle * M_PI / 180.0f);
	}

	float x;
	float y;
	float z;
};

#ifdef DEBUG
inline std::ostream& operator<<(std::ostream& os, const Pointf& point) {
	os << '(' << point.x << '/' << point.y << '/' << point.z << ')';
	return os;
}
#endif

#endif //POINT_H


/*
cos takes radian, returns degrees
*/
