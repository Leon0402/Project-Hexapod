#ifndef POINT_H
#define POINT_H

#include "Stream.h"

#ifndef X86_64
  #include <math.h>
#else
  #include <cmath>
#endif

class Pointf {

public:
	Pointf(float x = 0.0f, float y = 0.0f, float z = 0.0f);

	void rotateX(float angle);
  void rotateY(float angle);
  void rotateZ(float angle);
  void rotateXYZ(float yawAngle, float pitchAngle, float rollAngle);
  uint8_t distanceTo(const Pointf& point) const;

	float x;
	float y;
	float z;
};

inline Stream& operator<<(Stream& stream, const Pointf& point) {
	stream << '(' << point.x << '/' << point.y << '/' << point.z << ')';
	return stream;
}
#endif //POINT_H
