#ifndef POINT_H
#define POINT_H

#include "Controls.h"

#ifdef DEBUG
#include <cstdint>
#include <cmath>
#else
#include <math.h>
#endif

class Point {
public:
	Point(int8_t x1, int8_t x2, int8_t x3) :
			x(x1), y(x2), z(x3) {
	}

	void rotateXY(int16_t angle) {
		x = x * cos(angle * M_PI / 180) - y * sin(angle * M_PI / 180);
		y = x * sin(angle * M_PI / 180) + y * cos(angle * M_PI / 180);
	}

	int8_t x;
	int8_t y;
	int8_t z;
};

class Pointf {
public:
	Pointf(float x1, float x2, float x3) :
			x(x1), y(x2), z(x3) {
	}

	void rotateXY(float angle) {
		x = x * cos(angle * M_PI / 180) - y * sin(angle * M_PI / 180);
		y = x * sin(angle * M_PI / 180) + y * cos(angle * M_PI / 180);
	}

	float x;
	float y;
	float z;
};
#endif //POINT_H
