#ifndef QUADRATICFUNCTION_H
#define QUADRATICFUNCTION_H

class QuadraticFunction {

public:
  QuadraticFunction(float a = 0.0f, float b = 0.0f, float c = 0.0f);

  float getY(float y) const;

  float a;
  float b;
  float c;
};
#endif
