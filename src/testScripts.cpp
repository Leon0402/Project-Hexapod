 #include "TestScripts.h"

void bodyIk_test(Hexapod& hexapod) {
  /*
  float points[][3] = {{15, 0, 0},    {7.5, 0, 0},  {15, 0, 0} ,  {0, 0, 0},
                       {0, -15, 0},   {0, -7.5, 0}, {0, -15, 0},  {0, 0, 0},
                       {0, 0, 15},    {0, 0, 7.5},  {0, 0, 15},   {0, 0, 0},
                       {0, 15, 15},   {0, 0, 0},    {0, -15, 15}, {0, 0, 0},
                       {0, -15, -15}, {0, 0, 0},    {0, 15, -15}, {0, 0, 0},
                       {-15, 0, 0},   {-7.5, 0, 0}, {-15, 0, 0},  {0, 0, 0},
                       {0, 15, 0},    {0, 7.5, 0},  {0, 15, 0},   {0, 0, 0},
                       {0, 0, -15},   {0, 0, -7.5}, {0, 0, -15},  {0, 0, 0},
                       {0, -15, -15}, {0, 0, 0},    {0, 15, -15}, {0, 0, 0},
                       {0, 15, 15},   {0, 0, 0},    {0, -15, 15}, {0, 0, 0}};

  for(float (&point)[3]: points) {
    float a = point[0];
    float b = point[1];
    float c = point[2];
    hexapod.bodyIk(a, b, c);
    //hexapod.bodyIk(5.0f, -3.0f, 2.0f);
  }*/
}

/*
void startPosition(Hexapod& hexapod) {
  hexapod.
}*/

void moveLinear_test(Hexapod& hexapod) {
  //wave gait
  //avr::cout << "waveGait" << '\n';
  for(uint8_t i = 0; i < 2; ++i) {
    hexapod.moveLinear(waveGait, 0, false);
  }

  //tripod gait
  //avr::cout << "tripodGait" << '\n';
  for(uint8_t i = 0; i < 2; ++i) {
    hexapod.moveLinear(tripodGait, 0, false);
  }

  //ripple gait
  //avr::cout << "trippleGait" << '\n';
  hexapod.moveLinear(rippleGait, 0, false, true, false);
  for(uint8_t i = 0; i < 1; ++i) {
    hexapod.moveLinear(rippleGait, 0, false);
  }
  hexapod.moveLinear(rippleGait, 0, false, false, true);
}
