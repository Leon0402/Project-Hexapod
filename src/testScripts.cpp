#include "TestScripts.h"

void bodyIk_test(Hexapod& hexapod) {
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
  for(auto point: points) {
    hexapod.bodyIk(point[0], point[1], point[2]);
  }
}

void moveLinear_test(Hexapod& hexapod) {
  //wave gait
  avr::cout << "waveGait" << '\n';
  hexapod.moveLinear(waveGait, 0, true);

  //ripple gait
  avr::cout << "rippleGait" << '\n';
  hexapod.moveLinear(rippleGait, 0, true, true, false);
  hexapod.moveLinear(rippleGait, 0, true);
  hexapod.moveLinear(rippleGait, 0, true, false, true);

  //tripod gait
  avr::cout << "tripodGait" << '\n';
  hexapod.moveLinear(tripodGait, 0, true);
}
