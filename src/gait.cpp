#include "Gait.h"

<<<<<<< HEAD
namespace {
  uint8_t waveStartSequence[0] {};
  uint8_t wavePattern[6] {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000};
  uint8_t waveEndSequence[0] {};

  uint8_t rippleStartSequence[1] {0b00000001};
  uint8_t ripplePattern[6] {0b00010001, 0b00100001, 0b00100010, 0b00001010, 0b00001100, 0b00010100};
  uint8_t rippleEndSequence[1] {0b00010000};

  uint8_t tripodStartSequence[0] {};
  uint8_t tipodPattern[2] {0b00010101, 0b00101010};
  uint8_t tripodEndSequence[0] {};
}

Gait<0, 6, 0> waveGait {waveStartSequence, wavePattern, waveEndSequence, 1, 5};
Gait<1, 6, 1> rippleGait {rippleStartSequence, ripplePattern, rippleEndSequence, 2, 4, true, false};
Gait<0, 2, 0> tripodGait {tripodStartSequence, tipodPattern, tripodEndSequence, 1, 1};
=======
Gait<6, 0, 0> waveGait = {{}, {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000}, {}, 1, 5};
Gait<6, 1, 1> rippleGait = {{0b00000001},
                            {0b00010001,
                             0b00100001,
                             0b00100010,
                             0b00001010,
                             0b00001100,
                             0b00010100},
                            {0b00010000}, 2, 4};
Gait<2, 0, 0> tripodGait = {{},{0b00010101, 0b00101010}, {}, 1, 1};
>>>>>>> 9060e447d636b443dcfa18bc2e84334e92c8f3ce
