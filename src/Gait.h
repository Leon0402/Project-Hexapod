#ifndef GAITS_H
#define GAITS_H

#ifndef X86_64
  #include <inttypes.h>
#else
  #include <cstdint>
#endif

template<uint8_t PATTERN_SIZE, uint8_t START_SEQUENZE_SIZE, uint8_t END_SEQUENZE_SIZE>
struct Gait {
  uint8_t startSequenze[START_SEQUENZE_SIZE];
  uint8_t pattern[PATTERN_SIZE];
  uint8_t endSequenze[END_SEQUENZE_SIZE];
  uint8_t swingPhaseCycles;
  uint8_t stancePhaseCycles;
};

extern Gait<6, 0, 0> waveGait;
extern Gait<6, 1, 1> rippleGait;
extern Gait<2, 0, 0> tripodGait;

#endif //GAITS_H
