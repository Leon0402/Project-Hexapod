#ifndef GAITS_H
#define GAITS_H

#ifndef X86_64
  #include <inttypes.h>
#else
  #include <cstdint>
#endif

template<uint8_t PATTERN_SIZE>
struct Gait {
  uint8_t pattern[PATTERN_SIZE];
  uint8_t swingPhaseCycles;
  uint8_t stancePhaseCycles;
};

extern Gait<6> waveGait;

#endif //GAITS_H
