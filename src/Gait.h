#ifndef GAITS_H
#define GAITS_H

#ifndef X86_64
  #include <inttypes.h>
#else
  #include <cstdint>
#endif

template<uint8_t startSequenzeSize, uint8_t patternSize, uint8_t endSequenzeSize>
class Gait {

public:
  Gait(uint8_t (&startSequenze)[startSequenzeSize], uint8_t (&pattern)[patternSize], uint8_t (&endSequenze)[endSequenzeSize],
  uint8_t swingPhaseCycles, uint8_t stancePhaseCycles, bool startSequenzeInclusive = false, bool endSequenzeInclusive = false);

  uint8_t getCompleteCycleSize(bool useStartSequenze, bool useEndSequenze) const;

  void getCompleteCycle(uint8_t* cycle, bool useStartSequenze, bool useEndSequenze) const;

  uint8_t getSwingPhaseCycles() const;
  uint8_t getStancePhaseCycles() const;

private:
  const uint8_t (&startSequenze)[startSequenzeSize];
  const uint8_t (&pattern)[patternSize];
  const uint8_t (&endSequenze)[endSequenzeSize];

  const uint8_t swingPhaseCycles;
  const uint8_t stancePhaseCycles;

  const bool startSequenzeInclusive;
  const bool endSequenzeInclusive;
};

#include "Gait.inl"

extern Gait<0, 6, 0> waveGait;
extern Gait<0, 6, 0> wave2Gait;
extern Gait<1, 6, 1> rippleGait;
extern Gait<0, 2, 0> tripodGait;

#endif //GAITS_H
