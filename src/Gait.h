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

extern Gait<0, 6, 0> waveGait;
extern Gait<1, 6, 1> rippleGait;
extern Gait<0, 2, 0> tripodGait;

template<uint8_t startSequenzeSize, uint8_t patternSize, uint8_t endSequenzeSize>
Gait<startSequenzeSize, patternSize, endSequenzeSize>::Gait(uint8_t (&startSequenze)[startSequenzeSize], uint8_t (&pattern)[patternSize], uint8_t (&endSequenze)[endSequenzeSize],
uint8_t swingPhaseCycles, uint8_t stancePhaseCycles, bool startSequenzeInclusive, bool endSequenzeInclusive)
: startSequenze {startSequenze}, pattern {pattern}, endSequenze {endSequenze}, swingPhaseCycles {swingPhaseCycles},
  stancePhaseCycles {stancePhaseCycles}, startSequenzeInclusive {startSequenzeInclusive}, endSequenzeInclusive {endSequenzeInclusive} {}

template<uint8_t startSequenzeSize, uint8_t patternSize, uint8_t endSequenzeSize>
uint8_t Gait<startSequenzeSize, patternSize, endSequenzeSize>::getCompleteCycleSize(bool useStartSequenze, bool useEndSequenze) const {
  uint8_t size = patternSize;
  if(useStartSequenze && !this->startSequenzeInclusive) {
    size += startSequenzeSize;
  }
  if(useEndSequenze && !this->endSequenzeInclusive) {
    size += endSequenzeSize;
  }
  return size;
}

template<uint8_t startSequenzeSize, uint8_t patternSize, uint8_t endSequenzeSize>
void Gait<startSequenzeSize, patternSize, endSequenzeSize>::getCompleteCycle(uint8_t* cycle, bool useStartSequenze, bool useEndSequenze) const {
  //Add the start sequence to cycle
  if(useStartSequenze) {
    for(uint8_t i = 0; i < startSequenzeSize; ++i) {
      *cycle++ = this->startSequenze[i];
    }
  }

  //decide wether the start/endSequenze should replace parts of the actual pattern
  uint8_t patternStartIndex = 0;
  uint8_t patternEndIndex = 0;
  if(useStartSequenze && this->startSequenzeInclusive) {
    patternStartIndex = startSequenzeSize;
  }
  if(useEndSequenze && endSequenzeInclusive) {
    patternEndIndex = endSequenzeSize;
  }

  //Add the pattern to the cycle
  for(uint8_t i = patternStartIndex; i < patternSize - patternEndIndex; ++i) {
    *cycle++ = this->pattern[i];
  }

  //Add the end sequance to the cycle
  if(useEndSequenze) {
    for(uint8_t i = 0; i < endSequenzeSize; ++i) {
      *cycle++ = this->endSequenze[i];
    }
  }
}

template<uint8_t startSequenzeSize, uint8_t patternSize, uint8_t endSequenzeSize>
uint8_t Gait<startSequenzeSize, patternSize, endSequenzeSize>::getSwingPhaseCycles() const {
  return this->swingPhaseCycles;
}

template<uint8_t startSequenzeSize, uint8_t patternSize, uint8_t endSequenzeSize>
uint8_t Gait<startSequenzeSize, patternSize, endSequenzeSize>::getStancePhaseCycles() const {
  return this->stancePhaseCycles;
}

#endif //GAITS_H
