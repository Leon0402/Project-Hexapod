#ifndef GAIT_INL
#define GAIT_INL

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

#endif //GAIT_INL
