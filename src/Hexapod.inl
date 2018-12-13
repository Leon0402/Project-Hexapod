#ifndef HEXAPOD_INL
#define HEXAPOD_INL

#include <util/delay.h>

namespace {
  inline void doCalculations(const Leg& leg, const Point<int16_t>& localPosition, uint8_t currentPhase, uint8_t swingPhaseCycles, uint8_t stancePhaseCycles,
  float slope, bool moveUpwards, uint8_t steps, Point<int16_t>& destination, LinearFunction& linearFunction, QuadraticFunction& quadraticFunction, float& nextStep, float& nextStepInaccuracy) {
    linearFunction = leg.getLinearFunction(slope);

    if(currentPhase) {
      destination = leg.getLastLinearPoint(linearFunction, slope, moveUpwards);
      quadraticFunction = leg.getQuadraticFunction(destination, -75, 0);
      nextStep = (destination.x - localPosition.x)/static_cast<float>(swingPhaseCycles*steps);
    } else {
      destination = leg.getLastLinearPoint(linearFunction, slope, !moveUpwards);
      nextStep = (destination.x - localPosition.x)/static_cast<float>(stancePhaseCycles*steps);
    }
    nextStepInaccuracy = 0.0f;
  }
}

template<uint8_t gaitStartSequenceSize, uint8_t gaitPatternSize, uint8_t gaitEndSequenceSize>
void Hexapod::moveLinear(const Gait<gaitStartSequenceSize, gaitPatternSize, gaitEndSequenceSize>& gait, float slope, bool moveUpwards, bool startSequenze, bool endSequenze) {
  //create array out of the startSequenze, pattern and endSequenze
  const uint8_t patternSize = gait.getCompleteCycleSize(startSequenze, endSequenze);
  uint8_t pattern[patternSize];
  gait.getCompleteCycle(pattern, startSequenze, endSequenze);

  constexpr uint8_t intermediateSteps = 15;
  static LinearFunction linearFunctions[6];
  static QuadraticFunction quadraticFunctions[6];
  static Point<int16_t> destination[6];
  static float nextSteps[6];
  static float nextStepInaccuracies[6];

  static float oldSlope = slope*(-1);
  static bool oldMoveUpwards = !moveUpwards;

  //determine if calculations have to be done, don't do them if nothing has changed and the leg has
  //not reached its destination point
  if(oldMoveUpwards != moveUpwards || oldSlope != slope) {
    for(uint8_t k = 0; k < 6; ++k) {
      doCalculations(this->legs[k], this->legs[k].getLocalPosition(), (pattern[0] >> k) & 0x01, gait.getSwingPhaseCycles(),
      gait.getStancePhaseCycles(), slope, moveUpwards, intermediateSteps, destination[k], linearFunctions[k], quadraticFunctions[k], nextSteps[k], nextStepInaccuracies[k]);
    }
    oldSlope = slope;
    oldMoveUpwards = moveUpwards;
  } else {
    for(uint8_t k = 0; k < 6; ++k) {
      Point<int16_t> localPosition = this->legs[k].getLocalPosition();
      if(localPosition == destination[k]) {
        doCalculations(this->legs[k], this->legs[k].getLocalPosition(), (pattern[0] >> k) & 0x01, gait.getSwingPhaseCycles(),
        gait.getStancePhaseCycles(), slope, moveUpwards, intermediateSteps, destination[k], linearFunctions[k], quadraticFunctions[k], nextSteps[k], nextStepInaccuracies[k]);
      }
    }
  }

  for(uint8_t i = 0; i < patternSize; ++i) {
    for(uint8_t j = 0; j < intermediateSteps; ++j) {
      for(uint8_t k = 0; k < 6; ++k) {
        Point<int16_t> localPosition = this->legs[k].getLocalPosition();
        uint8_t currentPhase = (pattern[i] >> k) & 0x01;

        //determine if the phase has changed and calculations have to be done new
        if(j == 0) {
          if (i != 0 && (((pattern[i-1] >> k) & 0x01) != currentPhase)) {
            doCalculations(this->legs[k], localPosition, currentPhase, gait.getSwingPhaseCycles(),
            gait.getStancePhaseCycles(), slope, moveUpwards, intermediateSteps, destination[k], linearFunctions[k], quadraticFunctions[k], nextSteps[k], nextStepInaccuracies[k]);
          }
        }

        //calculates x and avoids inaccuracies
        int16_t nextStepRounded = round(nextSteps[k] + nextStepInaccuracies[k]);
        nextStepInaccuracies[k] = nextSteps[k] + nextStepInaccuracies[k] - nextStepRounded;
        int16_t x = localPosition.x + nextStepRounded;
        //check that leg is never behind its bounds
        if((nextStepRounded > 0 && x > destination[k].x) || (nextStepRounded < 0 && x < destination[k].x) || nextSteps[k] == 0) {
          //Wenn an Grenze, dann sollte automatisch nextStep = 0 sein? ... quadratische Funktion kann bei Sprung nicht
          //bestimmt werden (muss übersprungen werden hier)
          this->moveToLocalPoint(static_cast<LegPosition>(k), destination[k]);
          continue;
        }

        //calculates next Position
        Point<int16_t> nextPosition {x, linearFunctions[k].getY(x), localPosition.z};
        if(currentPhase) {
          nextPosition.z = quadraticFunctions[k].getY(x);
        }
        //moves the leg to the next Position
        this->moveToLocalPoint(static_cast<LegPosition>(k), nextPosition);
        _delay_ms(1);
      }
    }
  }
}

#endif //HEXAPOD_INL
