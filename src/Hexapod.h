#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "Servocontroller.h"
#include "Leg.h"
#include "Gait.h"

#include "Stream.h"
#include <util/delay.h>

enum class LegPosition { FrontLeft, MiddleLeft, RearLeft, FrontRight, MiddleRight, RearRight};

class Hexapod {
public:
  /*!
  @brief  Instanstiates the Hexapod with it's legs and servocontrollers
  */
  Hexapod();


  /*!
  @brief Updates all legs
  @param currentMillis time passed in milliseconds
  */
  void update(uint32_t currentMillis);

  /*!
  @brief Moves Linear
  */
  template<uint8_t gaitStartSequenceSize, uint8_t gaitPatternSize, uint8_t gaitEndSequenceSize>
  void moveLinear(const Gait<gaitStartSequenceSize, gaitPatternSize, gaitEndSequenceSize>& gait, float slope, bool moveUpwards, bool startSequenze = false, bool endSequenze = false);

  /*
  @brief Complete body rotation
  @param yawAngle The yaw angle
  @param pitchAngle The pitch angle
  @param rollAngle The roll angle
  */
  void bodyIk(int8_t yawAngle, int8_t pitchAngle, int8_t rollAngle);

  /*
  @brief Body movement: yaw
  @param angle The yaw angle
  */
  void yaw(int8_t angle);

  /*!
  @brief Body movement: pitch
  @param angle The pitch angle
  */
  void pitch(int8_t angle);

  /*!
  @brief Body movement: roll
  @param angle The roll angle
  */
  void roll(int8_t angle);

  /*!
  @brief Moves a leg to a point in its local coordinate system in the given time
  @param legPosition Position of the leg, which should move (see LegPosition Enum)
  @param destination Point in the local coordinate system of the leg the leg should move to
  @param time The time the leg needs to move
  */
  void moveToLocalPoint(const LegPosition& legPosition, const Point<int16_t>& destination, uint16_t time = 0);

  /*!
  @brief Moves a leg to a point in the global coordinate system in the given time
  @param legPosition Position of the leg, which should move (see LegPosition Enum)
  @param destination Point in the global coordinate system the leg should move to
  @param time The time the leg needs to move
  */
  void moveToGlobalPoint(const LegPosition& legPosition, const Point<int16_t>& destination, uint16_t time = 0);

private:
  void move(const LegPosition& legPosition, uint16_t time = 0);

  int8_t yawAngle;
  int8_t pitchAngle;
  int8_t rollAngle;

  Servocontroller servocontroller1;
  Servocontroller servocontroller2;
  Leg legs[6];
};

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
  static bool firstCall = true;

  if(firstCall) {
    for(uint8_t k = 5; k < 6; ++k) {
      doCalculations(this->legs[k], this->legs[k].getLocalPosition(), (pattern[0] >> k) & 0x01, gait.getSwingPhaseCycles(),
      gait.getStancePhaseCycles(), slope, moveUpwards, intermediateSteps, destination[k], linearFunctions[k], quadraticFunctions[k], nextSteps[k], nextStepInaccuracies[k]);
    }
    firstCall = false;
  }

  for(uint8_t i = 0; i < patternSize; ++i) {
    for(uint8_t j = 0; j < intermediateSteps; ++j) {
      for(uint8_t k = 5; k < 6; ++k) {
        Point<int16_t> localPosition = this->legs[k].getLocalPosition();
        uint8_t currentPhase = (pattern[i] >> k) & 0x01;

        //determine if the phase has changed and calculations have to be done new
        if(j == 0) {
          if(i == 0) {
            //Am Ziel angekommen beim letzten Funktionsaufruf
            //Probleme: Andere Methode, die lokale Position verändert, wurde zwischendrin aufgerufen
            //Probleme: Spart Berechnungen für else Zweig, dafür viele static Variablen
            if(localPosition == destination[k]) {
              doCalculations(this->legs[k], localPosition, currentPhase, gait.getSwingPhaseCycles(),
              gait.getStancePhaseCycles(), slope, moveUpwards, intermediateSteps, destination[k], linearFunctions[k], quadraticFunctions[k], nextSteps[k], nextStepInaccuracies[k]);
            }
          } else if (((pattern[i-1] >> k) & 0x01) != currentPhase) {
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
        //  avr::cout << "Jump" << '\n';
          nextPosition.z = quadraticFunctions[k].getY(x);
        }
        //avr::cout << k << ": " << nextPosition << '\n';
        //moves the leg to the next Position
        this->moveToLocalPoint(static_cast<LegPosition>(k), nextPosition);
      }
    }
  }
}

inline void doCalculations(const Leg& leg, const Point<int16_t>& localPosition, uint8_t currentPhase, uint8_t swingPhaseCycles, uint8_t stancePhaseCycles,
float slope, bool moveUpwards, uint8_t steps, Point<int16_t>& destination, LinearFunction& linearFunction, QuadraticFunction& quadraticFunction, float& nextStep, float& nextStepInaccuracy) {
  linearFunction = leg.getLinearFunction(slope);

  if(currentPhase) {
    avr::cout << "JUMP" << '\n';
    destination = leg.getLastLinearPoint(linearFunction, slope, moveUpwards);
    quadraticFunction = leg.getQuadraticFunction(destination, -75, 0);
    nextStep = (destination.x - localPosition.x)/static_cast<float>(swingPhaseCycles*steps);
  } else {
    avr::cout << "STANCE" << '\n';
    destination = leg.getLastLinearPoint(linearFunction, slope, !moveUpwards);
    nextStep = (destination.x - localPosition.x)/static_cast<float>(stancePhaseCycles*steps);
  }
  nextStepInaccuracy = 0.0f;
  avr::cout << "NextStept: " << nextStep << '\n';
  avr::cout << "Destination: " << destination << '\n';
}
#endif //HEXAPOD_H
