#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>

class stepper {
  public:
    volatile int32_t currentPulses = 0, targetPulses = 0, stepsToTarget;
    int32_t stepsNeeded;
    float stepsPerPulse, gearRatio, stepsPerDeg, pulsesPerDegOut;
    int pulsesPerRev, stepsPerRev;
    volatile float fractionRemaining;
    int pinEnable, pinStep, pinDir, pinEncA, pinEncB, pinHome;
    volatile bool inPosition = false;
    bool enabled = true;
    stepper();
    stepper(int enable, int step, int dir, int encA, int encB, int home, int _pulsesPerRev, int _stepsPerRev, float _gearRatio);
    void enable();
    void disable();
    void encoder();
    int computeStepsToTarget();
    void computeFractionRemaining();
    float computePosition();
    float computeTarget();
};

#endif
