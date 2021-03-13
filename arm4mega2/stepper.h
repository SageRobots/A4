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

//void stepperSetup(int pinEnable, int pinStep, int pinDir, int pinEncA, int pinEncB, int pinHome);
//void enable(int pinEnable);
//void encoder0();
//void encoder1();
//void encoder2();
//void encoder3();

#endif
