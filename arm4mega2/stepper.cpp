#include "stepper.h"

stepper::stepper(){}
stepper::stepper(int enable, int step, int dir, int encA, int encB, int home, int _pulsesPerRev, int _stepsPerRev, float _gearRatio) {
  //assign the pins
  pinEnable = enable;
  pinStep = step;
  pinDir = dir;
  pinEncA = encA;
  pinEncB = encB;
  pinHome = home;

  //set the pin modes
  pinMode(pinEnable, OUTPUT);
  pinMode(pinStep, OUTPUT);
  pinMode(pinDir, OUTPUT);
  pinMode(pinEncA, INPUT);
  pinMode(pinEncB, INPUT);
  pinMode(pinHome, INPUT_PULLUP);

  //compute the stepsPerPulse
  stepsPerPulse = (float)_stepsPerRev/(float)_pulsesPerRev;
  pulsesPerRev = _pulsesPerRev;
  stepsPerRev = _stepsPerRev;
  gearRatio = _gearRatio;
  stepsPerDeg = stepsPerRev*gearRatio/360.0;
  pulsesPerDegOut = pulsesPerRev*gearRatio/360.0;
}
void stepper::enable() {
  digitalWrite(pinEnable, LOW);
}
void stepper::disable() {
  digitalWrite(pinEnable, HIGH);
}
void stepper::encoder() {
  //if pin B is the same, the motion is ccw positive
  if(digitalRead(pinEncB)) {
    currentPulses++;
  }
  else {
    currentPulses--;
  }
}

int stepper::computeStepsToTarget() {
  stepsToTarget = (targetPulses - currentPulses) * stepsPerPulse;
  return stepsToTarget;
}

void stepper::computeFractionRemaining() {
  fractionRemaining = abs((float)stepsToTarget/(float)stepsNeeded);
}

float stepper::computePosition() {
  return currentPulses/pulsesPerDegOut;
}

float stepper::computeTarget() {
  return targetPulses/pulsesPerDegOut;
}
