#include "stepper.h"

stepper::stepper(){}
stepper::stepper(int enable, int step, int dir, int encA, int encB, int home, int pulsesPerRev, int stepsPerRev) {
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
  stepsPerPulse = (float)stepsPerRev/(float)pulsesPerRev;
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
