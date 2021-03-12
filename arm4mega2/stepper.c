#include "stepper.h"

void stepperSetup(int pinEnable, int pinStep, int pinDir, int pinEncA, int pinEncB, int pinHome) {
  pinMode(pinEnable, OUTPUT);
  pinMode(pinStep, OUTPUT);
  pinMode(pinDir, OUTPUT);
  pinMode(pinEncA, INPUT);
  pinMode(pinEncB, INPUT);
  pinMode(pinHome, INPUT_PULLUP);
}
