#include "gripper.h"

gripper::gripper(int dir, int pow, int open) {
	pinDir = dir;
	pinPow = pow;
	pinOpen = open;
	pinMode(pinDir, OUTPUT);
  pinMode(pinPow, OUTPUT);
  pinMode(pinOpen, INPUT_PULLUP);
}

void gripper::actuate(int pow) {
	if (pow > 0) {
		// close the gripper
		digitalWrite(pinDir, HIGH);
    analogWrite(pinPow, 255-pow);
    opening = false;
	} else if (pow < 0) {
		//open the gripper
		digitalWrite(pinDir, LOW);
		analogWrite(pinPow, -pow);
		opening = true;
		timeGrip = millis();
	} else {
		digitalWrite(pinDir, LOW);
		digitalWrite(pinPow, LOW);
	}
}

void gripper::update() {
	open = !digitalRead(pinOpen);
	//if opening and open sensor on, turn motor off
	if (opening) {
		if (open) {
			opening = false;
			actuate(0);
		} else if (millis()-timeGrip > 5000) {
			opening = false;
			actuate(0);
		}
	}
}
