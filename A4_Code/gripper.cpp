#include "gripper.h"

gripper::gripper(int in1, int in2, int open) {
	pin1 = in1;
	pin2 = in2;
	pinOpen = open;
	pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pinOpen, INPUT_PULLUP);
}

void gripper::actuate(int pow) {
  power = pow;
	if (pow > 0) {
		// close the gripper
		analogWrite(pin2, 255-pow);
    digitalWrite(pin1, HIGH);
    opening = false;
	} else if (pow < 0) {
		//open the gripper
		digitalWrite(pin2, HIGH);
		analogWrite(pin1, 255+pow);
		opening = true;
		timeGrip = millis();
	} else {
		digitalWrite(pin1, LOW);
		digitalWrite(pin2, LOW);
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
