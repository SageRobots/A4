#ifndef GRIPPER_H
#define GRIPPER_H

#include <Arduino.h>

class gripper {
public:
	int pin1, pin2, pinOpen;
	bool opening, open;
  int power = 0;
	unsigned long timeGrip;
	gripper(int in1, int in2, int open);
	void actuate(int pow);
  void update();
};

#endif
