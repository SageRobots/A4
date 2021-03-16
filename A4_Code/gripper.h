#ifndef GRIPPER_H
#define GRIPPER_H

#include <Arduino.h>

class gripper {
public:
	int pinDir, pinPow, pinOpen;
	bool opening, open;
	unsigned long timeGrip;
	gripper(int dir, int pow, int open);
	void actuate(int pow);
  void update();
};

#endif
