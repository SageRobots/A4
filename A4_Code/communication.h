#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "stepper.h"
#include "gripper.h"

class com {
	public:
		struct command {
		  bool moveAxis[4];
		  int32_t targetPulses[4];
		  int axisToHome;
		  int gripperPow;
		  bool set;
		  bool complete;
		  unsigned long wait;
      unsigned long waitStart;
		  bool waitComplete;
		};
		command commandBuffer[10];
		uint8_t head, tail, nextHead, nextTail;

		volatile uint32_t tooFast = 0;
		stepper *steppers;
    gripper *myGripper;

    float speedMax = 100;
    float accel = 100;
    bool execute=false;

		com(stepper _steppers[4], gripper* _gripper);
    void initializeBuffer();
		void readSerial();
		void parseLine(char* line);
		void printStatus();
    void updateBuffer(unsigned long timeNow);
    void setComplete();
};

#endif
