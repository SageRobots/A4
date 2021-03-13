#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "stepper.h"

class com {
	private:
		float speedMax = 90;
		float speedMin = 20;
		float speedNow;
		float accel = 30;

	public:
		struct command {
		  bool moveAxis[4];
		  int32_t targetPulses[4];
		  int32_t axisToHome;
		  int32_t gripperState;
		  int32_t gripperPow;
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

		com(stepper _steppers[4]);
    void initializeBuffer();
		void readSerial();
		void parseLine(char* line);
		void printStatus();
    void updateBuffer();
    void setComplete();
};

#endif
