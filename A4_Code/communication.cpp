#include "communication.h"

com::com(stepper _steppers[4]) {
  steppers = _steppers;
  initializeBuffer();
}

void com::initializeBuffer() {
  head = 1;
  tail = 0;
  nextHead = 2;
  nextTail = 1;
  commandBuffer[tail].complete = true;
  commandBuffer[tail].waitComplete = true;
  commandBuffer[nextTail].set = false;
}

void com::readSerial() {
	  //read serial input
  byte c;
  char line[80];
  uint8_t i;
  
  while (Serial.available()) {
    c = Serial.read();
      //set termination character if end of line
      if (c=='\n'||c=='\r') {
        line[i]='\0';
        parseLine(line);
        head = nextHead;
        if (nextHead == 9) {
          nextHead = 0;
        } else {
          nextHead++;
        }
        i=0;
      } else {
        if (c<=' ') {
          //remove white space and control characters
        } else if(c >= 'a' && c <= 'z') {
          //make all uppercase
          line[i++] = c-'a'+'A';
        } else {
          line[i++] = c;
        }
      }
  }
}

void com::parseLine(char* line) {

  uint8_t i,j;
  char cmd[4];
  cmd[3] = '\0';
  char number[10];
  double fValue;
  uint8_t c;
  i=0;
  //set head data to default values
  commandBuffer[head].gripperPow = 0;
  commandBuffer[head].axisToHome = 9;
  commandBuffer[head].moveAxis[0] = 0;
  commandBuffer[head].moveAxis[1] = 0;
  commandBuffer[head].moveAxis[2] = 0;
  commandBuffer[head].moveAxis[3] = 0;
  commandBuffer[head].wait = 0;

  while (line[i] != '\0') {

    //the command type is 3 characters
    cmd[0] = line[i];
    cmd[1] = line[++i];
    cmd[2] = line[++i];
    
    //the command value is all numbers and decimal that follow
    c = line[++i];
    j = 0;
    for(j=0;j<=9;j++) {
      if(((c <= '9') && (c >= '0')) || (c == '.') || (c== '-')) {
        number[j] = c;
        c = line[++i];
      } else {
        number[j] = '\0';
      }
    }
    fValue = atof(number);
    //interpret parsed command
    if (strcmp(cmd,"M0A") == 0) {
      //absolute move of J0
      commandBuffer[head].targetPulses[0] = round(fValue*steppers[0].pulsesPerDegOut);
      commandBuffer[head].moveAxis[0] = true;
    } else if (strcmp(cmd,"M1A") == 0) {
      //absolute move of J1
      commandBuffer[head].targetPulses[1] = round(fValue*steppers[1].pulsesPerDegOut);
      commandBuffer[head].moveAxis[1] = true;
    } else if (strcmp(cmd,"M2A") == 0) {
      //absolute move of J2
      commandBuffer[head].targetPulses[2] = round(fValue*steppers[2].pulsesPerDegOut);
      commandBuffer[head].moveAxis[2] = true;
    } else if (strcmp(cmd,"M3A") == 0) {
      //absolute move of J3
      commandBuffer[head].targetPulses[3] = round(fValue*steppers[3].pulsesPerDegOut);
      commandBuffer[head].moveAxis[3] = true;
    } else if (strcmp(cmd,"SPD") == 0) {
      speedMax = fValue;
    } else if (strcmp(cmd,"ACC") == 0) {
      accel = fValue;
    } else if (strcmp(cmd,"HOM") == 0) {
      commandBuffer[head].axisToHome = fValue;
    } else if (strcmp(cmd,"ENA") == 0) {
      steppers[round(fValue)].enable();
    } else if (strcmp(cmd,"DIS") == 0) {
      steppers[round(fValue)].disable();
    } else if (strcmp(cmd,"STO") == 0) {
      for(int i=0; i<4; i++) {
        steppers[i].currentPulses = steppers[i].targetPulses;
      }
      initializeBuffer();
    } else if (strcmp(cmd,"GRP") == 0) {
      commandBuffer[head].gripperPow = round(fValue);
    } else if (strcmp(cmd,"DEL") == 0) {
      commandBuffer[head].wait = fValue;
    } else {
      Serial.print("\nUnknown command");
    }
  }
  Serial.print("\nCommand: ");
  Serial.print(cmd);
  commandBuffer[head].set = 1;
}

void com::printStatus() {
	if(head != tail) {
	  Serial.print("\nReady");
	}
	Serial.print("\nPos: ");
	for(int i=0; i<4; i++) {
	  Serial.print(steppers[i].computePosition());
	  Serial.print(", ");
	}
	Serial.print("\nTarget: ");
	for(int i=0; i<4; i++) {
	  Serial.print(steppers[i].computeTarget());
	  Serial.print(", ");
	}
	if (tooFast > 0) {
	  Serial.print("\nToo fast: ");
	  Serial.print(tooFast);
	  tooFast = 0;
	}
}

void com::updateBuffer(unsigned long timeNow) {
  //check if wait complete
  if(timeNow-commandBuffer[tail].waitStart >= commandBuffer[tail].wait) {
    commandBuffer[tail].waitComplete = 1;
  }
  //check if command complete
  execute=false;
  if(commandBuffer[nextTail].set && commandBuffer[tail].complete && commandBuffer[tail].waitComplete) {
    commandBuffer[tail].complete = false;
    commandBuffer[tail].waitComplete = false;
    commandBuffer[tail].waitStart = timeNow;
    commandBuffer[tail].set = false;
    tail = nextTail;
    if(nextTail == 9) {
      nextTail = 0;
    } else {
      nextTail++;
    }
    execute=true;
  }
}

void com::setComplete() {
  commandBuffer[tail].complete = true;
  
}
