
#include "stepper.h"

//define variables for pins
const int pinGripDir = 10;
const int pinGripPow = 11;

unsigned long timeNow, timeGrip, timeWait;
unsigned long timePrint, timePrevSpeed;
volatile uint32_t tooFast = 0;
bool timeWaitSet;

volatile uint8_t busy = 0;
uint8_t master, slave1, slave2, slave3, masterStepBit, slave1StepBit, slave2StepBit, slave3StepBit;
volatile uint8_t dirBits, stepBits;
float speedMax = 90;
float speedMin = 20;
float speedNow;
float accel = 30;
uint32_t accelSteps;
float x, t;
uint32_t iTemp;
volatile int32_t axisToHome = 9;
int32_t gripperState = 0;
int32_t gripperPow = 0;
int32_t wait = 0;

enum motionType {
  noMove,
  accelMove,
  holdMove,
  decelMove
};

motionType motionType;

struct command {
  bool moveAxis[4];
  int32_t targetPulses[4];
  int32_t axisToHome;
  int32_t gripperState;
  int32_t gripperPow;
  bool set;
  bool complete;
  int32_t wait;
  bool waitComplete;
};
command commandBuffer[10];
uint8_t head, tail, nextHead, nextTail;

//create array of stepper class
stepper steppers[4];

//ports and bits for writing direction and step ports in 1 command
#define DIR_PORT PORTB
const uint8_t dirBit[4] = {1<<0, 1<<1, 1<<2, 1<<3};
const uint8_t dirMask = dirBit[0] | dirBit[1] | dirBit[2] | dirBit[3];

#define STEP_PORT PORTC
const uint8_t stepBit[4] = {1<<0, 1<<1, 1<<2, 1<<3};
const uint8_t stepMask = stepBit[0] | stepBit[1] | stepBit[2] | stepBit[3];

void setup() {
  initializeBuffer();
  //stepper objects (int enable, int step, int dir, int encA, int encB, int home, int pulsesPerRev, int stepsPerRev)
  steppers[0] = stepper(A8, 37, 53, 21, 13, A0, 1000, 200, 26+103.0/121.0);
  steppers[1] = stepper(A1, 36, 52, 20, 17, 4, 1000, 200, 26+103.0/121.0);
  steppers[2] = stepper(A10, 35, 51, 19, A14, 24, 1000, 200, 26+103.0/121.0);
  steppers[3] = stepper(49, 34, 50, 18, 25, 29, 300, 200, 26+103.0/121.0);

  //stepper setup
  //configure timer 1 as stepper driver input, CTC, no prescaler
  TCCR1A = 0;
  TCCR1B |= (1<<WGM12)|(1<<CS10);
  TIMSK1 = 0;
  //configure timer 3 as stepper port reset
  TCCR3A = 0;
  TCCR3B |= (1<<WGM32)|(1<<CS30);
  TIMSK3 = 0;

  //attach interrupts to encoder A channels
  attachInterrupt(digitalPinToInterrupt(steppers[0].pinEncA), encoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(steppers[1].pinEncA), encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(steppers[2].pinEncA), encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(steppers[3].pinEncA), encoder3, RISING);

  //gripper setup
  pinMode(pinGripDir, OUTPUT);
  pinMode(pinGripPow, OUTPUT);

  //serial setup
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");
}

void loop() {
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

  //check if wait time is complete
  if(timeNow-timeWait >= commandBuffer[tail].wait) {
    commandBuffer[tail].waitComplete = 1;
  }

  //execute command if tail is complete and nextTail is set
  if(commandBuffer[nextTail].set && commandBuffer[tail].complete && commandBuffer[tail].waitComplete) {
    commandBuffer[tail].complete = false;
    commandBuffer[tail].waitComplete = false;
    commandBuffer[tail].set = false;
    tail = nextTail;
    if(nextTail == 9) {
      nextTail = 0;
    } else {
      nextTail++;
    }
    executeCommand();
  }

  timeNow = millis();

  //run the gripper
  gripper();

  //alter the speed
  if ((motionType > noMove) && (timeNow - timePrevSpeed >= 100)) {
    //accelerate or hold at least halfway, then until stepsToTarget <= accelSteps
    if (motionType == accelMove) { //accelerate
      speedNow += (timeNow-timePrevSpeed)/1000.0*accel;
      if (speedNow > speedMax) {
        speedNow = speedMax;
        accelSteps = abs(steppers[master].stepsToTarget - steppers[master].stepsNeeded);
        motionType = holdMove;
      }
      if (steppers[master].fractionRemaining <= 0.5) {
        accelSteps = abs(steppers[master].stepsToTarget - steppers[master].stepsNeeded);
        motionType = holdMove;
      }
    } else if (motionType == holdMove) {
      if (abs(steppers[master].stepsToTarget) <= accelSteps) motionType = decelMove;
    } else if (motionType == decelMove) {
        speedNow -= (timeNow-timePrevSpeed)/1000.0*accel;
        if (speedNow < speedMin) speedNow = speedMin;
    }
    iTemp = 1000000/(speedNow*steppers[master].stepsPerDeg);
    timePrevSpeed = timeNow;
  }
  
  //periodically print the position
  if (timeNow - timePrint >= 2000) {
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
    timePrint = timeNow;
  }
}

void encoder0() {
  steppers[0].encoder();
}

void encoder1() {
  steppers[1].encoder();
}

void encoder2() {
  steppers[2].encoder();
}

void encoder3() {
  steppers[3].encoder();
}

void parseLine(char *line) {

  uint8_t i,j;
  char cmd[4];
  cmd[3] = '\0';
  char number[10];
  double fValue;
  uint8_t c;
  i=0;
  //initialize commandBuffer
  commandBuffer[head].gripperState = 0;
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
      commandBuffer[head].moveAxis[0] = 1;
    } else if (strcmp(cmd,"M1A") == 0) {
      //absolute move of J1
      commandBuffer[head].targetPulses[1] = round(fValue*steppers[1].pulsesPerDegOut);
      commandBuffer[head].moveAxis[1] = 1;
    } else if (strcmp(cmd,"M2A") == 0) {
      //absolute move of J2
      commandBuffer[head].targetPulses[2] = round(fValue*steppers[2].pulsesPerDegOut);
      commandBuffer[head].moveAxis[2] = 1;
    } else if (strcmp(cmd,"M3A") == 0) {
      //absolute move of J3
      commandBuffer[head].targetPulses[3] = round(fValue*steppers[3].pulsesPerDegOut);
      commandBuffer[head].moveAxis[3] = 1;
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
      if (fValue > 0) {
        commandBuffer[head].gripperState = 1;
        commandBuffer[head].gripperPow = fValue;
      } else {
        commandBuffer[head].gripperState = -1;
        commandBuffer[head].gripperPow = -1*fValue;
      }
    } else if (strcmp(cmd,"DEL") == 0) {
      commandBuffer[head].wait = fValue;
    } else {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
    }
  }
  commandBuffer[head].set = 1;
}

void executeCommand() {
  //copy data out of buffer
  timeWait = timeNow;
  axisToHome = commandBuffer[tail].axisToHome;
  for(int i=0; i<4; i++) {
    if(commandBuffer[tail].moveAxis[i]) {
      steppers[i].targetPulses = commandBuffer[tail].targetPulses[i];
      steppers[i].inPosition = false;
    }
    if (axisToHome == i) {
      steppers[i].currentPulses = 0;
      steppers[i].targetPulses = round(steppers[i].pulsesPerRev*steppers[i].gearRatio);
      steppers[i].inPosition = false;
    }
  }
  
  gripperState = commandBuffer[tail].gripperState;
  gripperPow = commandBuffer[tail].gripperPow;
  
  //axis with most steps is master
  int iMax = 0;
  for(int i=0; i<4; i++) {
    steppers[i].stepsNeeded = steppers[i].computeStepsToTarget();
    if(abs(steppers[i].stepsNeeded) > abs(steppers[iMax].stepsNeeded)) {
      iMax = i;
    }
  }

  //assign master and slave indices
  master = 0;
  slave1 = 1;
  slave2 = 2;
  slave3 = 3;
  masterStepBit = stepBit[0];
  slave1StepBit = stepBit[1];
  slave2StepBit = stepBit[2];
  slave3StepBit = stepBit[3];
  if(iMax == 1) {
    master = 1;
    slave1 = 0;
    masterStepBit = stepBit[1];
    slave1StepBit = stepBit[0];
  } else if (iMax == 2) {
    master = 2;
    slave2 = 0;
    masterStepBit = stepBit[2];
    slave2StepBit = stepBit[0];
  } else {
    master = 3;
    slave3 = 0;
    masterStepBit = stepBit[3];
    slave3StepBit = stepBit[0];
  }
  
  //enable interrupt to start the move
  //set interrupt time based on speedMin
  speedNow = speedMin;
  iTemp = 1000000/(speedNow*steppers[master].stepsPerDeg); //speedMin (deg/s), stepsPerDeg, 1000000 us/s
  if (axisToHome == 9) {
    motionType = 1;
  } else {
    motionType = 0;
  }
  timePrevSpeed = timeNow;
  TIMSK1 |= 1<<OCIE1A;
}

ISR(TIMER1_COMPA_vect) {
  //avoid reentering interrupt
  if(busy) {
    //flag this value as too fast
    tooFast = OCR1A;
    return;
  }
  busy = 1;
  //this interrupt takes a long time to run, so we re-enable other interrupts here
  sei();
  OCR1A = iTemp;
  //compute steps to target from current position
  for(int i=0; i<4; i++) {
    steppers[i].computeStepsToTarget();
    if (steppers[i].stepsToTarget == 0) {
      steppers[i].inPosition = true;
    }
  }
  if (steppers[0].inPosition && steppers[1].inPosition && steppers[2].inPosition && steppers[3].inPosition) {
    busy = 0;
    TIMSK1 = 0;
    motionType = 0;
    commandBuffer[tail].complete = 1;
    return;
  }
  for(int i=0; i<4; i++) {
    if (axisToHome == i && !digitalRead(steppers[i].pinHome)) {
      steppers[i].currentPulses = 0;
      steppers[i].targetPulses = 0;
      busy = 0;
      TIMSK1 = 0;
      axisToHome = 9;
      commandBuffer[tail].complete = 1;
      return;
    }
  }

  //set direction 0
  for(int i=0; i<4; i++) {
    if (steppers[i].stepsToTarget < 0) {
      dirBits |= dirBit[i];
    } else {
      dirBits &= ~dirBit[i];
    }
  }

  //write direction port
  DIR_PORT = (DIR_PORT & ~dirMask) | (dirBits & dirMask);

  for(int i=0; i<4; i++) {
    steppers[i].computeFractionRemaining();
  }
  //set the step bits based on pulses
  stepBits = 0;
  if (steppers[master].stepsToTarget != 0) {
    stepBits |= masterStepBit;
  }
  //slave axes follow completion percentage
  if (steppers[slave1].fractionRemaining >= steppers[master].fractionRemaining) {
    stepBits |= slave1StepBit;
  }
  if (steppers[slave2].fractionRemaining >= steppers[master].fractionRemaining) {
    stepBits |= slave2StepBit;
  }
  if (steppers[slave3].fractionRemaining >= steppers[master].fractionRemaining) {
    stepBits |= slave3StepBit;
  }

  //write the step port
  STEP_PORT = (STEP_PORT & ~stepMask)|(stepBits & stepMask);

  //enable the pulse reset timer
  OCR3A = 100;
  TIMSK3 |= 1<<OCIE3A;
  busy = 0;
}

ISR(TIMER3_COMPA_vect) {
  //reset step pins
  STEP_PORT = STEP_PORT & ~stepMask;
  //disable timer
  TIMSK3 = 0;
}

void gripper() {
  if(gripperState == 1) {
    digitalWrite(pinGripDir, 1);
    analogWrite(pinGripPow, 255-gripperPow);
    gripperState = 2;
    timeGrip = timeNow;
  } else if (gripperState == -1) {
    digitalWrite(pinGripDir, 0);
    analogWrite(pinGripPow, gripperPow);
    gripperState = -2;
    timeGrip = timeNow;
  }
  if (timeNow - timeGrip >= 4000) {
    digitalWrite(pinGripDir, 0);
    digitalWrite(pinGripPow, 0);
    gripperState = 0;
  }
}

void initializeBuffer() {
  head = 1;
  tail = 0;
  nextHead = 2;
  nextTail = 1;
  commandBuffer[tail].complete = true;
  commandBuffer[tail].waitComplete = true;
  commandBuffer[nextTail].set = false;
}
