
#include "stepper.h"

//define variables for pins used
const int pinEncA[4] = {21,20,19,18};
const int pinEncB[4] = {13,17,A14,25};
const int pinEnable[4] = {A8,A1,A10,49};
const int pinDir[4] = {53,52,51,50};
const int pinHome[4] = {A0,4,24,29};
const int pinStep[4] = {37,36,35,34};

const int pinGripDir = 10;
const int pinGripPow = 11;

unsigned long timeNow, timeGrip, timeWait;
unsigned long timePrint, timePrevSpeed;
volatile uint32_t tooFast = 0;
bool timeWaitSet;

const float gearRatio[4] = {26+103.0/121, 26+103.0/121, 26+103.0/121, 26+103.0/121};
int pulsesPerRev[4] = {1000, 1000, 1000, 300}; //pulses per revolution of input shaft
int stepsPerRev[4] = {200, 200, 200, 200}; //steps per revolution of input shaft
const float pulsesPerDeg[4] = {1000.0/360,1000.0/360,1000.0/360,300.0/360}; //gear box input degrees
volatile uint8_t busy = 0;
const float stepsPerDeg[4] = {1/1.8*gearRatio[0]*2, 1/1.8*gearRatio[1]*2, 1/1.8*gearRatio[2]*2, 1/1.8*gearRatio[3]*2}; //~30
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
const uint8_t dirPort = PORTB;
const uint8_t dirBit[4] = {1<<0, 1<<1, 1<<2, 1<<3};
const uint8_t dirMask = dirBit[0] | dirBit[1] | dirBit[2] | dirBit[3];

#define STEP_PORT PORTC
#define STEP_BIT_0 0
#define STEP_BIT_1 1
#define STEP_BIT_2 2
#define STEP_BIT_3 3
#define STEP_MASK ((1<<STEP_BIT_0)|(1<<STEP_BIT_1)|(1<<STEP_BIT_2)|(1<<STEP_BIT_3))

void setup() {
  initializeBuffer();

  for(int i=0; i<4; i++) {
    steppers[i] = stepper(pinEnable[i], pinStep[i], pinDir[i], pinEncA[i], pinEncB[i], pinHome[i], pulsesPerRev[i], stepsPerRev[i]);
  }

  //stepper setup
  //configure timer 1 as stepper driver input, CTC, no prescaler
  TCCR1A = 0;
  TCCR1B |= (1<<WGM12)|(1<<CS10);
  TIMSK1 = 0;
  //configure timer 3 as stepper port reset
  TCCR3A = 0;
  TCCR3B |= (1<<WGM32)|(1<<CS30);
  TIMSK3 = 0;

//  stepperSetup(pinEnable0, pinStep0, pinDir0, pinEnc0A, pinEnc0B, pinHome0);
//  stepperSetup(pinEnable1, pinStep1, pinDir1, pinEnc1A, pinEnc1B, pinHome1);
//  stepperSetup(pinEnable2, pinStep2, pinDir2, pinEnc2A, pinEnc2B, pinHome2);
//  stepperSetup(pinEnable3, pinStep3, pinDir3, pinEnc3A, pinEnc3B, pinHome3);
  
  //attach interrupts to encoder A channels
  attachInterrupt(digitalPinToInterrupt(pinEncA[0]), encoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncA[1]), encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncA[2]), encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncA[3]), encoder3, RISING);

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
    iTemp = 1000000/(speedNow*stepsPerDeg[0]);
    timePrevSpeed = timeNow;
  }
  
  //periodically print the position
  if (timeNow - timePrint >= 2000) {
    if(head != tail) {
      Serial.println("Ready");
    }
    Serial.print("\nPos: ");
    Serial.print(steppers[0].currentPulses/gearRatio[0]/pulsesPerDeg[0]);
    Serial.print(", ");
    Serial.print(steppers[1].currentPulses/gearRatio[1]/pulsesPerDeg[1]);
    Serial.print(", ");
    Serial.print(steppers[2].currentPulses/gearRatio[2]/pulsesPerDeg[2]);
    Serial.print(", ");
    Serial.println(steppers[3].currentPulses/gearRatio[3]/pulsesPerDeg[3]);
    Serial.print("Target: ");
    Serial.print(steppers[0].targetPulses/gearRatio[0]/pulsesPerDeg[0]);
    Serial.print(", ");
    Serial.print(steppers[1].targetPulses/gearRatio[1]/pulsesPerDeg[1]);
    Serial.print(", ");
    Serial.print(steppers[2].targetPulses/gearRatio[2]/pulsesPerDeg[2]);
    Serial.print(", ");
    Serial.println(steppers[3].targetPulses/gearRatio[3]/pulsesPerDeg[3]);
    if (tooFast > 0) {
      Serial.print("Too fast: ");
      Serial.println(tooFast);
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
      commandBuffer[head].targetPulses[0] = round(fValue*pulsesPerDeg[0]*gearRatio[0]);
      commandBuffer[head].moveAxis[0] = 1;
    } else if (strcmp(cmd,"M1A") == 0) {
      //absolute move of J1
      commandBuffer[head].targetPulses[1] = round(fValue*pulsesPerDeg[1]*gearRatio[1]);
      commandBuffer[head].moveAxis[1] = 1;
    } else if (strcmp(cmd,"M2A") == 0) {
      //absolute move of J2
      commandBuffer[head].targetPulses[2] = round(fValue*pulsesPerDeg[2]*gearRatio[2]);
      commandBuffer[head].moveAxis[2] = 1;
    } else if (strcmp(cmd,"M3A") == 0) {
      //absolute move of J3
      commandBuffer[head].targetPulses[3] = round(fValue*pulsesPerDeg[3]*gearRatio[3]);
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
      steppers[i].targetPulses = round(360*pulsesPerDeg[i]*gearRatio[i]);
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
  masterStepBit = 1<<STEP_BIT_0;
  slave1StepBit = 1<<STEP_BIT_1;
  slave2StepBit = 1<<STEP_BIT_2;
  slave3StepBit = 1<<STEP_BIT_3;
  if(iMax == 1) {
    master = 1;
    slave1 = 0;
    masterStepBit = 1<<STEP_BIT_1;
    slave1StepBit = 1<<STEP_BIT_0;
  } else if (iMax == 2) {
    master = 2;
    slave2 = 0;
    masterStepBit = 1<<STEP_BIT_2;
    slave2StepBit = 1<<STEP_BIT_0;
  } else {
    master = 3;
    slave3 = 0;
    masterStepBit = 1<<STEP_BIT_3;
    slave3StepBit = 1<<STEP_BIT_0;
  }
  
  //enable interrupt to start the move
  //set interrupt time based on speedMin
  speedNow = speedMin;
  iTemp = 1000000/(speedNow*stepsPerDeg[0]); //speedMin (deg/s), stepsPerDeg, 1000000 us/s
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
    if (axisToHome == i && !digitalRead(pinHome[i])) {
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
    if (stepsToTarget[i] < 0) {
      dirBits |= 1<<DIR_BIT_0;
    } else {
      dirBits &= ~(1<<DIR_BIT_0);
    }
  }
  //set direction 1
  if (stepsToTarget[1] < 0) {
    dirBits |= 1<<DIR_BIT_1;
  } else {
    dirBits &= ~(1<<DIR_BIT_1);
  }
  //set direction 2
  if (stepsToTarget[2] < 0) {
    dirBits |= 1<<DIR_BIT_2;
  } else {
    dirBits &= ~(1<<DIR_BIT_2);
  }
  //set direction 3
  if (steppers[3]stepsToTarget >= 0) {
    dirBits |= 1<<DIR_BIT_3;
  } else {
    dirBits &= ~(1<<DIR_BIT_3);
  }
  //home 1 in negative direction
  if (axisToHome == 0) {
    dirBits |= 1<<DIR_BIT_0;
  } else if (axisToHome == 1) {
    dirBits |= 1<<DIR_BIT_1;
  } else if (axisToHome == 2) {
    dirBits |= 1<<DIR_BIT_2;
  }
  //write direction port
  DIR_PORT = (DIR_PORT & ~DIR_MASK) | (dirBits & DIR_MASK);

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
  STEP_PORT = (STEP_PORT & ~STEP_MASK)|(stepBits & STEP_MASK);

  //enable the pulse reset timer
  OCR3A = 100;
  TIMSK3 |= 1<<OCIE3A;
  busy = 0;
}

ISR(TIMER3_COMPA_vect) {
  //reset step pins
  STEP_PORT = (STEP_PORT & ~STEP_MASK);
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
