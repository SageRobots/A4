#include stepper.h

//declare variables for pins used
const int pinEnc0A = 21;
const int pinEnc1A = 20;
const int pinEnc2A = 19;
const int pinEnc3A = 18;

const int pinEnc0B = 13;
const int pinEnc1B = 17;
const int pinEnc2B = A14;
const int pinEnc3B = 25;

const int pinEnable0 = A8;
const int pinEnable1 = A1;
const int pinEnable2 = A10;
const int pinEnable3 = 49;

const int pinDir0 = 53;
const int pinDir1 = 52;
const int pinDir2 = 51;
const int pinDir3 = 50;

const int pinHome0 = A0;
const int pinHome1 = 4;
const int pinHome2 = 24;
const int pinHome3 = 29;

const int pinStep0 = 37;
const int pinStep1 = 36;
const int pinStep2 = 35;
const int pinStep3 = 34;

const int pinGripDir = 10;
const int pinGripPow = 11;

unsigned long timeNow, timeGrip, timeWait;
unsigned long timePrint, timePrevSpeed;
volatile uint32_t tooFast = 0;
bool timeWaitSet;

volatile int32_t currentPulses[4] = {0,0,0,0};
const float gearRatio[4] = {26+103.0/121, 26+103.0/121, 26+103.0/121, 26+103.0/121};
const float pulsesPerDeg[4] = {1000.0/360,1000.0/360,1000.0/360,300.0/360}; //gear box input degrees
volatile int32_t targetPulses[4] = {0,0,0,0};
volatile bool inPosition[4] = {false, false, false, false};
volatile uint8_t busy = 0;
volatile int32_t stepsToTarget[4], stepsNeeded[4];
volatile float fractionRemaining[4];
const float stepsPerPulse[4] = {1/1.8*360/1000, 1/1.8*360/1000, 1/1.8*360/1000, 1/1.8*360/300};
const float stepsPerDeg[4] = {1/1.8*gearRatio[0]*2, 1/1.8*gearRatio[1]*2, 1/1.8*gearRatio[2]*2, 1/1.8*gearRatio[3]*2}; //~30
volatile uint8_t master, slave, dirBits, stepBits;
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
  int32_t axisToHome = 9;
  int32_t gripperState;
  int32_t gripperPow;
  bool set = 0;
  bool complete = 0;
  int32_t wait = 0;
  bool waitComplete = 0;
};
command commandBuffer[10];
uint8_t head = 1, tail = 0, nextHead = 2, nextTail = 1;

//CPU map
#define DIR_PORT PORTB
#define DIR_BIT_0 0
#define DIR_BIT_1 1
#define DIR_BIT_2 2
#define DIR_BIT_3 3
#define DIR_MASK ((1<<DIR_BIT_0)|(1<<DIR_BIT_1)|(1<<DIR_BIT_2)|(1<<DIR_BIT_3))

#define STEP_PORT PORTC
#define STEP_BIT_0 0
#define STEP_BIT_1 1
#define STEP_BIT_2 2
#define STEP_BIT_3 3
#define STEP_MASK ((1<<STEP_BIT_0)|(1<<STEP_BIT_1)|(1<<STEP_BIT_2)|(1<<STEP_BIT_3))

void setup() {
  //set first tail complete to indicate it is ready to be overwritten
  commandBuffer[tail].complete = 1;
  commandBuffer[tail].waitComplete = 1;

  //stepper setup
  //configure timer 1 as stepper driver input, CTC, no prescaler
  TCCR1A = 0;
  TCCR1B |= (1<<WGM12)|(1<<CS10);
  TIMSK1 = 0;
  //configure timer 3 as stepper port reset
  TCCR3A = 0;
  TCCR3B |= (1<<WGM32)|(1<<CS30);
  TIMSK3 = 0;

  stepperSetup(pinEnable0, pinStep0, pinDir0, pinEnc0A, pinEnc0B, pinHome0);
  stepperSetup(pinEnable1, pinStep1, pinDir1, pinEnc1A, pinEnc1B, pinHome1);
  stepperSetup(pinEnable2, pinStep2, pinDir2, pinEnc2A, pinEnc2B, pinHome2);
  stepperSetup(pinEnable3, pinStep3, pinDir3, pinEnc3A, pinEnc3B, pinHome3);
  
  //attach interrupts to encoder A channels
  attachInterrupt(digitalPinToInterrupt(pinEnc0A), encoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEnc1A), encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEnc2A), encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEnc3A), encoder3, RISING);

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
        accelSteps = abs(stepsToTarget[master] - stepsNeeded[master]);
        motionType = holdMove;
      }
      if (fractionRemaining[master] <= 0.5) {
        accelSteps = abs(stepsToTarget[master] - stepsNeeded[master]);
        motionType = holdMove;
      }
    } else if (motionType == holdMove) {
      if (abs(stepsToTarget[master]) <= accelSteps) motionType = decelMove;
    } else if (motionType == decelMove) {
        speedNow -= (timeNow-timePrevSpeed)/1000.0*accel;
        if (speedNow < speedMin) speedNow = speedMin;
    }
    iTemp = 1000000/(speedNow*stepsPerDeg[0]);
    timePrevSpeed = timeNow;
    //Serial.print("OCR1A: ");
    //Serial.println(iTemp);
  }
  
  //periodically print the position
  if (timeNow - timePrint >= 2000) {
    if(head != tail) {
      Serial.println("Ready");
    }
    Serial.println(digitalRead(pinHome0));
    Serial.print("\nPos: ");
    Serial.print(currentPulses[0]/gearRatio[0]/pulsesPerDeg[0]);
    Serial.print(", ");
    Serial.print(currentPulses[1]/gearRatio[1]/pulsesPerDeg[1]);
    Serial.print(", ");
    Serial.print(currentPulses[2]/gearRatio[2]/pulsesPerDeg[2]);
    Serial.print(", ");
    Serial.println(currentPulses[3]/gearRatio[3]/pulsesPerDeg[3]);
    Serial.print("Target: ");
    Serial.print(targetPulses[0]/gearRatio[0]/pulsesPerDeg[0]);
    Serial.print(", ");
    Serial.print(targetPulses[1]/gearRatio[1]/pulsesPerDeg[1]);
    Serial.print(", ");
    Serial.print(targetPulses[2]/gearRatio[2]/pulsesPerDeg[2]);
    Serial.print(", ");
    Serial.println(targetPulses[3]/gearRatio[3]/pulsesPerDeg[3]);
    if (tooFast > 0) {
      Serial.print("Too fast: ");
      Serial.println(tooFast);
      tooFast = 0;
    }
//    Serial.print("gripperState: ");
//    Serial.println(gripperState);
//    Serial.print("pulses to target 3: ");
//    Serial.println(stepsToTarget[3]);
//    Serial.print("pulses to target 1: ");
//    Serial.println(stepsToTarget[1]);
//    Serial.print("difference: ");
//    Serial.println(stepsToTarget[1]-stepsToTarget[0]);
    timePrint = timeNow;
  }

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
      digitalWrite(pinEnable, round(fValue));
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
  uint8_t i;
  //copy data out of buffer
  timeWait = timeNow;
  axisToHome = commandBuffer[tail].axisToHome;
  if(commandBuffer[tail].moveAxis[0]) {
    targetPulses[0] = commandBuffer[tail].targetPulses[0];
    inPosition[0] = false;
  }
  if(commandBuffer[tail].moveAxis[1]) {
    targetPulses[1] = commandBuffer[tail].targetPulses[1];
    inPosition[1] = false;
  }
  if(commandBuffer[tail].moveAxis[2]) {
    targetPulses[2] = commandBuffer[tail].targetPulses[2];
    inPosition[2] = false;
  }
  if(commandBuffer[tail].moveAxis[3]) {
    targetPulses[3] = commandBuffer[tail].targetPulses[3];
    inPosition[3] = false;
  }
  
  gripperState = commandBuffer[tail].gripperState;
  gripperPow = commandBuffer[tail].gripperPow;

  for(i=0;i<=3;i++) {
    if (axisToHome == i) {
      currentPulses[i] = 0;
      targetPulses[i] = round(360*pulsesPerDeg[i]*gearRatio[i]);
      inPosition[i] = false;
    }
  }
  
  //axis with most steps is master
  stepsToTarget[0] = (targetPulses[0] - currentPulses[0])*stepsPerPulse[0];
  stepsToTarget[1] = (targetPulses[1] - currentPulses[1])*stepsPerPulse[1];
  stepsToTarget[2] = (targetPulses[2] - currentPulses[2])*stepsPerPulse[2];
  stepsToTarget[3] = (targetPulses[3] - currentPulses[3])*stepsPerPulse[3];
  stepsNeeded[0] = stepsToTarget[0];
  stepsNeeded[1] = stepsToTarget[1];
  stepsNeeded[2] = stepsToTarget[2];
  stepsNeeded[3] = stepsToTarget[3];
  master = 0; //maxIndex(stepsToTarget);
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

void encoder0() {
  //if pin B is the same, the motion is ccw positive
  if(digitalRead(pinEnc0B)) {
    currentPulses[0]++;
  }
  else {
    currentPulses[0]--;
  }
}

void encoder1() {
  //if pin B is the same, the motion is ccw positive
  if(digitalRead(pinEnc1B)) {
    currentPulses[1]++;
  }
  else {
    currentPulses[1]--;
  }
}

void encoder2() {
  //if pin B is the same, the motion is ccw positive
  if(digitalRead(pinEnc2B)) {
    currentPulses[2]++;
  }
  else {
    currentPulses[2]--;
  }
}

void encoder3() {
  //if pin B is the same, the motion is ccw positive
  if(digitalRead(pinEnc3B)) {
    currentPulses[3]++;
  }
  else {
    currentPulses[3]--;
  }
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
  uint8_t j;
  //compute steps to target from current position
  for(j=0; j<=3; j++) {
    stepsToTarget[j] = (targetPulses[j] - currentPulses[j])*stepsPerPulse[j];
    if (stepsToTarget[j] == 0) {
      inPosition[j] = true;
    }
  }
  if (inPosition[0]) { // && inPosition[1] && inPosition[2] && inPosition[3]) {
    busy = 0;
    TIMSK1 = 0;
    motionType = 0;
    commandBuffer[tail].complete = 1;
    return;
  }
  if (axisToHome == 0 && !digitalRead(pinHome0)) {
    //read the home input
    currentPulses[0] = 0;
    targetPulses[0] = 0;
    busy = 0;
    TIMSK1 = 0;
    axisToHome = 9;
    commandBuffer[tail].complete = 1;
    return;
  }
  if (axisToHome == 1 && !digitalRead(pinHome1)) {
    //read the home input
    currentPulses[1] = 0;
    targetPulses[1] = 0;
    busy = 0;
    TIMSK1 = 0;
    axisToHome = 9;
    commandBuffer[tail].complete = 1;
    return;
  }
  if (axisToHome == 2 && !digitalRead(pinHome2)) {
    //read the home input
    currentPulses[2] = 0;
    targetPulses[2] = 0;
    busy = 0;
    TIMSK1 = 0;
    axisToHome = 9;
    commandBuffer[tail].complete = 1;
    return;
  }
  if (axisToHome == 3 && !digitalRead(pinHome3)) {
    currentPulses[3] = 0;
    targetPulses[3] = 0;
    TIMSK1 = 0;
    axisToHome = 9;
    commandBuffer[tail].complete = 1;
    busy = 0;
    return;
  }
  //set direction 0
  if (stepsToTarget[0] < 0) {
    dirBits |= 1<<DIR_BIT_0;
  } else {
    dirBits &= ~(1<<DIR_BIT_0);
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
  if (stepsToTarget[3] >= 0) {
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

  fractionRemaining[0] = abs((float) stepsToTarget[0]/stepsNeeded[0]);
  fractionRemaining[1] = abs((float) stepsToTarget[1]/stepsNeeded[1]);
  fractionRemaining[2] = abs((float) stepsToTarget[2]/stepsNeeded[2]);
  fractionRemaining[3] = abs((float) stepsToTarget[3]/stepsNeeded[3]);
  //set the step bits based on pulses
  stepBits = 0;
  if (master == 0) {
    if (stepsToTarget[0] != 0) {
      stepBits |= 1<<STEP_BIT_0;
    }
    //slave axes follow completion percentage
    if (fractionRemaining[1] >= fractionRemaining[0]) {
      stepBits |= 1<<STEP_BIT_1;
    }
    if (fractionRemaining[2] >= fractionRemaining[0]) {
      stepBits |= 1<<STEP_BIT_2;
    }
    if (fractionRemaining[3] >= fractionRemaining[0]) {
      stepBits |= 1<<STEP_BIT_3;
    }
  } else if (master == 1) {
    if (stepsToTarget[1] != 0) {
      stepBits |= 1<<STEP_BIT_1;
    }
    //slave axes follow completion percentage
    if (fractionRemaining[0] >= fractionRemaining[1]) {
      stepBits |= 1<<STEP_BIT_0;
    }
    if (fractionRemaining[2] >= fractionRemaining[1]) {
      stepBits |= 1<<STEP_BIT_2;
    }
    if (fractionRemaining[3] >= fractionRemaining[1]) {
      stepBits |= 1<<STEP_BIT_3;
    }
  } else if(master == 2) {
    if (stepsToTarget[2] != 0) {
      stepBits |= 1<<STEP_BIT_2;
    }
    //slave axes follow completion percentage
    if (fractionRemaining[0] >= fractionRemaining[2]) {
      stepBits |= 1<<STEP_BIT_0;
    }
    if (fractionRemaining[1] >= fractionRemaining[2]) {
      stepBits |= 1<<STEP_BIT_1;
    }
    if (fractionRemaining[3] >= fractionRemaining[2]) {
      stepBits |= 1<<STEP_BIT_3;
    }
  } else {
    if (stepsToTarget[3] != 0) {
      stepBits |= 1<<STEP_BIT_3;
    }
    //slave axes follow completion percentage
    if (fractionRemaining[0] >= fractionRemaining[3]) {
      stepBits |= 1<<STEP_BIT_0;
    }
    if (fractionRemaining[1] >= fractionRemaining[3]) {
      stepBits |= 1<<STEP_BIT_1;
    }
    if (fractionRemaining[2] >= fractionRemaining[3]) {
      stepBits |= 1<<STEP_BIT_2;
    }
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

int maxIndex(int32_t values[4]) {
  int i;
  int iMax = 0;
  for(i = 0; i<=3; i++) {
    if(abs(values[i]) > abs(values[iMax])) {
      iMax = i;
    }
  }
  return iMax;
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
