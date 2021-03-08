#define useBluetooth

#ifdef useBluetooth
  #include "BluetoothSerial.h"
  BluetoothSerial SerialBT;
  const String bluetoothAdvertisedName = "car ESP32";
  #define theSerial SerialBT
#else
  #define theSerial Serial
#endif



float targetSpeed = 0.0; //target speed in m/s
const float minTargetSpeed = 0.9; //can't go slower than this

//super simple control system for now, PID to come at some point
// output = (targetSpeed * motorKpBase) + ((targetSpeed - mesSpeed) * motorKpAdj)
const float motorKpBase = 19.0; //motorKpBase will be based on rudimentary calibration (hopefully linear)
const float motorKpAdj = 15.0; //motorKpAdj should be small to reduce overshoot, but big enough to prevent underperforming steady-states (at very low/high speeds)
const uint32_t motorControlInterval = 100; //time between control system updates in millis. The shorter this is, the more aggresive motorKpAdj will be used
uint32_t motorControlTimer;

//let's make a fun little First In First Out speed logger, to keep track of (short-term) speed changes (to debug control loop stability)
const uint8_t speedLogFIFOsize = 8; //size of speedLog
uint8_t speedLogItt = 0; //speedLog itterator, points to OLDEST value (which is soon to be overwritten by the newest one)
float speedLogFIFO[speedLogFIFOsize][2]; //speedLog array, holds both target and measured speed
uint32_t encoLogFIFO[speedLogFIFOsize][2]; //encoLog array, holds encoder count and timestamp

#define encoPin 15

//const float encoHoles = 1.0/6.0; //6 light-pulse-triggering holes in the encoder disc
const float encoHoles = 1.0/12.0; //12 light-pulse-triggering holes in the encoder disc
const float encoGearRatio = 20.0/37.0; //37T gear on wheel side, 22T gear on encoder disc side
const float encoCountPerRev = encoHoles * encoGearRatio; //counts * this = wheel revolutions
//the previous values are absolute and precise, the wheel circumference is measured/calibrated and can vary a bit
const float encoWheelCirc = PI * 62.4; //2*pi*r = pi*d
const float encoCountPerMM = encoCountPerRev * encoWheelCirc; //counts * this = mm traveled (does not consider direction, assumes always forward)

volatile uint32_t encoCount = 0;
uint32_t encoCountLast;
uint32_t encoTimer; //stores timestamp associated with encoCountLast

IRAM_ATTR void encoISR() {
  encoCount += 1;
}

void logSpeed(float targetSpeedToLog, float mesSpeedToLog) {
  speedLogFIFO[speedLogItt][0] = targetSpeedToLog;  speedLogFIFO[speedLogItt][1] = mesSpeedToLog;
  //encoLogFIFO[speedLogItt][0] = encoCount;  encoLogFIFO[speedLogItt][0] = millis();
  speedLogItt++;
  if(speedLogItt >= speedLogFIFOsize) { speedLogItt = 0; } //note: '>' should never happen (can only happen through bad multicore code)
}

float avgSpeed(uint8_t depth = speedLogFIFOsize) {
  if((depth == 0) || (depth > speedLogFIFOsize)) { //why would you do this
    //Serial.println("invalid avgSpeed depth");
    return(0.0); //this just avoids dividing by 0
  }
  float sumOfSpeeds = 0.0;
  uint8_t depthRemaining = depth;
  for(int16_t i=((int16_t)speedLogItt)-1; (i>=0) && (depthRemaining>0); i--, depthRemaining--) {
    sumOfSpeeds += speedLogFIFO[i][1];
  }
  if(depthRemaining > 0) {
    for(int16_t i=((int16_t)speedLogFIFOsize)-1; (i>=speedLogItt) && (depthRemaining>0); i--, depthRemaining--) {
      sumOfSpeeds += speedLogFIFO[i][1];
    }
  }
  return(sumOfSpeeds / depth);
}


#define servoPin 26
#define servoMidOffset 50 //if the servo isnt correctly centered, change this number

#define motorPin 27

#define servoMin 500
#define servoMax 2400
static const int servoMid = (servoMax - servoMin)/2 + servoMin + (servoMidOffset);
#define servoPWMChan 0

static const int motorForwStart = 1469; //forward range is from 1460 to 500
static const int motorBackwStart = 1550; //backwards range is from 1550 to 2400
static const int motorMid = 1510; //neutral(?)
static const int motorForwRange = motorForwStart-servoMin; //forward range is from 1460 to 500
static const int motorBackwRange = servoMax-motorBackwStart; //backwards range is from 1550 to 2400
#define motorPWMChan 1


/*
the test will be to drive forward (straight line), in 3 phases:
accelerate, constant velocity, decelerate.
the results (difference in encoder count since last interval)
the resuls will be spit out at the end, in batches (to lighten the load on the bluetooth serial)
*/

uint32_t testStartTime; //in millis
uint8_t testPhase = 0;
//const uint32_t testTimes[3] = {2000, 
const uint32_t accelTime = 2000; //testPhase = 1
const uint32_t conVelTime = 2000; //testPhase = 2
const uint32_t decelTime = 2000; //testPhase = 3
const uint32_t ovflTime = 400;
const uint32_t resultArraySize = ((accelTime + conVelTime + decelTime + ovflTime)/motorControlInterval);
uint16_t resultArray[resultArraySize];
uint32_t resultsCaptured = 0; //reset at start of test, increment by 1 every loop (up to size of resultArray)

const uint16_t resultPrintBatchSize = 100;
const uint32_t resultPrintLineDelay = 10;
const char resultPrintCommandChar = 'p'; //input command, 'start printing' and 'print more'
const char resultPrintMoreChar = 'm'; //not command, just to indication that more data is available
const char resultPrintDoneChar = 'd'; //not command, just to indicate the end of the list
const char resultPrintResetChar = 'r';
const char testStartChar = 's'; //send 's100' to start a test with a target speed of 100cm/s.
uint32_t resultPrintCounter = 0;

const char servoAdjustCommandChar = 'a'; //same usage as testStartChar, but for servo stuff


void setup() {
  Serial.begin(115200);
  #ifdef useBluetooth
    SerialBT.begin(bluetoothAdvertisedName);
  #endif
  theSerial.setTimeout(5);
  
  pinMode(encoPin, INPUT_PULLUP);
  attachInterrupt(encoPin, encoISR, FALLING);
  
  ledcSetup(servoPWMChan, 61, 14); // at ~61Hz and 14bit res, the duty cycle value = microseconds
  ledcAttachPin(servoPin, servoPWMChan);
  ledcSetup(motorPWMChan, 61, 14); // at ~61Hz and 14bit res, the duty cycle value = microseconds
  ledcAttachPin(motorPin, motorPWMChan);
  
  ledcWrite(servoPWMChan, servoMid);
  ledcWrite(motorPWMChan, motorMid);
}

void loop() {
  if(theSerial.available()) {
    delay(1);
    char commandChar = theSerial.read();
    if(commandChar == testStartChar) {
      int serialInt = theSerial.parseInt();
      if((serialInt > 0) && (serialInt <= 1000)) {
        targetSpeed = serialInt / 100.0;
      } else {
        targetSpeed = 0.0;
      }
      resultsCaptured = 0;
      resultPrintCounter = 0;
      testStartTime = millis();
      motorControlTimer = millis(); //manually trigger loop
      theSerial.print("testing at ");
      theSerial.println(targetSpeed*100);
      testPhase = 1;
    } else if(commandChar == resultPrintCommandChar) {
      if(resultPrintCounter == 0) {
        theSerial.println(resultPrintCommandChar);
      }
      uint32_t batchStartItt = resultPrintCounter;
      uint32_t batchEndItt = min(batchStartItt + resultPrintBatchSize, resultsCaptured);
      while(resultPrintCounter < batchEndItt) {
        theSerial.println(resultArray[resultPrintCounter]);
        delay(resultPrintLineDelay);
        resultPrintCounter++;
      }
      if(resultPrintCounter < resultsCaptured) {
        theSerial.println(resultPrintMoreChar);
      } else {
        theSerial.println(resultPrintDoneChar);
        resultPrintCounter = 0;
      }
    } else if(commandChar == resultPrintResetChar) {
      theSerial.println(resultPrintResetChar);
      resultPrintCounter = 0;
    } else if(commandChar == servoAdjustCommandChar) {
      int serialInt = theSerial.parseInt();
      ledcWrite(servoPWMChan, servoMid + constrain(serialInt, -1000, 1000));
      theSerial.println(ledcRead(servoPWMChan));
    } else {
      theSerial.println("nah");
    }
    //flush serial:
    while(theSerial.available()) {
      theSerial.read();
    }
  }

  if(millis() >= motorControlTimer) {
    motorControlTimer = millis() + motorControlInterval;
    
    uint32_t countDif = encoCount - encoCountLast; //compare with data from last time
    float mmDif = countDif * encoCountPerMM;
    float mesSpeed = mmDif / (millis() - encoTimer); //millimeters / milliseconds = meters/second
    
    encoLogFIFO[speedLogItt][0] = encoCount;  encoLogFIFO[speedLogItt][0] = millis();
    logSpeed(targetSpeed, mesSpeed);

    //theSerial.print(targetSpeed);  theSerial.print('\t');
    //theSerial.print(mesSpeed);  theSerial.print('\t');

    if(targetSpeed > minTargetSpeed) {
      float avgMesSpeed = avgSpeed(4); //get average speed (default is full depth, otherwise: avgSpeed(depth))
      //theSerial.print(avgMesSpeed);  theSerial.print('\t');
      float output = (targetSpeed * motorKpBase) + ((targetSpeed - avgMesSpeed) * motorKpAdj);
      ledcWrite(motorPWMChan, motorForwStart - constrain(((int16_t)output), -20, motorForwRange)); //shouldn't come out to illigal values (outside of 500-2400us)
      //theSerial.println(output/10.0); //print a (hopefully) legible value
    } else {
      ledcWrite(motorPWMChan, motorMid); //set motor to neutral/braking
      //theSerial.println(0.0);
    }

    if((testPhase > 0) && (testPhase < 4)) {
      resultArray[resultsCaptured] = countDif;
      resultsCaptured++;
      if(resultsCaptured >= resultArraySize) { theSerial.println("result buffer overflow"); ledcWrite(motorPWMChan, motorMid); while(1){} } //abort
      if(((millis() - testStartTime) >= accelTime) && (testPhase < 2)) { //unnecessary
        testPhase = 2;
        theSerial.print("conVel phase ");
        theSerial.println(avgSpeed());
      }
      if(((millis() - testStartTime) >= (accelTime + conVelTime)) && (testPhase < 3)) {
        targetSpeed = 0;
        ledcWrite(motorPWMChan, motorMid); //set motor to neutral/braking
        testPhase = 3;
        theSerial.print("decel phase ");
        theSerial.println(avgSpeed());
      }
      if((millis() - testStartTime) >= (accelTime + conVelTime + decelTime)) {
        targetSpeed = 0; //(just to be sure)
        ledcWrite(motorPWMChan, motorMid); //set motor to neutral/braking (just to be sure)
        testPhase = 4;
        theSerial.print("test done ");
        theSerial.println(avgSpeed());
        uint32_t distanceEncoCount = 0;
        for(uint32_t i=0; i<resultsCaptured; i++) {
          distanceEncoCount += resultArray[i];
        }
        theSerial.println((float) distanceEncoCount * encoCountPerMM);
      }
    }
    
    encoCountLast = encoCount;  encoTimer = millis(); //save data for next time
  }
}
