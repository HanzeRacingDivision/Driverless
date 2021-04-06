//#define useBluetooth
//#define usePWM

#ifdef useBluetooth
  #include "BluetoothSerial.h"
  BluetoothSerial SerialBT;
  const String bluetoothAdvertisedName = "car ESP32";
  #define theSerial SerialBT
#else
  #define theSerial Serial
  //#define theSerial Serial2
#endif

#ifdef usePWM
  #include <pwmInput.h>
  pwmInput steerPWM(5, true, false, 950, 2050, 100, 25, 1512);
  pwmInput throtPWM(2, true, false, 1050, 1950, 75, 25, 1512);

  bool serialControlOverride = false;
  uint32_t lastRisingTime = 0;
  const uint32_t PWMnoSignalTimeout = 1000000; //if there's no PWM signal for this long
#endif

#define abs(x) ((x)>0?(x):-(x)) //makes abs() work with floats

const float minTargetSpeed = 0.7; //can't go slower than this
float targetSpeed; //target speed in m/s
int16_t servoTarget; //target steering servo pulselength (initialized to servoMid at setup())

#define expKpSteer
#define expKpSteerZeroing

//super simple control system for now, PID to come at some point
const uint32_t motorControlInterval = 100; //time between control system updates in millis. The shorter this is, the more aggresive motorKpAdj will be used
uint32_t motorControlTimer;
// output = (targetSpeed * motorKpBase) + ((targetSpeed - mesSpeed) * motorKpAdj) + some other things
const float motorKpBase = 19.0; //motorKpBase will be based on rudimentary calibration (hopefully linear)
const float motorKpAdj = 30.0; //motorKpAdj should be small to reduce overshoot, but big enough to prevent underperforming steady-states (at very low/high speeds)
const uint8_t motorKpAdjAvgDepth = 4; //number of points to consider when calculating average speed (MUST BE <= speedLogFIFOsize)
const float motorKpSteerBase = 45.0; //the effects of steering (resistance) are unknown, but can be calibrated by changing this number
const uint8_t motorKpSteerAvgDepth = 4; //number of points to consider when calculating average steering angle (MUST BE <= servoLogFIFOsize)
#ifdef expKpSteer //the steering resistance curve can be approximated with an exponential curve (at the cost of ~20/25 microseconds per calculation)
  //on the ESP32, exp() takes around 10 microseconds (according to my own quick tests)
  const float motorKpSteerMult[2] = {0.017974*motorKpSteerBase, 0.063663*motorKpSteerBase}; //left and right turns behanve differently (dont ask me why)
  const float motorKpSteerExp[2] = {-0.115, 0.0677};
  #ifdef expKpSteerZeroing //a (linear) transition between no compensation and max compensation
    const float motorKpSteerMax[2] = {-25, 25}; //degrees
    const float motorKpSteerMultTwo[2] = {motorKpSteerMult[0]/motorKpSteerMax[0], motorKpSteerMult[1]/motorKpSteerMax[1]};
    //output += motorKpSteerMult*exp(motorKpSteerExp*steerAngle) - ((motorKpSteerMax-steerAngle)*motorKpSteerMultTwo)
    //without this, output=motorKpSteerMult if steerAngle=0
    //this just adds a linear function to stretch the output from (motorKpSteerMult, maxKp) to (0, maxKp)
  #else
    //note: this will have a hard transition between no compensation and some compensation, which may induce speed oscillation
    const float motorKpSteerMin[2] = {-5, 5}; //degrees (exponential output is not 0 at 0 steerAngle (input), so any steerAngle less than this does not affect motor
    //output += motorKpSteerMult*exp(motorKpSteerExp*steerAngle)   IF steerAngle >/< min
  #endif
#else
  const float motorKpSteerMult[2] = {-0.0216*motorKpSteerBase, 0.0177*motorKpSteerBase};//output += (steerAngle-motorKpSteerOffset)*motorKpSteerMult
  const float motorKpSteerOffset[2] = {(0.232332*motorKpSteerBase)/motorKpSteerMult[0], (0.097332*motorKpSteerBase)/motorKpSteerMult[1]};
#endif
//const float motorKpVolt = ; //adjustment based on battery voltage, lower voltage -> higher motor power (compensating)

//let's make a fun little First In First Out speed logger, to keep track of (short-term) speed changes (to debug control loop stability)
const uint8_t speedLogFIFOsize = 8; //size of speedLog
uint8_t speedLogItt = 0; //speedLog itterator, points to OLDEST value (which is soon to be overwritten by the newest one)
float speedLogFIFO[speedLogFIFOsize][2]; //speedLog array, holds both target and measured speed
uint32_t encoLogFIFO[speedLogFIFOsize][2]; //encoLog array, holds encoder count and timestamp

//and because the ESP32 ADC is not great, let's make another FIFO array, just for (servo pot) ADC readings
const uint8_t servoLogFIFOsize = 10; //size of servoLogFIFO
uint8_t servoLogItt = 0; //servoLog itterator, points to OLDEST value (which is soon to be overwritten by the newest one)
int16_t servoLogFIFO[servoLogFIFOsize][2]; //servoLog log, holds both servo target and measured value (target as pulselength and measurement as ADC value)
//should servoLogFIFO also hold the average at that point???
const uint32_t servoLogTimerInterval = 10; //milliseconds between servoPot measurements
uint32_t servoLogTimer;

//const float encoHoles = 1.0/6.0; //6 light-pulse-triggering holes in the encoder disc
const float encoHoles = 1.0/12.0; //12 light-pulse-triggering holes in the encoder disc
const float encoGearRatio = 20.0/37.0; //37T gear on wheel side, 22T gear on encoder disc side
const float encoCountPerRev = encoHoles * encoGearRatio; //counts * this = wheel revolutions
//the previous values are absolute and precise, the wheel circumference is measured/calibrated and can vary a bit
const float encoWheelCirc = PI * 62.4; //2*pi*r = pi*d
const float encoCountPerMM = encoCountPerRev * encoWheelCirc; //counts * this = mm traveled (does not consider direction, assumes always forward)
const float encoCountPerCM = encoCountPerMM / 10.0; //(only intended for serialFeedback, as float rounding error is not as significant there)
const float encoCountPerDM = encoCountPerMM / 100.0; //(only intended for serialFeedback, as float rounding error is not as significant there)
const float encoCountPerM = encoCountPerMM / 1000.0; //(only intended for serialFeedback, as float rounding error is not as significant there)

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


void logServo(int16_t servoTargetToLog, int16_t servoADCtoLog) {
  servoLogFIFO[servoLogItt][0] = servoTargetToLog;  servoLogFIFO[servoLogItt][1] = servoADCtoLog;
  servoLogItt++;
  if(servoLogItt >= servoLogFIFOsize) { servoLogItt = 0; } //note: '>' should never happen (can only happen through bad multicore code)
}

int16_t avgServo(uint8_t depth = servoLogFIFOsize) {
  if((depth == 0) || (depth > servoLogFIFOsize)) { //why would you do this
    //Serial.println("invalid avgServo depth");
    return(0); //this just avoids dividing by 0
  }
  uint32_t sumOfADCvals = 0;
  uint8_t depthRemaining = depth;
  for(int16_t i=((int16_t)servoLogItt)-1; (i>=0) && (depthRemaining>0); i--, depthRemaining--) {
    sumOfADCvals += servoLogFIFO[i][1];
  }
  if(depthRemaining > 0) {
    for(int16_t i=((int16_t)servoLogFIFOsize)-1; (i>=servoLogItt) && (depthRemaining>0); i--, depthRemaining--) {
      sumOfADCvals += servoLogFIFO[i][1];
    }
  }
  return((int16_t)(sumOfADCvals / depth));
}



#define encoPin 15

#define servoPin 26  //servo PWM pin
#define servoPotPin 33  //servo potentiometer pin
#define servoMidOffset 80 //if the servo isnt correctly centered, change this number

#define motorPin 27  //motor PWM pin

#define servoMin 500  //servo minimum pulse length (PWM)
#define servoMax 2400
static const int16_t servoMid = (servoMax - servoMin)/2 + servoMin + (servoMidOffset); //(output) center value (straight steering)
static const float servoMicrosPerDegree[2] = {28.948, 23.283}; //(output) microseconds per degree
static const float servoMicrosPerDegreeOffset[2] = {-0.106, 0.028}; //servoPWM = servoMid + ((degrees - offset) * microsPerDegree))
#define servoPWMChan 0
static const float servoADCvalPerDegree[2] = {49.634, 40.494}; //(input) ADCval per degree
static const int16_t servoPotMid = 2025; //(input) center value (straight steering) for ADC data from servo potentiometer
static const int16_t servoADCvalPerDegreeOffset[2] = {servoPotMid-6, servoPotMid+3}; //degrees = (ADCval - offset) / ADCvalPerDegree

static const int16_t motorForwStart = 1469; //forward range (calibrated)
static const int16_t motorBackwStart = 1550; //backwards range (to be calibrated)
static const int16_t motorMid = 1510; //neutral(?)
static const int16_t motorForwRange = motorForwStart-servoMin; //forward range is from 1460 to 500
static const int16_t motorBackwRange = servoMax-motorBackwStart; //backwards range is from 1550 to 2400
#define motorPWMChan 1


int16_t degreesToServoPWM(float degreeInput) { //convert from degrees to servo PWM pulselength
  //degreeInput = constrain(degreeInput, serialInputAngleMin, serialInputAngleMax) //already done before function is called
  if(degreeInput < servoMicrosPerDegreeOffset[0]) {
    return(servoMid + ((int16_t)((degreeInput-servoMicrosPerDegreeOffset[0]) * servoMicrosPerDegree[0])));
  } else if(degreeInput > servoMicrosPerDegreeOffset[1]) {
    return(servoMid + ((int16_t)((degreeInput-servoMicrosPerDegreeOffset[1]) * servoMicrosPerDegree[1])));
  } else {
    return(servoMid);
  }
}

float ADCvalToDegrees(int16_t ADCvalInput) { //convert from (servo pot) ADCval to degrees
  if(ADCvalInput < servoADCvalPerDegreeOffset[0]) {
    return(((float)(ADCvalInput - servoADCvalPerDegreeOffset[0]))/servoADCvalPerDegree[0]);
  } else if(ADCvalInput > servoADCvalPerDegreeOffset[1]) {
    return(((float)(ADCvalInput - servoADCvalPerDegreeOffset[1]))/servoADCvalPerDegree[1]);
  } else {
    return(0.0);
  }
}

float motorKpSteer(uint8_t depth = servoLogFIFOsize) {
  float steerVal = ADCvalToDegrees(avgServo(depth));
  #ifdef expKpSteer
    #ifdef expKpSteerZeroing
      if(steerVal < 0) {
        return(motorKpSteerMult[0]*exp(motorKpSteerExp[0]*steerVal) - ((motorKpSteerMax[0]-steerVal)*motorKpSteerMultTwo[0]));
      } else if(steerVal > 0) {
        return(motorKpSteerMult[1]*exp(motorKpSteerExp[1]*steerVal) - ((motorKpSteerMax[1]-steerVal)*motorKpSteerMultTwo[1]));
      }
    #else
      if(steerVal < motorKpSteerMin) {
        return(motorKpSteerMult[0]*exp(motorKpSteerExp[0]*steerVal));
      } else if(steerVal > motorKpSteerMin) {
        return(motorKpSteerMult[1]*exp(motorKpSteerExp[1]*steerVal));
      }
    #endif
  #else
    if(steerVal < motorKpSteerOffset[0]) {
      return((steerVal-motorKpSteerOffset[0])*motorKpSteerMult[0]);
    } else if(steerVal > motorKpSteerOffset[1]) {
      return((steerVal-motorKpSteerOffset[1])*motorKpSteerMult[1]);
    }
  #endif
  else {
    return(0.0);
  }
}

//float motorKpVolt() {
//  TBD
//}

//the main PC will send desired speed + steering angle in the following format:
//speed steeringAngle\n
//example:
//1.50 22.5
//units: speed in m/s, 2 decimals  ;  angle in degrees, 1 decimal
const uint16_t minSerialInputLength = 9; //"x.xx x.x\n" is the shortest possible message
const uint32_t serialInMinLenTimeout = 5000; //(micros) if received input is less than minSerialInputLength bytes long, wait for up to serialInMinLenTimeout micros for the rest
const uint32_t serialInReadTimeout = 5000; //(micros) if reading all the bytes in the serial buffer (while-loop) takes longer than this: that's too many bytes, something is wrong
const char serialInputSeperator = ' '; //seperator between data
const float serialInputSpeedMin = -3.1; //minimum allowed speed input
const float serialInputSpeedMax = 5.1; //maximum allowed speed input
const float serialInputAngleMin = -25.1; //minimum allowed angle input
const float serialInputAngleMax = 25.1; //maximum allowed angle input

//the MCU must send sensor feedback to main PC in the following format:
//speed distance_driven_since_last_time steeringAngle\n
//example:
//1.23 0.89 15.0
//units: speed in m/s, 2 decimals  ;  distance in meters, 2 decimals  ;  angle in degrees, 1 decimal
uint32_t serialFeedbackTimer;
const uint32_t serialFeedbackInterval = 100; //time between serial feedback messages (in millis)
const uint8_t serialFeedbackAvgSpeedDepth = 2; //number of points to consider when calculating average speed (MUST BE <= speedLogFIFOsize)
const uint8_t serialFeedbackAvgSteerDepth = 2; //number of points to consider when calculating average speed (MUST BE <= servoLogFIFOsize)
const char serialFeedbackSeperator = ' '; //seperator between data
uint32_t serialFeedbackEncoCountLast; //holds encoder count from last time timer was triggered


void setup() {
  Serial.begin(115200); //always start regular serial, makes debugging easier
  #ifdef useBluetooth
    SerialBT.begin(bluetoothAdvertisedName);
  #endif
  theSerial.setTimeout(2); //leftover from an older version, but probably still worth having

  #ifdef usePWM
    steerPWM.init(INPUT_PULLDOWN);
    throtPWM.init(INPUT_PULLDOWN);
  #endif
  
  pinMode(encoPin, INPUT_PULLUP);
  attachInterrupt(encoPin, encoISR, FALLING);

  pinMode(servoPin, OUTPUT); //probably also part of ledcAttachPin() but just for safety
  pinMode(servoPotPin, INPUT); //basically all pins are input by default, but just for safety
  pinMode(motorPin, OUTPUT); //probably also part of ledcAttachPin() but just for safety
  
  ledcSetup(servoPWMChan, 244, 12); // at ~244Hz and 12bit res, the duty cycle value = microseconds
  ledcAttachPin(servoPin, servoPWMChan);
  ledcSetup(motorPWMChan, 61, 14); // at ~61Hz and 14bit res, the duty cycle value = microseconds
  ledcAttachPin(motorPin, motorPWMChan);

  servoTarget = servoMid;
  ledcWrite(servoPWMChan, servoTarget);
  ledcWrite(motorPWMChan, motorMid);
}

void loop() {
  //serial input
  if(theSerial.available()) {
    bool longEnoughInput = true;
    #ifndef useBluetooth  //bluetooth serial has some issues with this
      if(theSerial.available() < minSerialInputLength) { //either not all the bytes have made it into the serial buffer yet, or something went wrong
        //you could skip this one loop(), and try again the next time, but the current code is not in that much of a hurry, so just sit and wait (doing nothing)
        uint32_t serialInMinLenStart = micros(); //normally i'd add timeout value to this value, and the use "while(micros() < this)", but that's not rollover compatible
        bool keepWaiting = true;
        while(((micros()-serialInMinLenTimeout) < serialInMinLenTimeout) && keepWaiting) { //the ESP is fast enough that this (rollover safe!) math takes basically no time
          keepWaiting = (theSerial.available() < minSerialInputLength); //if bytes do hit the minimum, it will stop the while-loop
        }
        //if((micros()-serialInMinLenTimeout) >= serialInMinLenTimeout) { //this method saves a few microseconds, but wrongfully discards messages where the last byte came in at the very last microsecond
        if(theSerial.available() < minSerialInputLength) { //if at this point, there still isnt a full message in the buffer, just chuck it all out
          //theSerial.println("minLength timeout (flushing)");
          while(theSerial.available()) { //everything in buffer
            theSerial.read(); //just read out, dont store it anywhere
          } //a flush command would be faster, but arduino serial flush functions were spotty in the (distant?) past, this is safer
          //return;  //return the whole loop()
          longEnoughInput = false; //using return(); could delay the motor control loop in a super worst-case-scenario (floating-pin serial input noise)
        }
      }
    #endif
    if(longEnoughInput) { //if the message length is >= minSerialInputLength
      //read message
      String serialInWholeString = "";
      uint32_t serialInReadStart = micros(); //normally i'd add timeout value to this value, and the use "while(micros() < this)", but that's not rollover compatible
      while((theSerial.available()) && ((micros()-serialInReadStart) < serialInReadTimeout)) {
        serialInWholeString += (char)theSerial.read();
      }
      if((micros()-serialInReadStart) >= serialInReadTimeout) {
        //theSerial.println("read timeout");
      }
      
      //parse message
      serialInWholeString.trim(); //(should be unnecessary) removes trailing and leading whitespace (' ' and '\n')
      int16_t serialInputSeperatorLocationLast = 0;
      int16_t serialInputSeperatorLocation = serialInWholeString.indexOf(serialInputSeperator, serialInputSeperatorLocationLast); //get index of first seperator
      if(serialInputSeperatorLocation < 0) { /*Serial.println("bad message, no seperators");*/ serialInputSeperatorLocation = serialInWholeString.length(); } //shouldn't happen, but if it does just attempt to parse anyway
      String speedString = serialInWholeString.substring(serialInputSeperatorLocationLast, serialInputSeperatorLocation); //get substring up to (not including) seperator
      float serialInputSpeed = speedString.toFloat();
      if((serialInputSpeed > serialInputSpeedMin) && (serialInputSpeed < serialInputSpeedMax)) {
        targetSpeed = serialInputSpeed;
      } //else { Serial.println("bad message, couldn't parse speed"); }
      serialInputSeperatorLocationLast = serialInputSeperatorLocation;
      serialInputSeperatorLocation = serialInWholeString.indexOf(serialInputSeperator, serialInputSeperatorLocationLast+1); //get index of next seperator
      if(serialInputSeperatorLocation < 0) { serialInputSeperatorLocation = serialInWholeString.length(); } //if there are no more seperators, then read whole remainder
      String angleString = serialInWholeString.substring(serialInputSeperatorLocationLast, serialInputSeperatorLocation); //get substring up to (not including) seperator
      float serialInputAngle = angleString.toFloat();
      if((serialInputAngle > serialInputAngleMin) && (serialInputAngle < serialInputAngleMax)) {
        servoTarget = degreesToServoPWM(serialInputAngle);
      } //else { Serial.println("bad message, couldn't parse angle"); }
      #ifdef usePWM
        if(!serialControlOverride) {
          //theSerial.println("switching to serial commands");
          serialControlOverride = true;
        }
      #endif
      //flush serial for ultra safety:
      while(theSerial.available()) {
        theSerial.read();
      }
    }
  }

  #ifdef usePWM
    if(!serialControlOverride) {
      uint32_t newRisingTime = throtPWM._risingTime; //when dealing with interrupts, this is a relatively safe approach
      if(newRisingTime != lastRisingTime) {
        targetSpeed = throtPWM.normalized() * 2.0;
        servoTarget = servoMid + ((int16_t)((steerPWM.normalized()*30.0) * servoMicrosPerDegree)); //convert from degrees to servo PWM pulselength
      } else {
        if((micros() - newRisingTime) > PWMnoSignalTimeout) { //if the PWM source hasn't sent anything in a while
          targetSpeed = 0; //brake
        }
      }
      lastRisingTime = newRisingTime;
    }
  #endif
  
  //(motor) control loop
  if(millis() >= motorControlTimer) {
    motorControlTimer = millis() + motorControlInterval;
    
    float mmDif = (encoCount - encoCountLast) * encoCountPerMM; //compare with data from last time and calculate millimeters travelled
    float mesSpeed = mmDif / (millis() - encoTimer); //millimeters / milliseconds = meters/second
    encoCountLast = encoCount;  encoTimer = millis(); //save data for next time (immedietly after reading it, because it is updated through interrupts)
    
    encoLogFIFO[speedLogItt][0] = encoCount;  encoLogFIFO[speedLogItt][0] = millis();
    logSpeed(targetSpeed, mesSpeed);
    //logServo(servoTarget, analogRead(servoPotPin));
    
    if(targetSpeed > minTargetSpeed) { //this check is done here (as opposed to when receiving the targetspeed input/command) because there already needs to be an if-statement anyways, motorForwStart != motorMid
      float avgMesSpeed = avgSpeed(motorKpAdjAvgDepth); //get average speed (default is full depth, otherwise: avgSpeed(depth))
      float output = (targetSpeed * motorKpBase); //base multiplier
      output += ((targetSpeed - avgMesSpeed) * motorKpAdj); //speed error multiplier
      output += motorKpSteer(motorKpSteerAvgDepth) * targetSpeed; //steering adds resistance, this counteracts that (calibration was done at 1.0m/s target speed, idk if i need to multiply by speed yet, TBD)
      //output += motorKpVolt() * targetSpeed; //lower battery voltage -> less power, this counteracts that  TBD
      ledcWrite(motorPWMChan, motorForwStart - constrain(((int16_t)output), -20, motorForwRange)); //shouldn't come out to illigal values (outside of 500-2400us)
    } else if(targetSpeed < -0.01) { // braking (doing "<0" is not quite 100% safe when using floats, but it may have been fine here)
      ledcWrite(motorPWMChan, motorMid); //temporary, braking/backwards math TBD
      //ledcWrite(motorPWMChan, motorBackwStart + some math here); //set motor to braking/backwards
    } else {
      ledcWrite(motorPWMChan, motorMid); //set motor to neutral/braking
    }

    ledcWrite(servoPWMChan, servoTarget); //TBD real actual (calibrated) servo math
  }

  if(millis() >= servoLogTimer) {
    servoLogTimer = millis() + servoLogTimerInterval;
    logServo(servoTarget, analogRead(servoPotPin));
    //logServo(ledcRead(servoPWMChan), analogRead(servoPotPin));
  }
  
  
  //sensor feedback (over serial)
  if(millis() >= serialFeedbackTimer) {
    serialFeedbackTimer = millis() + serialFeedbackInterval;
    
    float steeringAngle = ADCvalToDegrees(avgServo(serialFeedbackAvgSteerDepth));
    float serialFeedbackDrvnDist = (encoCount - serialFeedbackEncoCountLast) * encoCountPerM; //determine distance driven since last feedback
    serialFeedbackEncoCountLast = encoCount; //do this immedietly after, becuase encoCount is incremented via interrupts, so making/sending the string first could lead to skipped counts
    
    String dataString = String(avgSpeed(serialFeedbackAvgSpeedDepth), 2) + serialFeedbackSeperator + String(serialFeedbackDrvnDist, 2) + serialFeedbackSeperator + String(steeringAngle, 1);
    theSerial.println(dataString); //you could save 1 line of code by skipping the string init, but arduino doesn't want stuff in println()
  }
}
