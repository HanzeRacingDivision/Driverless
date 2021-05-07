//#define useBluetooth
//#define usePWM

#define noInstructStop //stop driving if no instruction has been sent for a while

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

//#define expKpSteer          //depricated(?)
//#define expKpSteerZeroing   //removed in favor of polynomial

//super simple control system for now, PID to come at some point
const uint32_t motorControlInterval = 100; //time between control system updates in millis. The shorter this is, the more aggresive motorKpAdj will be used
uint32_t motorControlTimer;
// output = (targetSpeed * motorKpBase) + ((targetSpeed - mesSpeed) * motorKpAdj) + some other things
const float motorKpBase = 19.0; //motorKpBase will be based on rudimentary calibration (hopefully linear)
const float motorKpAdj = 30.0; //motorKpAdj should be small to reduce overshoot, but big enough to prevent underperforming steady-states (at very low/high speeds)
const uint8_t motorKpAdjAvgDepth = 4; //number of points to consider when calculating average speed (MUST BE <= speedLogFIFOsize)

const float motorKpSteerBase = 45.0; //the effects of steering (resistance) are unknown, but can be calibrated by changing this number
const uint8_t motorKpSteerAvgDepth = 4; //number of points to consider when calculating average steering angle (MUST BE <= servoLogFIFOsize)
//#ifdef expKpSteer //the steering resistance curve can be approximated with an exponential curve (at the cost of ~20/25 microseconds per calculation)
//  //on the ESP32, exp() takes around 10 microseconds (according to my own quick tests)
//  const float motorKpSteerMult[2] = {0.017974*motorKpSteerBase, 0.063663*motorKpSteerBase}; //left and right turns behanve differently (dont ask me why)
//  const float motorKpSteerExp[2] = {-0.115, 0.0677};
//  #ifdef expKpSteerZeroing //a (linear) transition between no compensation and max compensation
//    const float motorKpSteerMax[2] = {-25, 25}; //degrees
//    const float motorKpSteerMultTwo[2] = {motorKpSteerMult[0]/motorKpSteerMax[0], motorKpSteerMult[1]/motorKpSteerMax[1]};
//    //output += motorKpSteerMult*exp(motorKpSteerExp*steerAngle) - ((motorKpSteerMax-steerAngle)*motorKpSteerMultTwo)
//    //without this, output=motorKpSteerMult if steerAngle=0
//    //this just adds a linear function to stretch the output from (motorKpSteerMult, maxKp) to (0, maxKp)
//  #else
//    //note: this will have a hard transition between no compensation and some compensation, which may induce speed oscillation
//    const float motorKpSteerMin[2] = {-5, 5}; //degrees (exponential output is not 0 at 0 steerAngle (input), so any steerAngle less than this does not affect motor
//    //output += motorKpSteerMult*exp(motorKpSteerExp*steerAngle)   IF steerAngle >/< min
//  #endif
//#else
//  const float motorKpSteerMult[2] = {-0.0216*motorKpSteerBase, 0.0177*motorKpSteerBase};//output += (steerAngle-motorKpSteerOffset)*motorKpSteerMult
//  const float motorKpSteerOffset[2] = {(0.232332*motorKpSteerBase)/motorKpSteerMult[0], (0.097332*motorKpSteerBase)/motorKpSteerMult[1]};
//#endif
const float motorKpSteerPolynomial[3] = {0.2877, 0.0387, 0.0156}; //format: {c, b, a}, asin ax^2 + bx + c

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
const float encoRevPerCount = encoHoles * encoGearRatio; //counts * this = wheel revolutions
//the previous values are absolute and precise, the wheel circumference is measured/calibrated and can vary a bit
const float encoWheelCirc = PI * 62.4; //2*pi*r = pi*d
const float encoMMperCount = encoRevPerCount * encoWheelCirc; //counts * this = mm traveled (does not consider direction, assumes always forward)
const float encoMperCount = encoMMperCount / 1000.0; //(mostly for debugging purpouses)

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



#define encoPin 21

#define servoPin 26  //servo PWM pin
#define servoPotPin 33  //servo potentiometer pin
#define servoMidOffset 80 //if the servo isnt correctly centered, change this number

#define motorPin 27  //motor PWM pin

#define servoMin 500  //servo minimum pulse length (PWM)
#define servoMax 2400
#define servoPWMChan 0
const int16_t servoMid = (servoMax - servoMin)/2 + servoMin + (servoMidOffset); //(output) center value (straight steering)
const int16_t servoPotMid = 2025; //(input) center value (straight steering) for ADC data from servo potentiometer

//const float servoMicrosPerDegree[2] = {28.864, 21.862}; //(output) microseconds per degree
//const float servoMicrosPerDegreeOffset[2] = {0.085, -0.275}; //servoPWM = servoMid + ((degrees - offset) * microsPerDegree))
//const float servoADCvalPerDegree[2] = {50.000, 37.634}; //(input) ADCval per degree
//const int16_t servoADCvalPerDegreeOffset[2] = {servoPotMid-(0), servoPotMid-(-6)}; //degrees = (ADCval - offset) / ADCvalPerDegree

const float servoDegreesToMicrosPolynomial[3] = {-11.794,  25.597, -0.1344}; //format: {c, b, a}, asin ax^2 + bx + c
const float servoADCvalToDegreesPolynomial[3] = {0.3905, 0.0232, 0.000003}; //format: {c, b, a}, asin ax^2 + bx + c

//const float servoADCvalToDegreesBigPolynomial[4] = {0.2577, 0.0221, 0.002, 0.001259921}; //format: {d, c, B, A}, asin (Ax)^3 + (Bx)^2 + cx + d
////IMPORTANT: due to floating point number limitations, the x^n parameters (3rd & 4th entries) are stored as their Nth root (so "a(x^3)" -> "(Ax)^3" where 'a' is the normal polynomial parameter and 'A' is the Nth root, so A=a^(1/3) )

const int16_t motorForwStart = 1469; //forward range (calibrated)
const int16_t motorBackwStart = 1550; //backwards range (to be calibrated)
const int16_t motorMid = 1510; //neutral(?)
const int16_t motorForwRange = motorForwStart-servoMin; //forward range is from 1460 to 500
const int16_t motorBackwRange = servoMax-motorBackwStart; //backwards range is from 1550 to 2400
#define motorPWMChan 1


//int16_t degreesToServoPWM(float degreeInput) { //convert from degrees to servo PWM pulselength
//  //degreeInput = constrain(degreeInput, serialInputAngleMin, serialInputAngleMax) //already done before function is called
//  if(degreeInput < servoMicrosPerDegreeOffset[0]) {
//    return(servoMid + ((int16_t)((degreeInput-servoMicrosPerDegreeOffset[0]) * servoMicrosPerDegree[0])));
//  } else if(degreeInput > servoMicrosPerDegreeOffset[1]) {
//    return(servoMid + ((int16_t)((degreeInput-servoMicrosPerDegreeOffset[1]) * servoMicrosPerDegree[1])));
//  } else {
//    return(servoMid);
//  }
//}
//
//float ADCvalToDegrees(int16_t ADCvalInput) { //convert from (servo pot) ADCval to degrees
//  if(ADCvalInput < servoADCvalPerDegreeOffset[0]) {
//    return(((float)(ADCvalInput - servoADCvalPerDegreeOffset[0]))/servoADCvalPerDegree[0]);
//  } else if(ADCvalInput > servoADCvalPerDegreeOffset[1]) {
//    return(((float)(ADCvalInput - servoADCvalPerDegreeOffset[1]))/servoADCvalPerDegree[1]);
//  } else {
//    return(0.0);
//  }
//}
int16_t degreesToServoPWM(float degreeInput) { //convert from degrees to servo PWM pulselength
  //return(servoMid + polynomial(degreeInput, servoDegreesToMicrosPolynomial, 2));
  return(servoMid + (degreeInput*degreeInput)*servoDegreesToMicrosPolynomial[2] + degreeInput*servoDegreesToMicrosPolynomial[1] + servoDegreesToMicrosPolynomial[0]);
}
float ADCvalToDegrees(int16_t ADCvalInput) { //convert from (servo pot) ADCval to degrees
  //return(polynomial((float)ADCvalInput-servoPotMid, servoADCvalToDegreesPolynomial, 2));
  
  ADCvalInput -= servoPotMid;
  return(((int32_t)ADCvalInput*ADCvalInput)*servoADCvalToDegreesPolynomial[2] + ADCvalInput*servoADCvalToDegreesPolynomial[1] + servoADCvalToDegreesPolynomial[0]);
  
  //return(bigPolynomial((float)ADCvalInput-servoPotMid, servoADCvalToDegreesBigPolynomial, 3));
}
//float polynomial(float input, const float polynomialParameters[], const uint8_t polynomialOrder) {
//  float returnVal = polynomialParameters[0] + input*polynomialParameters[1];
//  for(uint8_t i=2; i<=polynomialOrder; i++) { returnVal += pow(input, i) * polynomialParameters[i]; }
//  return(returnVal);
//}
//float bigPolynomial(float input, const float bigPolynomialParameters[], const uint8_t polynomialOrder) { //doing a*(x^3) may overflow a float, doing (A*x)^3, where A is a^(1/3), should not.
//  float returnVal = bigPolynomialParameters[0] + input*bigPolynomialParameters[1];
//  for(uint8_t i=2; i<=polynomialOrder; i++) { returnVal += pow(input*bigPolynomialParameters[i], i); } //<-- pivotal difference!
//  return(returnVal);
//}

float motorKpSteer(uint8_t depth = servoLogFIFOsize) {
  float steerVal = ADCvalToDegrees(avgServo(depth));
//  #ifdef expKpSteer
//    #ifdef expKpSteerZeroing
//      if(steerVal < 0) {
//        return(motorKpSteerMult[0]*exp(motorKpSteerExp[0]*steerVal) - ((motorKpSteerMax[0]-steerVal)*motorKpSteerMultTwo[0]));
//      } else if(steerVal > 0) {
//        return(motorKpSteerMult[1]*exp(motorKpSteerExp[1]*steerVal) - ((motorKpSteerMax[1]-steerVal)*motorKpSteerMultTwo[1]));
//      }
//    #else
//      if(steerVal < motorKpSteerMin) {
//        return(motorKpSteerMult[0]*exp(motorKpSteerExp[0]*steerVal));
//      } else if(steerVal > motorKpSteerMin) {
//        return(motorKpSteerMult[1]*exp(motorKpSteerExp[1]*steerVal));
//      }
//    #endif
//  #else
//    if(steerVal < motorKpSteerOffset[0]) {
//      return((steerVal-motorKpSteerOffset[0])*motorKpSteerMult[0]);
//    } else if(steerVal > motorKpSteerOffset[1]) {
//      return((steerVal-motorKpSteerOffset[1])*motorKpSteerMult[1]);
//    }
//  #endif
//  else {
//    return(0.0);
//  }
  return(motorKpSteerPolynomial[2]*(steerVal*steerVal) + motorKpSteerPolynomial[1]*steerVal + motorKpSteerPolynomial[0]);
  //return(polynomial(steerVal, motorKpSteerPolynomial, 3));
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
#ifdef noInstructStop
  uint32_t noInstructTimer;
  const uint32_t noInstructTimeout = 1000; //(millis) if no insctruction has been received for this long, stop moving
#endif
//the MCU must send sensor feedback to main PC in the following format:
//encoCount steeringAngleInt milllis
//as uint32, int16 and uint32 respectively
//there are also start & end sync bytes around every message
//the encoCount is converted to meters on the receiving end,
//the angle is intified by multiplying with angleIntifyMult, and then taking the integer part of it
uint32_t serialFeedbackTimer;
const uint32_t serialFeedbackInterval = 25; //time between serial feedback messages (in millis)
const uint8_t serialFeedbackAvgSteerDepth = 2; //number of points to consider when calculating average speed (MUST BE <= servoLogFIFOsize)
const uint8_t serialFeedbackStartSyncBytes[2] = {255,255};
const uint8_t serialFeedbackEndSyncBytes[2] = {'\r','\n'}; // these can be any bytes, but these seem extra appropriate
const float angleIntifyMult = 500.0;

void sendSerialFeedback(uint32_t encoDif, int16_t servoAngle, uint32_t timestamp) {
  theSerial.write(serialFeedbackStartSyncBytes, 2);
  /*
//  theSerial.write(encoDif); theSerial.write(encoDif>>8); theSerial.write(encoDif>>16); theSerial.write(encoDif>>24);
//  theSerial.write(servoAngle); theSerial.write(servoAngle>>8);
//  theSerial.write(timestamp); theSerial.write(timestamp>>8); theSerial.write(timestamp>>16); theSerial.write(timestamp>>24);
  // forloop approach (slower(?))
  for(uint8_t i=0; i<sizeof(encoDif); i++,j++) { theSerial.write(encoDif>>(i*8)); }
  for(uint8_t i=0; i<sizeof(servoAngle); i++,j++) { theSerial.write(servoAngle>>(i*8)); }
  for(uint8_t i=0; i<sizeof(timestamp); i++,j++) { theSerial.write(timestamp>>(i*8)); }
  */
  
  //i think serial writing has some overhead, which may be lessened by writing from a buffer, instead of 1-byte-at-a-time
  uint8_t writeBuf[sizeof(encoDif)+sizeof(servoAngle)+sizeof(timestamp)]; // math should be precompiled
  writeBuf[0] = encoDif;     writeBuf[1] = encoDif>>8;    writeBuf[2] = encoDif>>16;    writeBuf[3] = encoDif>>24;
  writeBuf[4] = servoAngle;  writeBuf[5] = servoAngle>>8;
  writeBuf[6] = timestamp;   writeBuf[7] = timestamp>>8;  writeBuf[8] = timestamp>>16;  writeBuf[9] = timestamp>>24;
  /* // forloop approach (slower(?))
  uint8_t j=0;
  for(uint8_t i=0; i<sizeof(encoDif); i++,j++) { writeBuf[j] = encoDif>>(i*8); }
  for(uint8_t i=0; i<sizeof(servoAngle); i++,j++) { writeBuf[j] = servoAngle>>(i*8); }
  for(uint8_t i=0; i<sizeof(timestamp); i++,j++) { writeBuf[j] = servoAngle>>(i*8); }
  */
  theSerial.write(writeBuf, sizeof(encoDif)+sizeof(servoAngle)+sizeof(timestamp)); // math should be precompiled
  
  theSerial.write(serialFeedbackEndSyncBytes, 2);
}


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

  //servoTarget = servoMid;
  servoTarget = degreesToServoPWM(0.0);
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
        while(((micros()-serialInMinLenStart) < serialInMinLenTimeout) && keepWaiting) { //the ESP is fast enough that this (rollover safe!) math takes basically no time
          keepWaiting = (theSerial.available() < minSerialInputLength); //if bytes do hit the minimum, it will stop the while-loop
        }
        //if((micros()-serialInMinLenStart) >= serialInMinLenTimeout) { //this method saves a few microseconds, but wrongfully discards messages where the last byte came in at the very last microsecond
        if(theSerial.available() < minSerialInputLength) { //if at this point, there still isnt a full message in the buffer, just chuck it all out
          //Serial.println("minLength timeout (flushing)");
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
        //Serial.print("read timeout:"); Serial.println(serialInWholeString);
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
        #ifdef noInstructStop
          noInstructTimer = millis() + noInstructTimeout;
        #endif
      } //else { Serial.println("bad message, couldn't parse speed"); }
      serialInputSeperatorLocationLast = serialInputSeperatorLocation;
      serialInputSeperatorLocation = serialInWholeString.indexOf(serialInputSeperator, serialInputSeperatorLocationLast+1); //get index of next seperator
      if(serialInputSeperatorLocation < 0) { serialInputSeperatorLocation = serialInWholeString.length(); } //if there are no more seperators, then read whole remainder
      String angleString = serialInWholeString.substring(serialInputSeperatorLocationLast, serialInputSeperatorLocation); //get substring up to (not including) seperator
      float serialInputAngle = angleString.toFloat();
      if((serialInputAngle > serialInputAngleMin) && (serialInputAngle < serialInputAngleMax)) {
        servoTarget = degreesToServoPWM(serialInputAngle);
        ledcWrite(servoPWMChan, servoTarget); //might as well set this right away (instead of waiting for the next motor control loop
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
  #ifdef noInstructStop
    } else if(millis()>noInstructTimer) {
      //Serial.println("no instruction received for too long, stopping...");
      noInstructTimer = 4294967295; //stop timer
      targetSpeed = 0.0;
  //    servoTarget = degreesToServoPWM(0.0);
  //    ledcWrite(servoPWMChan, servoTarget); //might as well set this right away (instead of waiting for the next motor control loop
    #endif
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
    
    float mmDif = (encoCount - encoCountLast) * encoMMperCount; //compare with data from last time and calculate millimeters travelled
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

    ledcWrite(servoPWMChan, servoTarget);
  }

  if(millis() >= servoLogTimer) {
    servoLogTimer = millis() + servoLogTimerInterval;
    logServo(servoTarget, analogRead(servoPotPin));
    //logServo(ledcRead(servoPWMChan), analogRead(servoPotPin));
  }
  
  
  //sensor feedback (over serial)
  if(millis() >= serialFeedbackTimer) {
    serialFeedbackTimer = millis() + serialFeedbackInterval;
    
    float steeringDegrees = ADCvalToDegrees(avgServo(serialFeedbackAvgSteerDepth));
    int16_t intifiedSteeringDegrees = (steeringDegrees * angleIntifyMult); //NOTE: if the value (in degrees) exceeds (-32.768, 32.767), it will rollover and come out all wrong)
//    if(((steeringDegrees < 0.0) && (intifiedSteeringDegrees > 0)) || ((steeringDegrees > 0.0) && (intifiedSteeringDegrees < 0))) {
//      // the simple scalar multiplication should not have altered the sign (+ or -) of the value, so an inverted sign is indicative of rollover
//      Serial.print("intify rollover:"); Serial.println(steeringDegrees,1);
//    }
    
    sendSerialFeedback(encoCount, intifiedSteeringDegrees, millis());
  }
}
