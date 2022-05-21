#define abs(x) ((x)>0?(x):-(x)) //makes abs() work with floats


#include "thijsFIFO.h"

//#define threeWireHbridge //seems to perform worse (less power, more noise) across the board, not sure why yet.
//#define constPowerMode  //not the best idea (power hungry + runs hot for minor perf gain (maybe))             TBD!!
#define motENpin 3         //you could do a second (higher freq) PWM on this one to limit the power, but IDK if it'll work well (since threeWireHbridge sucks)
const uint8_t motApin = 5; //timer0 PWM pins have a const freq becuase timer0 is also used for millis() timekeeping
const uint8_t motBpin = 6; //you can still halve the frequency by setting PWM mode (with the TCCR0A register, page 84 of datasheet)


#define smoothTargetApproach
//#define quadraticApproach
const float maxDecel = 0.3000; //the deceleration charactaristics of the motor. Too high and you overshoot, too low and you undershoot (and vibrate at the end)
const float minDecelVal = maxDecel * 0.75; //the higher this value, the later the decel starts, so the faster you arrive (also more likely to overshoot)
const float overBrakeMult = 1.00;

const float absMaxPower = 0.90; //(range = 0.0~1.0) you can limit the motor power to prevent belt slipping (and bone breaking if you're holding the wheel)

const int32_t targetMargin = 50; //(ADCunit) minimum angle error required for intervention (otherwise it would just keep oscillating around the target area forever)
#ifdef smoothTargetApproach
  const int32_t targetSmoothMargin = 200; //(ADCunit) angle error at which to start the smoothing
  #ifdef quadraticApproach  // provides a quadratic curve near the target
    const float targetSmoothDivider = ((targetSmoothMargin*targetSmoothMargin)/targetMargin)-targetMargin;
  #else  // provides a sinusoidal curve near the target
    const float targetSmoothDivider = (targetSmoothMargin-targetMargin)/PI;
  #endif
  float smoothApproachMult(int32_t absDiff) {
      if(absDiff < targetMargin) {
        return(0.0);
      } else if(absDiff < targetSmoothMargin) {
        #ifdef quadraticApproach  // provides a quadratic curve near the target
          return((((absDiff*absDiff) / targetMargin) - targetMargin) / targetSmoothDivider);
        #else  // provides a sinusoidal curve near the target
          return((cos((absDiff-targetMargin)/targetSmoothDivider) * (-0.5)) + 0.5);
        #endif
      } else {
        return(1.0);
      }
    }
#endif

///////////////////////// variables //////////////////////////
//float desiredPos = 0.0; //the desired (instructed) position of the servo)
//float currentPos = 0.0; //the current position of the servo (may be replaced by posHistFIFO[0])
//FIFO<float> posHistFIFO(10); //a First-In-First-Out array to store position history
//float currentSpeed = 0.0; //based on a (filtered/dampened) calculation from the pos (current & hist)

int32_t desiredPos = 0; //the desired (instructed) position of the servo)
int32_t currentPos = 0; //the current position of the servo (may be replaced by posHistFIFO[0])
//FIFO<int32_t> posHistFIFO(5); //a First-In-First-Out array to store position history
FIFO<float> speedHistFIFO(4); //a First-In-First-Out array to store (unfiltered) speed history
float currentSpeed = 0.0; //based on a (filtered/dampened) calculation from the pos (current & hist)

const uint16_t maxSingleDif = 4096/2; //(nyquist?) to detect rollover in a quick'n'dirty way
int16_t lastAngle = 0; //used for position (and rollover) calculation
int32_t lastPos = 0; //used for speed calculation
uint32_t lastPosTime; //used for speed calculation


//float calcSpeed() { //note: returns in the 'unit' of ADCunit per sampling interval. To convert to SI unit, use ADC resolution and sampling interval
//  int32_t sum = 0;
//  for(uint8_t i=0; i<(posHistFIFO.size-1); i++) {
//    sum += posHistFIFO[i] - posHistFIFO[i+1];
//  }
//  return(((float)sum) / (posHistFIFO.size-1));
//}
float calcSpeed() { //note: returns in the 'unit' of ADCunit per millisecond. To convert to SI unit, use ADC resolution
  float sum = 0;
  for(uint8_t i=0; i<speedHistFIFO.size; i++) {
    sum += speedHistFIFO._data[i]; //the order does not matter, so directly interfacing with the (unordered) FIFO data may be faster
  }
  return(sum / speedHistFIFO.size);
  //note: recalculating the whole thing every time is painfully slow. You could also just remove the oldest data from the (global var) sum and add the newest, but you'd run into floating point inaccuracies
}

float posToMotPow(int32_t currentPos, int32_t desiredPos, float currentSpeed) {
  float returnVal = 0.0;
  int32_t absDiff = abs(desiredPos-currentPos);
  if(absDiff >= targetMargin) { //avoid divide by 0
    float requiredDecel = (currentSpeed*currentSpeed) / (2.0*absDiff);
    //Serial.print("decel: "); Serial.print(requiredDecel); Serial.print(" = "); Serial.print(requiredDecel/maxDecel); Serial.print(' ');
    if(abs(requiredDecel) > minDecelVal) {
      returnVal = ((currentSpeed >= 0.0) ? -1.0 : 1.0) * (requiredDecel/maxDecel); //motor power directly proportional to maxDecel
      returnVal *= overBrakeMult; //brake a little more than is needed to prevent overshoot (prefer undershoot i guess)
    } else {
      returnVal = (desiredPos > currentPos) ? 1.0 : -1.0;
    }
    returnVal = constrain(returnVal, -absMaxPower, absMaxPower);
    #ifdef smoothTargetApproach
      returnVal *= smoothApproachMult(absDiff);
    #endif
  } //else {
  return(returnVal);
}

int32_t updatePos(int32_t currentPos, int16_t newAngle) {
  currentPos += (newAngle - lastAngle);
  if(abs(newAngle-lastAngle) > maxSingleDif) {
//    if((newAngle < lastAngle) != (currentSpeed > 0)) {
//      Serial.print("rollover confusion!: "); Serial.print(lastAngle); Serial.print("->"); Serial.print(newAngle); Serial.print(' '); Serial.println(currentSpeed);
//    }
    currentPos += (newAngle < lastAngle) ? 4095 : -4095;
  }
  lastAngle = newAngle;
  return(currentPos);
}

#ifdef constPowerMode
  #ifdef threeWireHbridge
    #undef threeWireHbridge
    #warning("undefined threeWireHbridge because you can't have constPowerMode and threeWireHbridge simultaniously")
  #endif
  // note: setup the wiring such that positive power values produce CW rotations, please
  //for some (novel) holding power schemes, you could do inverted PWM on 1 pin, and have the PWM values the same for both A and B
  //this means it will fluctuate between CW and CCW at high frequency, which should (theorecially) provide full torque at the whole range (which makes the control loops easier)
  //this does absolutely waste power (and therefore generate heat), so be carefull in testing)
  //the PWM value is also shifted, because (50% duty cycle) is now idle, 0% is full CW, 100% is full CCW (or the opposite, you get what i mean though)
  #define set_inverted_PWM      TCCR0A |= 0b00010000; //COMnB0 to 1
  #define set_normal_PWM        TCCR0A &= 0b11101111; //COMnB0 to 0
#endif

#ifdef constPowerMode
  void applyMotPow(float motPow=0.0) {
    int16_t tempVal = motPow*127.5;
    tempVal += 128;
    analogWrite(motApin, constrain(tempVal, 0, 255));
    analogWrite(motBpin, constrain(tempVal, 0, 255));
    #ifdef motENpin
      digitalWrite(motENpin, HIGH);
    #endif
  }
#else
  #ifdef threeWireHbridge
    void applyMotPow(float motPow=0.0) {
      int16_t tempVal = motPow*255.0;
      digitalWrite(motApin, tempVal > 0);
      digitalWrite(motBpin, tempVal < 0);
      analogWrite(motENpin, constrain(abs(tempVal), 0, 255));
    }
  #else
    void applyMotPow(float motPow=0.0) {
      int16_t tempVal = motPow*255.0;
      analogWrite(motApin, constrain(tempVal, 0, 255));
      analogWrite(motBpin, constrain(-tempVal, 0, 255));
      #ifdef motENpin
        digitalWrite(motENpin, HIGH);
        //digitalWrite(motENpin, abs(tempVal) > 0); //this may be worse (it could allow idling instead of shorting both motor terminals to GND)
      #endif
    }
  #endif
#endif

const uint32_t calibDeltaT = 500;
const uint8_t moves = 2;
const uint16_t measurements = 720/moves;
int16_t readBuff[moves][measurements];
//uint8_t firstBuff[measurements*2];
//uint8_t secondBuff[measurements*2];
//uint8_t thirdBuff[measurements*2];
//uint8_t readBuff[3][measurements*2];}
float powerTable[moves] = {absMaxPower, -absMaxPower};
int32_t posLimits[moves] = {3000, 6000};
uint32_t startTime[moves];
uint32_t endTime[moves];




//float calcTrendline(uint8_t moveIndex=0) { //a function which could (if completed) calculate the maxDecel automatically (not used, becuase Excel was easier)
//  float t, sumX, sumY, sumXsquared, sumXY, a, b;
//  uint16_t usedMeasurements = measurements;
//  for(uint16_t i=0; i<usedMeasurements; i++) {
//    currentPos = updatePos(currentPos, readBuff[moveIndex][i]);
//    //posHistFIFO.put(currentPos);
//    float newRawSpeed = ((currentPos - lastPos)*1000.0) / 500.0; //calculate speed in ADCticks per millisecond
//    lastPos = currentPos;
//    speedHistFIFO.put(newRawSpeed);
//    currentSpeed = calcSpeed(); //slow ~100+us
//    Serial.print(currentPos); Serial.print('\t'); Serial.println(currentSpeed);
//    t += 0.5;
//    sumX += t;
//    sumY += currentSpeed;
//    sumXsquared += (t*t);
//    sumXY += (t*currentSpeed);
//    if(currentSpeed < 0) {
//      usedMeasurements = i+1;
//    }
//  }
//  Serial.print(usedMeasurements); Serial.print(" in "); Serial.print(calibEndTime-calibStartTime); Serial.print("=>"); Serial.println((calibEndTime-calibStartTime)/measurements);
//  a = ((usedMeasurements * sumXY) - (sumX - sumY)) / ((usedMeasurements * sumXsquared) - (sumX * sumX));
//  b = (sumY - (a * sumX)) / usedMeasurements;
//  Serial.print("a, b: "); Serial.print(a); Serial.print(", "); Serial.println(b);
//}

void setup() {
  Serial.begin(115200);
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);

  twoWireClockSetup(800000); 

  twoWireResetConfig();
  //twoWirePrintConfig();

  pinMode(motApin, OUTPUT);
  pinMode(motBpin, OUTPUT);
  #ifdef motENpin
    pinMode(motENpin, OUTPUT);
  #endif


//  Serial.print("divider:"); Serial.println(targetSmoothDivider);
//  for(int16_t i=500; i>0; i--) {
//    Serial.print(i); Serial.print(' '); Serial.println(smoothApproachMult(i));
//  }
//  while(1);

//  lastAngle = twoWireReadTwoBytes(0x0C);
////  applyMotPow(powerTable[0]); //make sure starting speed is reached (settled)
////  delay(1000);
//  uint16_t usedMeasurements[moves];
//  for(uint8_t j=0; j<moves; j++) {
//    applyMotPow(powerTable[j]);
//    usedMeasurements[j] = measurements;
//    startTime[j] = micros();
//    for(uint16_t i=0; i<usedMeasurements[j]; i++) {
//      uint32_t loopStart = micros();
//      readBuff[j][i] = twoWireReadTwoBytes(0x0C);
//      currentPos = updatePos(currentPos, readBuff[j][i]);
//      if(abs(currentPos) >= posLimits[j]) {
//        usedMeasurements[j] = i+1;
//      }
//      delayMicroseconds((calibDeltaT-5)-(micros()-loopStart)); //calibrated delay to make preferred sample interval
//    }
//    endTime[j] = micros();
//  }
//  applyMotPow(0.0);
//
//  currentPos = 0;
//  lastAngle = readBuff[0][0];
//  
//  for(uint8_t j=0; j<moves; j++) {
//    Serial.print("move "); Serial.print(j); Serial.print("  power: "); Serial.println(powerTable[j]);
//    for(uint16_t i=0; i<usedMeasurements[j]; i++) {
//      currentPos = updatePos(currentPos, readBuff[j][i]);
//      //posHistFIFO.put(currentPos);
//      float newRawSpeed = ((currentPos - lastPos)*1000.0) / ((float)calibDeltaT); //calculate speed in ADCticks per millisecond
//      lastPos = currentPos;
//      speedHistFIFO.put(newRawSpeed);
//      currentSpeed = calcSpeed(); //slow ~100+us
//      Serial.print(currentPos); Serial.print('\t'); Serial.println(currentSpeed);
//      delay(1);
//    }
//    Serial.print(usedMeasurements[j]); Serial.print(" in "); Serial.print(endTime[j]-startTime[j]); Serial.print("=>"); Serial.println((endTime[j]-startTime[j])/usedMeasurements[j]);
//  }
//  while(1);
  

  lastAngle = twoWireReadTwoBytes(0x0C);
  lastPosTime = micros();
}

const uint8_t timerCount = 10;
uint32_t timers[timerCount][4];
uint8_t timerItt = 0;

void loop() {
  timers[timerItt][0] = micros();
  
  uint32_t loopStart = micros();
  int16_t newAngle = twoWireReadTwoBytes(0x0C); // ~75us
  timers[timerItt][1] = micros();
  
  currentPos = updatePos(currentPos, newAngle);
  //posHistFIFO.put(currentPos); //~10us(?)
  
  float newRawSpeed = ((currentPos - lastPos)*1000.0) / (loopStart - lastPosTime); //calculate speed in ADCticks per millisecond
  lastPos = currentPos;   lastPosTime = loopStart;
  speedHistFIFO.put(newRawSpeed);
  currentSpeed = calcSpeed(); //slow ~100+us
  timers[timerItt][2] = micros();
  
  float motPow = posToMotPow(currentPos, desiredPos, currentSpeed);
  applyMotPow(motPow); //~40us
  timers[timerItt][3] = micros();
  
  
  timerItt++;
  if(timerItt >= timerCount) {
    timerItt = 0;
//    for(uint8_t i=0; i<timerCount; i++) {
//      for(uint8_t j=0; j<3; j++) {
//        Serial.print(timers[i][j+1] - timers[i][j]); Serial.print(' ');
//      }
//      Serial.println();
//    }
//    Serial.println((timers[timerCount-1][3] - timers[0][0])/timerCount);
  }

  //Serial.print(currentPos); Serial.print('\t');
  //Serial.print(motPow); Serial.print('\t');
  //Serial.println(currentSpeed);

  uint32_t loopTime = (micros() - loopStart);
  if(loopTime < (500-5)) {
    delayMicroseconds((500-5) - loopTime);
  } else {
    //Serial.print("didn't make loopTime target:"); Serial.println(loopTime);
  }
}
