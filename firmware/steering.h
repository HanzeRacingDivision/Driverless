/*
this code is used to control the (wheelchair) motor that functions as a steering servo on the Hanze Racing Division (driverless) kart.
It is intended to un on an arduino nano (atmega328P) with the optiboot bootloader (better boot times from powerOff)
it should:
  use the BTN7960 H-bridge to control a large DC motor accurately
  read out the AS5600 absolute magnetic rotary encoder quickly and reliably
  communicate with a master computer to receive angle instructions and return angle (sensor) data.
  automatically find the center of its motion range (center the steering wheel)

note on centering:
 The steering only has a limited range on any car, and since the encoder is absolute, 
  you may get more consistent results by recalling an absolute angle as the center position.
 However!, if the motor belt EVER slips, or is reinstalled/adjusted in any way, 
  this calibration will become inaccurate.
 see 'define absoluteAngleCenter ...' for whether 

author (and probably curator): Thijs van Liempd
creation date: 17 Jan 2022
*/
#pragma once

#include "FIFO.h"   //a (temporary???) little class library for First In First Out arrays (should be semi-efficient)
#include "AS5600.h" //TBD: let HARD know where they can get all these cool, hip, sick, custom libraries

namespace steering { //this is to make clear to other sketches where the variables come from (and now i can use the same variablename in different namespaces)

//#define threeWireHbridge //seems to perform worse (less power, more noise) across the board, maybe EN switching time is too slow
//#define debugCenterCalib //print some fun debug info to help with centering


const uint8_t AS5600_SDApin = 22; //note: 1.5kOhm pullup resistors are strongly advised
const uint8_t AS5600_SCLpin = 17;
#define motENpin 2         //you could do a second (higher freq) PWM on this one to limit the power, but IDK if it'll work well (since threeWireHbridge sucks)
const uint8_t motApin = 16; //the ESP will let you do PWM on any output pin at any frequency and resolution
const uint8_t motBpin = 4; //i think 8bit PWM resolution is fine for an application like this.
#define motApin_PWM_channel 0
#define motBpin_PWM_channel 1
#define motPins_PWM_res 8     //same as arduino, because you dont need any more than this
#define motPins_PWM_freq 490  //same as arduino, doesnt need to be (but may affect maxDecel calibration slightly)

#define smoothTargetApproach  //reduce motor power near target to avoid vibrating around the target.
//#define quadraticApproach   //use quadratic (instead of sinusoidal) smoothing curve
const float maxDecel = 0.3000; //the deceleration charactaristics of the motor. Too high and you overshoot, too low and you undershoot (and vibrate at the end)
const float minDecelVal = maxDecel * 0.75; //the higher this value, the later the decel starts, so the faster you arrive (also more likely to overshoot)
const float overBrakeMult = 1.00; //the last multiplier applied to the motor power when braking. To under-brake, set to below 1.0
const float absMaxPower = 0.40; //(range = 0.0~1.0) you can limit the motor power to prevent personal harm (and belt slipping, of course)
//note: absMaxPower should not affect deceleration calibration (assuming it's sufficiently linear)

const int32_t targetMargin = 20; //(ADCunit) minimum angle error required for intervention (otherwise it would just keep oscillating around the target area forever)
#ifdef smoothTargetApproach
  const int32_t targetSmoothMargin = 200; //(ADCunit) angle error at which to start the smoothing (heavily affects target-area-oscillation)
  #ifdef quadraticApproach  // provides a quadratic curve near the target
    const float targetSmoothDivider = ((targetSmoothMargin*targetSmoothMargin)/targetMargin)-targetMargin;
  #else  // provides a sinusoidal curve near the target
    const float targetSmoothDivider = (targetSmoothMargin-targetMargin)/PI;
  #endif
#endif

// handy debug note: to debug if the motor direction is correct, use centerCalib()
#define quickCenterKnownRange 6930
//#define absoluteAngleCenter 1234 //(TBD) use absolute encode for consistent (calibrated) centering. You still need some kind of centerCalib function!
const uint32_t endFindTimeout = 4000; //(millis) if the speed is still above the threshold after this much time has passed, something is wrong (slipping belt???)
const float endFindInitialPower = absMaxPower * 0.4; //motor power to start edgefinding
const float endFindMaxPower = absMaxPower * 0.9; //max motor power when edgefinding (power it WILL reach when motor stalls)
const float endFindPowerStep = max(endFindMaxPower * 0.025, 1.0/255.0); //step size to increase power with (the max() function is to ensure minimum PWM resolution steps)
const uint32_t endFindTargetLoopTime = 1000; //(micros) will affect endFindPowerStep!
const float endFindSpeedThresh = 0.05; //if speed is less than this for a sufficient time (and power), the edge is considered to be found and the current position is returned
const float endFindPowerIncreaseSpeedThresh = endFindSpeedThresh * 3.0; //this should be higher than endFindSpeedThresh, or based on maxDecel.
const uint32_t endFindMinTotalTime = 500; //endFinding can't possibly take less time than this, give the motor a little bit to start moving...
const uint32_t endFindMinStallTime = 100; //if the speed has been below endFindSpeedThresh for at least this long (at max power), consider the edge to be found

const int32_t minExpectedEndRange = 5000; //give warning if centerCalib() finds a range less than this value (set to 0 to skip this check)
const bool quickCenterCalibDir = true; //whether to prefer positive or negative power calibration (shouldn't matter)


//////////////////// sensor library init ////////////////////
AS5600 sensor;
///////////////////////// variables //////////////////////////
int32_t desiredPos = 0; //the desired (instructed) position of the servo)
int32_t currentPos = 0; //the current position of the servo (may be replaced by posHistFIFO[0])
//FIFO<int32_t> posHistFIFO(5); //a First-In-First-Out array to store position history
FIFO<float> speedHistFIFO(4); //a First-In-First-Out array to store (unfiltered) speed history
float currentSpeed = 0.0; //based on a (filtered/dampened) calculation from the pos (current & hist)

bool centered = false; //at startup, the servo does not know its absolute position yet. Once a centerCalib function ran succesfully, this bool can be set to true.
int32_t calibratedEnds[2]; //holds the range determined by centerCalib(). Values will only be different if there is some sort of centerOffset defined. Measured alternative to quickCenterKnownRange

const uint16_t maxSingleDif = 4096/2; //(nyquist?) to detect rollover in a quick'n'dirty way
int16_t lastAngle = 0; //used for position (and rollover) calculation
int32_t lastPos = 0; //used for speed calculation
uint32_t lastPosTime; //used for speed calculation


float calcSpeed() { //note: returns in the 'unit' of ADCunit per millisecond (or whatever unit you store in speedHistFIFO). To convert to SI unit, use ADC resolution
  float sum = 0;
  for(uint8_t i=0; i<speedHistFIFO.size; i++) {
    sum += speedHistFIFO._data[i]; //the order does not matter, so directly interfacing with the (unordered) FIFO data may be faster
  }
  return(sum / speedHistFIFO.size);
  //note: recalculating the whole thing every time is painfully slow. You could also just remove the oldest data from the (global var) sum and add the newest, but you'd run into floating point inaccuracies
}

#ifdef smoothTargetApproach
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

void applyMotPow(float motPow=0.0) {
  int16_t tempVal = motPow*255.0;
  #ifdef threeWireHbridge
    digitalWrite(motApin, tempVal > 0);
    digitalWrite(motBpin, tempVal < 0);
    ledcWrite(motApin_PWM_channel, constrain(abs(tempVal), 0, 255));
  #else
    ledcWrite(motApin_PWM_channel, constrain(tempVal, 0, 255));
    ledcWrite(motBpin_PWM_channel, constrain(-tempVal, 0, 255));
    #ifdef motENpin
      digitalWrite(motENpin, HIGH);
      //digitalWrite(motENpin, abs(tempVal) > 0); //this is idling instead of braking, not what we need
    #endif
  #endif
}

int32_t findRangeEnd(bool powerDirection) { //find one of the movement range limits (by running into it, hopefully gently)
  #ifdef debugCenterCalib
    Serial.println();
    Serial.println("findRangeEnd starting!");
  #endif
  uint32_t endFindStartTime = millis(); //used for endFindTimeout
  bool keepFinding = true; //set to false when end-found parameters are all met
  uint32_t endFindSpeedThreshTimer;
  bool endFindSpeedThreshTimerStarted = false;
  float endFindMotorPower = endFindInitialPower * (powerDirection ? 1.0 : -1.0);
  applyMotPow(endFindMotorPower);
  while(((millis() - endFindStartTime) < endFindTimeout) && keepFinding) {
    uint32_t loopStart = micros();
    currentPos = updatePos(currentPos, sensor.getAngle()); // ~80us on arduino, ~130us on ESP32
    float newRawSpeed = ((currentPos - lastPos)*1000.0) / (loopStart - lastPosTime); //calculate speed in ADCticks per millisecond
    lastPos = currentPos;   lastPosTime = loopStart;
    speedHistFIFO.put(newRawSpeed);
    currentSpeed = calcSpeed(); //slow ~100+us on arduino, 
    #ifdef debugCenterCalib
      Serial.print(currentPos); Serial.print('\t'); Serial.print(currentSpeed); Serial.print('\t'); Serial.print(endFindMotorPower); Serial.print('\t'); Serial.println(endFindSpeedThreshTimer);
    #endif
    if(abs(currentSpeed) < endFindSpeedThresh) { //if the speed is sufficiently low
      if(!endFindSpeedThreshTimerStarted) {
        endFindSpeedThreshTimerStarted = true;
        endFindSpeedThreshTimer = millis();
        #ifdef debugCenterCalib
          Serial.print("endFindSpeedThreshTimerStarted "); Serial.println(endFindSpeedThreshTimer);
        #endif
      }
      if(((millis() - endFindSpeedThreshTimer) > endFindMinStallTime) //if the speed threshold has been maintained for a while
         && ((millis() - endFindStartTime) > endFindMinTotalTime)                 //and the loop overall has gone on long enough (giving motor time to start moving
         && (abs(endFindMotorPower) >= (endFindMaxPower * 0.9))) {   //and at least 90% of endFindMaxPower
        keepFinding = false; //the edge has been found
        #ifdef debugCenterCalib
          Serial.println("EDGE FOUND");
        #endif
      }
    } else {
      endFindSpeedThreshTimerStarted = false;
    }
    if(abs(currentSpeed) < endFindPowerIncreaseSpeedThresh) { //if the motor is going slow (stalled or hasn't started moving)
      endFindMotorPower += endFindPowerStep * (powerDirection ? 1.0 : -1.0); //increase the power a little
      endFindMotorPower = constrain(endFindMotorPower, -endFindMaxPower, endFindMaxPower);
    }
    applyMotPow(endFindMotorPower);
    uint32_t timeSpent = micros()-loopStart;
    if(timeSpent < (endFindTargetLoopTime-5)) { delayMicroseconds((endFindTargetLoopTime-5) - timeSpent); } //calibrated delay to make preferred sample interval
  }
  int32_t endPos = currentPos; //record currentPos before power is turned off again (should make very little difference, but still)
  applyMotPow(0.0); //turn off motor power
  if(keepFinding) {
    Serial.print("findRangeEnd("); Serial.print(powerDirection); Serial.println(") timed out!");
  }
  return(endPos);
}

void quickCenterCalib() { //move the servo to only 1 of the ends of the steering range to make its position absolute
  int32_t endPos = findRangeEnd(quickCenterCalibDir);
  #ifdef quickCenterKnownRange
    currentPos = quickCenterKnownRange * (quickCenterCalibDir ? 0.5 : -0.5); //currentPos is just half the total range
    centered = true;
    calibratedEnds[0] = quickCenterKnownRange/-2;
    calibratedEnds[1] = quickCenterKnownRange/2;
    //#ifdef centerOffset //etc
  #elif defined(absoluteAngleCenter)
    //TBD
    #error("absoluteAngleCenter code TBD");
  #else
    #warning("WARNING: do NOT use quickCenterCalib, as you're lacking the calibration data to make it work. see quickCenterKnownRange or absoluteAngleCenter")
    Serial.println("WARNING, cannot use quickCenterCalib() without quickCenterKnownRange or absoluteAngleCenter!");
    centered = false;
  #endif
}

int32_t centerCalib() { //move the servo to both ends of the steering range to make its position absolute
  #ifdef debugCenterCalib
    Serial.println("centerCalib() starting");
  #endif
  int32_t endPositions[2];
  endPositions[0] = findRangeEnd(false);
  endPositions[1] = findRangeEnd(true);
  int32_t range = endPositions[1] - endPositions[0];
  #ifdef debugCenterCalib
    Serial.print("centerCalib() debug: "); Serial.print(endPositions[0]); Serial.print(' '); Serial.println(endPositions[1]);
  #endif
  currentPos = range/2; //only works if findRangeEnd( TRUE ) is done last, otherwise it'd be (-range/2)
  centered = true;
  calibratedEnds[0] = range/-2;
  calibratedEnds[1] = range/2;
  //#ifdef centerOffset //etc
  if((minExpectedEndRange > 0) and (range < minExpectedEndRange)) { //only perform this check if minExpectedEndRange is nonzero
    Serial.print("centerCalib warning: range below minExpectedEndRange: "); Serial.println(range);
    //centered = false;
  } else if(range <= 0) {
    Serial.print("centerCalib warning: range<=0 : "); Serial.println(range);
    centered = false;
  }
  #ifdef quickCenterKnownRange
    if(range < (0.9 * ((float)quickCenterKnownRange))) {
      Serial.print("centerCalib warning: range smaller than quickCenterKnownRange: "); Serial.println(range);
      //centered = false;
    }
  #endif
  return(range);
}

bool steeringInit() {
//  pinMode(AS5600_SDApin, INPUT_PULLUP); //not needed, as twoWireSetup() does the pullup stuff for you
//  pinMode(AS5600_SCLpin, INPUT_PULLUP);
  sensor.init(1000000, AS5600_SDApin, AS5600_SCLpin); //on the ESP32 (almost) any pins can be I2C pins
  //note: on the ESP32 the actual I2C frequency is lower than the set frequency (by like 20~40% depending on pullup resistors, 1.5kOhm gets you about 800kHz)

  // TBD: sensor connection test with some form of feedback that THE STEERING SENSOR ISNT FRICKING WORKING (flashing LED_BUILTIN, which is motorEN, with motA and motB both LOW?)
  
  sensor.resetConfig();
  //sensor.setSF(3); //set slow-filter to the mode with the least delay
  //sensor.setFTH(7); //set fast filter threshold to something... idk yet
  //sensor.printConfig();

  pinMode(motApin, OUTPUT);
  pinMode(motBpin, OUTPUT);
  #ifdef motENpin
    pinMode(motENpin, OUTPUT);
  #endif
  #ifdef threeWireHbridge
    ledcSetup(motApin_PWM_channel, motPins_PWM_freq, motPins_PWM_res);
    ledcAttachPin(motENpin, motApin_PWM_channel);
  #else
    ledcSetup(motApin_PWM_channel, motPins_PWM_freq, motPins_PWM_res);
    ledcAttachPin(motApin, motApin_PWM_channel);
    ledcSetup(motBpin_PWM_channel, motPins_PWM_freq, motPins_PWM_res);
    ledcAttachPin(motBpin, motBpin_PWM_channel);
  #endif

  lastAngle = sensor.getAngle();
  lastPosTime = micros();

  //quickCenterCalib();
  int32_t foundRange = centerCalib();
//  #warning("steering calibration (and in general) is disabled for debugging!")
  //Serial.print("foundRange:"); Serial.println(foundRange);
  //delay(250);

  return(true); //all went well, return true
}

bool steeringLoop() {  //(uint32_t loopStart) {
  uint32_t loopStart = micros();
  int16_t newAngle = sensor.getAngle(); // ~80us on arduino, ~130us on ESP32
  
  currentPos = updatePos(currentPos, newAngle);
  //posHistFIFO.put(currentPos); //~10us(?) on arduino
  
  float newRawSpeed = ((currentPos - lastPos)*1000.0) / (loopStart - lastPosTime); //calculate speed in ADCticks per millisecond
  lastPos = currentPos;   lastPosTime = loopStart;
  speedHistFIFO.put(newRawSpeed);
  currentSpeed = calcSpeed(); //slow ~100+us on arduino, ~5us on ESP32
  
  float motPow = posToMotPow(currentPos, desiredPos, currentSpeed);
  applyMotPow(motPow); //~40us on arduino, ~10us on ESP32

  return(true); //all went well, return true
}

} // namespace steering
