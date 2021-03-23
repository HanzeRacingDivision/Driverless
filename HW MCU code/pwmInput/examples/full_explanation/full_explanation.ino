/*
 library for reading values from PWM sources like RC receivers
 specifically designed for the ESP32
 this example sketch showcases all functions and configurations
*/
#include <pwmInput.h>

#define pin 5  //digital pin, digitalPinToInterrupt() is used, but on the ESP32 it just returnes the same value

#define useDeadZones (false) //whether to use deadzones at the middle and at the extremes. Provides a more game-like feel and fixes small middle-stick offsets
#define useDeadzoneStretching (false) //deadzones reduce resolution, if this is true the output will be stretched (using map()) to extend back to minPwmVal/maxPwmVal
#define minPwmVal 950 //minimum pulse width (used to constrain() output)
#define maxPwmVal 2050 //minimum pulse width (used to constrain() output)
#define fullDeadThresh 100 //deadzone size (a.k.a. threshold) at extremes (minPwm/maxPwm)
#define midDeadThresh 50 //deadzone size at middle
#define midPwmVal 1500  //middle PWM value
//#define midPwmVal 0   //if set to 0 (or any value below minPwmVal), it is automatically calculated with  minPwmVal+((maxPwmVal-minPwmVal)/2)

pwmInput throttle(pin, useDeadZones, useDeadzoneStretching, minPwmVal, maxPwmVal, fullDeadThresh, midDeadThresh, midPwmVal);
//pwmInput throttle(pin); //defaults are (, false, false, 950, 2050, 100, 50, 0)

//// you can construct arrays of pwmInputs like this
//pwmInput channels[4] = {pwmInput(33, true), //Aileron
//                        pwmInput(32, true), //Elevator
//                        pwmInput(39, true), //Throttle
//                        pwmInput(36, true)};//Rudder

void setup() {
  Serial.begin(115200);
  throttle.init(); //you need to initialize the pin (will call pinMode() and setup the interrupt)
  //throttle.init(INPUT_PULLUP); //you can enter the pinMode() argument here if you want, defualt is (INPUT)

  //print out the currently set attributes
                      Serial.print(throttle.pwmInputPin); //pin
  Serial.print(' ');  Serial.print(throttle.deadZones);   //(bool) whether or not to use deadzones (game-like non-continuous input)
  Serial.print(' ');  Serial.print(throttle.deadzoneStretching); //(bool) deadzones takes away resolution, this stretches PWM() out to extend to minVal/maxVal
  
  //Serial.print(' ');  Serial.print(throttle.minVal); //raw, does not consider changes by useDeadzones and deadzoneStretching
  //Serial.print(' ');  Serial.print(throttle.maxVal); //raw, does not consider changes by useDeadzones and deadzoneStretching
  Serial.print(' ');  Serial.print(throttle.minPWMval()); //considers deadZones and deadzoneStretching (class attributes)
  Serial.print(' ');  Serial.print(throttle.maxPWMval()); //considers deadZones and deadzoneStretching (class attributes)
  //Serial.print(' ');  Serial.print(throttle.minPWMval(!useDeadZones, !useDeadzoneStretching)); //considers entered deadZones and deadzoneStretching booleans
  //Serial.print(' ');  Serial.print(throttle.maxPWMval(!useDeadZones, !useDeadzoneStretching)); //considers entered deadZones and deadzoneStretching booleans

  Serial.print(' ');  Serial.print(throttle.stickFullThreshold); //deadzone size (a.k.a. threshold) at extremes (minPwm/maxPwm)
  Serial.print(' ');  Serial.print(throttle.stickMiddleThreshold); //deadzone size at middle
  Serial.print(' ');  Serial.print(throttle.midVal); //middle PWM value
  Serial.println();
  Serial.println("waiting for first value...");
  uint32_t lastRisingTime = throttle._risingTime;
  while(throttle._risingTime == lastRisingTime) {/*wait*/} //_risingTime is a value that stores micros() at (rising) interrupt, so it will update when
  delayMicroseconds(throttle.maxVal); //not necessary, but it's safer to allow time for the input pulse drop back down to produce the first pwmValue
  Serial.println();
}

void loop() {
  //there are several ways to read out the data
  //as a pulse width
                      Serial.print(throttle.pwmValue); //raw value (volatile integer set in the interrupt function), no deadzones, but still constrained by min/max values
  Serial.print(' ');  Serial.print(throttle.PWM()); //returns pwmValue, but considers deadZones and deadzoneStretching (class attributes)
  Serial.print(' ');  Serial.print(throttle.PWM(!useDeadZones, !useDeadzoneStretching)); //returns pwmValue, but considers entered deadZones and deadzoneStretching booleans
  //as a float
  Serial.print(' ');  Serial.print(throttle.normalized()); //normalizes pwmValue to between -1.0 and 1.0
  Serial.print(' ');  Serial.print(throttle.normalized(false)); //normalizes pwmValue to between 0.0 and 1.0
  Serial.print(' ');  Serial.print(throttle.normalized(true, !useDeadZones)); //normalizes pwmValue and considers the entered deadZones boolean
  
  Serial.print(' ');  Serial.print(throttle.timeSinceLastUpdate()); //time since last (rising) interrupt in microseconds (be careful of rollover after 71.58 minutes)
  Serial.println();
  delay(100);
}
