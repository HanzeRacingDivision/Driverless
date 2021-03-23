/*
 *  pwmInput.h Library for getting simple inputs from drone receivers and things like that
 *  the current version is specifically for the ESP32
 *  
 *  Created by Thijs van Liempd on 19/03/2021.
 *  imperfect, but potentially functional
 */
 
#ifndef pwmInput_h
#define pwmInput_h

#include "Arduino.h"

DRAM_ATTR class pwmInput
{
  public:
    volatile uint32_t pwmValue; //(raw) variable, to get this value but with deadzones, use 
    volatile uint32_t _risingTime = 0; //although it's a public value, it's not meant to be messed with
    
    int16_t pwmInputPin;  //digital pin
    uint32_t minVal; //constrains within ISR
    uint32_t midVal; //used for normalized() as the midpoint
    uint32_t maxVal; //constrains within ISR
    bool deadZones; //for a drone-transmitter/game controller feel
    uint32_t stickMiddleThreshold; //middle +- this value is middle deadzone
    uint32_t stickFullThreshold; //middle +- this value is middle deadzone
    bool deadzoneStretching; //if true: PWM() output will shoot to min/max if threshold is crossed, if false: output will be constrained to (min+thresh)/(max-thresh)
    //the pulse length threshold line looks a little like this:
    // 0us////////////////////minPwmVal//stickFullThreshold////////stickMiddleThreshold//middle//stickMiddleThreshold////////stickFullThreshold//maxPwmVal//////////2500us
    
    pwmInput(int pin, bool useDeadZones=false, bool useDeadzoneStretching=false, uint32_t minPwmVal=950, uint32_t maxPwmVal=2050, uint32_t fullDeadThresh=100, uint32_t midDeadThresh=50, uint32_t midPwmVal=0);

    bool init(uint8_t pinModeArg=INPUT);

    uint32_t minPWMval(); //return minVal, considering deadZones and deadzoneStretching
    uint32_t maxPWMval(); //return minVal, considering deadZones and deadzoneStretching
    uint32_t minPWMval(bool useDeadZones, bool useDeadzoneStretching); //temporarily override class booleans
    uint32_t maxPWMval(bool useDeadZones, bool useDeadzoneStretching); //temporarily override class booleans
    uint32_t PWM(); //returns pwmValue, considering deadZones and deadzoneStretching
    uint32_t PWM(bool useDeadzones, bool useDeadzoneStretching); //temporarily override class booleans
    float normalized(bool plusMinusOne=true);
    float normalized(bool plusMinusOne, bool useDeadZones); //temporarily override class boolean
    uint32_t timeSinceLastUpdate() { return(micros() - _risingTime); }
};
#endif //pwmInput_h
