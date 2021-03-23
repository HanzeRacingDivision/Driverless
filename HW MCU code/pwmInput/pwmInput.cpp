/*
 *  pwmInput.cpp Library for getting simple inputs from drone receivers and things like that
 *  the current version is specifically for the ESP32
 *
 *  Created by Thijs van Liempd on 19/03/2021.
 *  imperfect, but potentially functional
 */
#pragma once

#include "pwmInput.h"

IRAM_ATTR void pwmRecordingChangeInterrupt(void* arg) {
  pwmInput* channel = static_cast<pwmInput*>(arg); //cast to pwmInput class
  if(digitalRead(channel->pwmInputPin)) { //on the ESP32, digitalRead() is fast enough that this makes more sense than using 2 seperate interrupt functions
    channel->_risingTime = micros();
  } else {
    uint32_t unconstrainedVal = micros() - channel->_risingTime;
    channel->pwmValue = constrain(unconstrainedVal, channel->minVal, channel->maxVal);
    //channel->pwmValue = unconstrainedVal; //slightly faster, but much more dangerous when it comes to missed pulses and floating pins
  }
}

pwmInput::pwmInput(int pin, bool useDeadZones, bool useDeadzoneStretching, uint32_t minPwmVal, uint32_t maxPwmVal, uint32_t fullDeadThresh, uint32_t midDeadThresh, uint32_t midPwmVal) {
  pwmInputPin = pin;
  deadZones = useDeadZones;
  minVal = minPwmVal;
  maxVal = maxPwmVal;
  stickFullThreshold = fullDeadThresh;
  stickMiddleThreshold = midDeadThresh;
  deadzoneStretching = useDeadzoneStretching;
  if((midPwmVal > minPwmVal) && (midPwmVal < maxPwmVal)) { //if a (good) value was entered as an argument
    midVal = midPwmVal;
  } else {
    midVal = minVal + ((maxVal-minVal)/2);
  }
  pwmValue = midVal; //safety
}

bool pwmInput::init(uint8_t pinModeArg) {
  pinMode(pwmInputPin, pinModeArg);
  attachInterruptArg(digitalPinToInterrupt(pwmInputPin), pwmRecordingChangeInterrupt, this, CHANGE);
}

uint32_t pwmInput::minPWMval() {
  if(deadZones && (!deadzoneStretching)) {
    return(minVal+stickFullThreshold+stickMiddleThreshold);
  } else {
    return(minVal);
  }
}

uint32_t pwmInput::maxPWMval() {
  if(deadZones && (!deadzoneStretching)) {
    return(maxVal-stickFullThreshold-stickMiddleThreshold);
  } else {
    return(maxVal);
  }
}

uint32_t pwmInput::minPWMval(bool useDeadZones, bool useDeadzoneStretching) {
  if(useDeadZones && (!useDeadzoneStretching)) {
    return(minVal+stickFullThreshold+stickMiddleThreshold);
  } else {
    return(minVal);
  }
}

uint32_t pwmInput::maxPWMval(bool useDeadZones, bool useDeadzoneStretching) {
  if(useDeadZones && (!useDeadzoneStretching)) {
    return(maxVal-stickFullThreshold-stickMiddleThreshold);
  } else {
    return(maxVal);
  }
}

uint32_t pwmInput::PWM() {
  if(deadZones) {
    if(pwmValue >= (midVal+stickMiddleThreshold)) { //above middle
      if(pwmValue < (maxVal-stickFullThreshold)) {
        if(deadzoneStretching) {
          return(map(constrain(pwmValue, midVal+stickMiddleThreshold, maxVal-stickFullThreshold), midVal+stickMiddleThreshold, maxVal-stickFullThreshold, midVal, maxVal));
        } else {
          return(pwmValue-stickMiddleThreshold);
        }
      } else {
        if(deadzoneStretching) {
          return(maxVal);
        } else {
          return(maxVal-stickFullThreshold-stickMiddleThreshold);
        }
      }
    } else if(pwmValue <= (midVal-stickMiddleThreshold)) { //below middle
      if(pwmValue > (minVal+stickFullThreshold)) {
        if(deadzoneStretching) {
          return(map(constrain(pwmValue, minVal+stickFullThreshold, midVal-stickMiddleThreshold), minVal+stickFullThreshold, midVal-stickMiddleThreshold, minVal, midVal));
        } else {
          return(pwmValue+stickMiddleThreshold);
        }
      } else {
        if(deadzoneStretching) {
          return(minVal);
        } else {
          return(minVal+stickFullThreshold+stickMiddleThreshold);
        }
      }
    } else { //at middle
      return(midVal);
    }
  } else {
    return(pwmValue);
  }
}

uint32_t pwmInput::PWM(bool useDeadzones, bool useDeadzoneStretching) {
  if(useDeadzones) {
    if(pwmValue >= (midVal+stickMiddleThreshold)) { //above middle
      if(pwmValue < (maxVal-stickFullThreshold)) {
        if(useDeadzoneStretching) {
          return(map(constrain(pwmValue, midVal+stickMiddleThreshold, maxVal-stickFullThreshold), midVal+stickMiddleThreshold, maxVal-stickFullThreshold, midVal, maxVal));
        } else {
          return(pwmValue-stickMiddleThreshold);
        }
      } else {
        if(useDeadzoneStretching) {
          return(maxVal);
        } else {
          return(maxVal-stickFullThreshold-stickMiddleThreshold);
        }
      }
    } else if(pwmValue <= (midVal-stickMiddleThreshold)) { //below middle
      if(pwmValue > (minVal+stickFullThreshold)) {
        if(useDeadzoneStretching) {
          return(map(constrain(pwmValue, minVal+stickFullThreshold, midVal-stickMiddleThreshold), minVal+stickFullThreshold, midVal-stickMiddleThreshold, minVal, midVal));
        } else {
          return(pwmValue+stickMiddleThreshold);
        }
      } else {
        if(useDeadzoneStretching) {
          return(minVal);
        } else {
          return(minVal+stickFullThreshold+stickMiddleThreshold);
        }
      }
    } else { //at middle
      return(midVal);
    }
  } else {
    return(pwmValue);
  }
}

float pwmInput::normalized(bool plusMinusOne) {
  if(plusMinusOne) {
    int32_t PWMval = ((int32_t) PWM(deadZones, false)) - midVal; //deadzones are applied in PWM()
    int32_t PWMmax = maxPWMval(deadZones, false) - midVal; //positive range
    int32_t PWMmin = midVal - minPWMval(deadZones, false); //negative range
    if(PWMval > 0) {
      return((float)PWMval / PWMmax);
    } else if(PWMval < 0) {
      return(PWMval / (float) PWMmin);
    } else {
      return(0);
    }
  } else {
    return( ((float)(PWM(deadZones, false) - minPWMval(deadZones, false))) / ((float)(maxPWMval(deadZones, false) - minPWMval(deadZones, false))));
  }
}

float pwmInput::normalized(bool plusMinusOne, bool useDeadZones) {
  if(plusMinusOne) {
    int32_t PWMval = ((int32_t) PWM(useDeadZones, false)) - midVal;  //deadzones are applied in PWM()
    int32_t PWMmax = maxPWMval(useDeadZones, false) - midVal; //positive range
    int32_t PWMmin = midVal - minPWMval(useDeadZones, false); //negative range
    if(PWMval > 0) {
      return((float)PWMval / PWMmax);
    } else if(PWMval < 0) {
      return(PWMval / (float) PWMmin);
    } else {
      return(0);
    }
  } else {
    return( ((float)(PWM(useDeadZones, false) - minPWMval(useDeadZones, false))) / ((float)(maxPWMval(useDeadZones, false) - minPWMval(useDeadZones, false))));
  }
}
