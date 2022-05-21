/*
this ESP32 will handle emulating the pedal, to make the kart-specific ESC controllable electronically.
(this basically only requires a DAC (and an ADC if you want to use the existing pedal as well))
This ESP will also do the wheel speed measurements (and communicate them to where they're needed),
To make things easier (hopefully) i'll try to implement a constant velocity control loop on this thing,
 but i can't promise it'll work well.

The kart has a big'ol switch for reverse 'gear'. This switch uses (for some reason) 60V signals.
Furthermore, on the kart's dashboard there is a speed select (key)switch.
Both of these things can be replaced by a series of mosfets/relays/Solid-State-Relays (triac?),
 if i want to. I'm not sure i want to. It seems prudent to have this ESP check whether the reverse switch is on,
 but it's not a true necessity either. We'll see how much time/energy i have to spare
That being said, a little resistor-divider to measure battery voltage seems like a quick and useful feature. IDK.

the brakes on the kart are hydraulic, so i have no intention of using those.
Instead, i'll try to get a little motor-breaking to work.
I don't know if the ESC will let me, but i'm damn well gonna try.
Once motor braking works, the control loop may become a tad more complex... :)

The pedal in the kart appears to be some kind of active component, not just a potentiometer.
It still functions like a pot (providing a 0~5v output voltage based on how far the pedal is pressed),
 but it seems to have some deadzone, and to let the output pin float (instead of pulling it) to 5V.
This ESP does not NEED to mimic this wacky behavior, unless testing suggests otherwise.
The oscilloscope showed some high frequency (and very short) voltage drops on the output signal,
 i'm guessing this is some kind of bad feedback from the ESC, rather than something the pedal does on purpose.
In any case, the voltage (and current) required to drive the pedal input on the ESC are more than the ESP's DAC can handle,
 so i'm using a Rail-To-Rail opAmp to multiply the voltage by a factor of 5/3.3=1.51515. (google 'voltage follower' or 'buffer amplifier')
 The opAmp math works out to: Vout = Vin * (1 + (R1 / R2))
 so i'll be using 34.8k and 68k resistors and an MCP6004 RTR opAmp to get this done.
The pedal is at 5V when idle, and 0+? volt when pressed

DAC testing:
it seems the ESC does NOT want the input voltage to go above 3.85V. Maybe the input is supposed to be 3.3V and somebody bungled the specs, maybe it's something else
either way, by setting DACmotorMax to (3.8/5)*255 = 200ish, this problem is solved
furthermore max throttle saturation happens at around 3.5V
and the wheels start to move (barely) at 1.5V

as for the wheel speed sensors...
*/
#pragma once

//#define abs(x) ((x)>0?(x):-(x)) //makes abs() work with floats
#include "thijsFIFO.h"   //a (temporary???) little class library for First In First Out arrays (should be semi-efficient)
#include <driver/dac.h>

namespace throttle { //this is to make clear to other sketches where the variables come from (and now i can use the same variablename in different namespaces)

//uint32_t debugPrintTimer; //deleteme
//const uint32_t debugPrintTimout = 6150; //deleteme

#define THR_realPedalPassthrough   //allows pedal to detemine (override) motor power/speed (again). Use realPedalAsTargetSpeed if you want to pedal to determine targetSpeed
#define THR_printExtraDebug    // i strongly recommend enabling this, as the pedal passthrough has some electrical problems (interference?noise?bad ADC?)
//#define THR_wheelEncodersPulseCounterHW  //using the pulseCounter HW units built into the ESP32 instead of (CPU-taxing) interrupts

// usefull macros: DAC_CHANNEL_1   DAC_GPIO25_CHANNEL   DAC_CHANNEL_1_GPIO_NUM
const dac_channel_t DACchannel = DAC_CHANNEL_1;  //ch1 is GPIO25, ch2 is GPIO26
//#define    THR_DACinvert  //invert DAC output, to make 0V == 255 and vice versa.  Useful if ESC is active LOW, or if you use an inverting amplifier

const uint8_t DACmotorIdle = 40;
//const uint8_t DACmotorMinThrottle = 75; //motor starts moving here (in low 'gear', at least)
const uint8_t DACmotorMax = 200; //motor will shut off if voltage goes above ~3.85V, so limit it to

//#define brakePin 15
//#define brakeActiveState HIGH

const uint32_t motorControlInterval = 100; //time between control system updates in millis. The shorter this is, the more aggresive motorKpAdj will be used
uint32_t motorControlTimer;

// output = (targetSpeed * motorKpBase) + ((targetSpeed - mesSpeed) * motorKpAdj) + some other things
const float motorKpBase = 106.0; //motorKpBase will be based on rudimentary calibration (hopefully linear)
const float motorKpAdj = 0.0; //motorKpAdj should be small to reduce overshoot, but big enough to prevent underperforming steady-states (at very low/high speeds)
//const float motorKpSteerPolynomial[3] = {0.2877, 0.0387, 0.0156}; //format: {c, b, a}, asin ax^2 + bx + c
//const float motorKpVoltSlope = ; //adjustment based on battery voltage, lower voltage -> higher motor power (compensating)
//const float motorKpVoltOffset = ; //like y=ax+b, where y=KpVolt, a==slope and b==offset

float targetSpeed; //target speed in m/s
const float targetSpeedLimits[2] = {-0.01, 1.5}; //(target) speed limits in m/s
const float minTargetSpeed = 0.05; //minimum target speed for the motor to function (in m/s)

//////////////////////////////// speed sensors: ////////////////////////////////
#ifdef THR_wheelEncodersPulseCounterHW
  #error("pulseCounter HW TBD, you'll have to just use interrupts for now");
#else
  //#warning("not using THR_wheelEncodersPulseCounterHW is inefficient, all these interrupts may affect the speed!");
  #define interruptCounterInstantInit
  class interruptCounter {
    public:
    uint32_t count = 0;
    #ifdef interruptCounterInstantInit
      const uint8_t pin;  const int intr_type;  const bool pullup;
      interruptCounter(uint8_t pin, int intr_type, bool pullup=false) : pin(pin), intr_type(intr_type), pullup(pullup) { pinMode(pin, pullup ? INPUT_PULLUP : INPUT); attachInterruptArg(pin, isr, &count, intr_type); }
      ~interruptCounter() { detachInterrupt(pin); }
    #else
      uint8_t _pin;
      //interruptCounter(uint8_t pin
      void init(uint8_t pin, int intr_type, bool pullup=false) { _pin=pin; pinMode(pin, pullup ? INPUT_PULLUP : INPUT); attachInterruptArg(pin, isr, &count, intr_type); }
      ~interruptCounter() { detachInterrupt(_pin); }
    #endif
    static void ARDUINO_ISR_ATTR isr(void* counterArg) {
      uint32_t* counter = static_cast<uint32_t*>(counterArg); // = (uint32_t*)counterArg;
      (*counter) += 1;
    }
  };
  
//  #ifdef interruptCounterInstantInit
//    interruptCounter test(34, FALLING, false);
//  #else
//    interruptCounter test;
//  #endif
#endif

//const uint8_t wheelEncoderPins[4] = {34, 35, 36, 39}; //these are input-only pins, capable of using interrupts and/or the PulseCounter
interruptCounter wheelEncoders[4] = {{34,FALLING}, {35,FALLING}, {36,FALLING}, {39,FALLING}}; //custom pulse-counter structs
uint32_t encoCountLast[4]; // the encoder classes do not remember the last encoder position (for good reason), so store that as well
uint32_t lastSpeedCalcTime; // timestamp at which the speed was last calculated
const float wheelCircumference = PI*0.30; // (meters) circumference = 2*PI*radius = PI * diameter
const float encoCountToMeters[4] = {wheelCircumference/12.0, wheelCircumference/12.0, wheelCircumference/12.0, wheelCircumference/12.0}; // use number of encoder pulses per rotation and circumference

const uint8_t FIRfilterDepth = 4;
const float FIRfilterStrengths[FIRfilterDepth] = {0.6, 0.25, 0.10, 0.05};

// the encoder positions are stored in the encoder structs
const uint8_t speedSensFIFOsize = 10; // how many historical datapoints to store (should be >= FIRfilterDepth)
FIFO<float> speedSensFIFOs[4] = {{speedSensFIFOsize}, {speedSensFIFOsize}, {speedSensFIFOsize}, {speedSensFIFOsize}}; //initialization of each individual FIFO (set FIFO size)
float currentForwSpeed = 0.0; // a (somewhat simplified) speed number, which is used by the feedback loop that handles throttle control
//float currentAltSpeed = 0.0; // some more shit calculated from the wealth of wheelspeed data.. (idk man, i feel like you should be able to glean some traction info or something)
//float tractionApprox = 1.0; // an approximation of the current traction (0.0~1.0), based on the multi-wheel speed data

/* data to be transmitted to host device(s):
 * current encoder counters (raw data is easier to transmit AND allows trying out several interpretations easier)
 * (locally calculated) speed and traction data? i think it would also make a lot of sense to just interpret this data on the host computer IF the communication rate is sufficiently high (it should be)
 */

#ifdef THR_realPedalPassthrough
  // the pedal seems to output 0.83->3.86V (@ 4.65V as 5V input), so the ADC should probably also do something like that
  #define THR_realPedalAsTargetSpeed  //uses the pedal to set targetSpeed (not motor power directly).
  const uint16_t realPedalADCrange[2] = {500, 3300}; //only used with realPedalAsTargetSpeed (for now?)
  const uint8_t realPedalADCpin = 33; //using an ADC1 (not ADC2) pin means it can function alongside wifi/BLE (when ADC2 is unavailable)
  //FIFO<uint16_t> realPedalFIFO(10); //a First-In-First-Out array to store (unfiltered) realPedalADC history
  const uint16_t realPedalADCthreshold = 550; //if the (ADC value of the) pedal gets ABOVE this threshold, start listening to it.
#endif

#ifdef THR_DACinvert
  #define DACwrite(val)  dac_output_voltage(DACchannel, 255-val);
#else
  #define DACwrite(val)  dac_output_voltage(DACchannel, val);
#endif

void throttleInit() {
  //pinMode(DACpin, OUTPUT); //does not seem to be needed
  dac_output_enable(DACchannel);
  DACwrite(DACmotorIdle);

  #ifdef brakePin
    pinMode(brakePin, OUTPUT);
    digitalWrite(brakePin, brakeActiveState); //turn on brakes
  #endif
  
  #ifdef THR_realPedalPassthrough
    pinMode(realPedalADCpin, INPUT);
  #endif
}

template<typename T> T FIR(FIFO<T>& inputFIFO, const T strengths[], uint8_t depth) {
  T result = 0;  for(uint8_t i=0; i<depth; i++) { result += inputFIFO[i] * strengths[i]; }
  return(result);
}

void updateSpeeds() {
  // consider all 4 wheel speeds, and (using some historical data to smooth it,) produce a (few?) useful numbers
  //  update: speedSensFIFOs  currentForwSpeed  and?  currentAltSpeed  tractionApprox
  uint32_t newSpeedCalcTime = micros();
//  if(millis() < debugPrintTimer) { /*Serial.print(newSpeedCalcTime - lastSpeedCalcTime); Serial.print('\t');*/ Serial.println(wheelEncoders[0].count); } // speed calibration/debug
  for(uint8_t i=0; i<4; i++) {
    uint32_t capturedEncoCount = wheelEncoders[i].count; //capture encoCount, becuase it may change while you're using it
    float newRawSpeed = (capturedEncoCount - encoCountLast[i]); // start with difference in encoderCount
    newRawSpeed /= (newSpeedCalcTime - lastSpeedCalcTime)*0.000001; //now you have to encoderCount/second
    newRawSpeed *= encoCountToMeters[i]; //now you have to meters/second
    speedSensFIFOs[i].put(newRawSpeed);
    encoCountLast[i] = capturedEncoCount;
  }
  // TBD: google how differentials work (mathematically) and how to verify traction using individual wheel data
  if(speedSensFIFOs[0].filled) {
    //currentForwSpeed = FIR<float>(speedSensFIFOs[0], FIRfilterStrengths, speedSensFIFOs[0].filled ? FIRfilterDepth : ((speedSensFIFOs[0]._cursor<FIRfilterDepth) ? speedSensFIFOs[0]._cursor : FIRfilterDepth)); // a simple FIR filter
    currentForwSpeed = FIR<float>(speedSensFIFOs[0], FIRfilterStrengths, FIRfilterDepth); // a simple FIR filter
  }
  lastSpeedCalcTime = newSpeedCalcTime;
}

bool throttleLoop() {
  #ifdef THR_realPedalPassthrough
    uint16_t realPedalADCval = analogRead(realPedalADCpin);
    if(realPedalADCval >= realPedalADCthreshold) {
      #ifdef THR_printExtraDebug
        Serial.print("real pedal is in control: "); Serial.print(realPedalADCval); Serial.print(" -> ");
      #endif
      #ifdef THR_realPedalAsTargetSpeed
        targetSpeed = (targetSpeedLimits[1] * (realPedalADCval - realPedalADCrange[0])) / ((float) (realPedalADCrange[1]-realPedalADCrange[0])); //convert the pedal's (inverted) ADC value to a targetSpeed
        targetSpeed = constrain(targetSpeed, targetSpeedLimits[0], targetSpeedLimits[1]);
        #ifdef THR_printExtraDebug
          Serial.println(targetSpeed);
        #endif
      #else
        #ifdef THR_printExtraDebug
          Serial.println(realPedalADCval >> 4);
        #endif
        //DACwrite(DACmotorIdle >> 4); //convert 12bit ADC value into 8bit DAC value directly
        dac_output_voltage(DACchannel, constrain(realPedalADCval >> 4, DACmotorIdle, DACmotorMax)); //convert 12bit ADC value into 8bit DAC value directly  (ignoring DACinvert option!)
        motorControlTimer = millis(); //stall the motor control loop (because the pedal determines power)
      #endif
    }
    // it might be nice to add some code which returns the targetSpeed/DACval to exactly 0 after the pedal is no longer pressed.
    // see 'noInstructStop' code in communication.h for inspiration
  #endif

  if((millis() - motorControlTimer) > motorControlInterval) { //not rollover proof, but only causes issues after like 49.7 days
    motorControlTimer = millis();
    updateSpeeds(); //update currentForwSpeed (among other things) (do this regardless of targetspeed)
    uint8_t newDACval = DACmotorIdle; //does not need to initialized to DACmotorIdle per-se
    if(targetSpeed > minTargetSpeed) {
      float output = DACmotorIdle + (targetSpeed * motorKpBase); //base multiplier
      //output += ((targetSpeed - currentForwSpeed) * motorKpAdj); //speed error multiplier    TBD!!!
      //output += motorKpSteer(steering::currentPos) * targetSpeed; //steering adds resistance, this counteracts that    TBD!!!
      //output += motorKpVolt() * targetSpeed; //lower battery voltage -> less power, this counteracts that     TBD!!!
      newDACval = constrain(output, DACmotorIdle, DACmotorMax);
      #ifdef brakePin
        digitalWrite(brakePin, !brakeActiveState); //turn off brakes
      #endif
    } else if(targetSpeed < -0.01) {
      //attempt motor braking here maybe?
      newDACval = DACmotorIdle;  //driving in reverse is TBD!!!
      #ifdef brakePin
        digitalWrite(brakePin, brakeActiveState);
      #endif
    } else {
      //attempt motor braking here???
      newDACval = DACmotorIdle;
      #ifdef brakePin
        digitalWrite(brakePin, brakeActiveState);
      #endif
    }
    DACwrite(newDACval);
//    #ifdef THR_printExtraDebug
//      Serial.print("DACwrite: "); Serial.println(newDACval);
//    #endif
  }

  return(true); //all went well, return true
}

}// namespace throttle
