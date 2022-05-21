/*
TBD:
  give motorCalibData fancy color
  make EEPROMmotorCalib work (EEPROM load/save and CRC func)
  make (preEnds) anti-suicide code (make sure motor doesnt kill pot or smash against output limits)
  calibration function (find motor calib data and preEnds)

i'd really like to treat the motor as an actuator with constant force, and the thing it's turning like a frictionless object with mass (only momentum)
 that way, you can just (calibrate! and) calculate how long the motor can accelerate + decelerate at 100% power (both ways) to acvhieve the desired distance (area under curve).
 This approach is reminiscent of spacecraft trajectories, including the suicide-burn parts. 
 The fastest way to traverse (frictioness) space is to accelerate for the first half of the distance and to decelerate for the second half. 
 The intergral of this constant-acceleration-based velocity curve is 0 at the end, but it does require perfectly simulated variables. If you start accelerating too late you overshoot (suicide burn)
 if you brake too early you are slow. Luckaly we can afford to be a little slow, so avoiding overshoot (and therefore potential oscillations) should be easy(ish).
 With the proper calibration, i would like to determine the power/torque (idk which term is more accurate here) curve of the motor,
 and after the servo is mounted a second calibration loop should determine the resistance (curve?) of the target (steering mechanism).
 in theory, once the system is properly calibrated, it should be capable of suicide-burn'ing untill the end of the movement range.
 In reality however, i would still like to have an extra little multiplier to make sure that the motor never hits the end of the range with any real force.
 If the belt (connecting the servo to the steering wheel) were removed, the servo should NOT overshoot and destroy the potentiometer (remember, NO REAL ENDSTOPS)
 Maybe a simple prediction algorithm based on measured acceleration (not expected accel), current pos and endstop pos could predict the trajectory enough to avoid pot-collisions 
 
*/


#define abs(x) ((x)>0?(x):-(x)) //makes abs() work with floats

#define debugSerial Serial

#include "thijsFIFO.h"

////////////////////// calibration data //////////////////////
//#define EEPROMmotorCalib  //store motor calibration in EEPROM and load on every boot    TBD!!

struct motorCalibData  //: public Printable     //struct to hold motor power calibration data. Can be pulled from- and put into EEPROM directly
{
  float basePower;
  float powerStep;
//  #ifdef PRINTABLE_CLASSES
//    size_t printTo(Print& p) const{
//      return(p.print("posVect("+String(_data[0])+','+String(_data[1])+")"));
//    }
//  #endif
};

const motorCalibData defualtMotorCalib = {0.075, (1.0/255.0)};

//const bool invertMotor = false; //not needed, just switch around 2 wires
const int16_t potMid = 512;
//const int16_t potEnds[2] = {0, 1023}; //for weirdly assymetrical setups
const int16_t defaultPreEnds[2] = {100, 923}; //for systems where the ends of the pot arent reached (strongly recommended as pot endstops are WEAK!)
const uint32_t potMeasureInterval = 10; //TBD: setup timerX (1 or 2) to generate an interrupt at a constant interval (make sure not to attack millis())
// by using a timer (overflow?) interrupt, the measurements become more constent, but the code becomes a little more complex
// proper interrupt flag management should be used to determine when to recalculate motor power (BTW, this can also be used to determine 'CPU usage' sortof)

////////////////////// pin definitions //////////////////////
const uint8_t potPin = A0;
//#define threeWireHbridge //seems to perform worse (less power, more noise) across the board, not sure why yet.
//#define constPowerMode  //not the best idea (power hungry + runs hot for minor perf gain (maybe))             TBD!!
#define motENpin 3         //you could do a second (higher freq) PWM on this one to limit the power, but IDK if it'll work well (since threeWireHbridge sucks)
const uint8_t motApin = 5; //timer0 PWM pins have a const freq becuase timer0 is also used for millis() timekeeping
const uint8_t motBpin = 6; //you can still halve the frequency by setting PWM mode (with the TCCR0A register, page 84 of datasheet)


///////////////////////// variables //////////////////////////
float desiredPos = 0.0; //the desired (instructed) position of the servo)
float currentPos = 0.0; //the current position of the servo (may be replaced by posHistFIFO[0])
FIFO<float> posHistFIFO(10); //a First-In-First-Out array to store position history
float currentSpeed = 0.0; //based on a (filtered/dampened) calculation from the pos (current & hist)

motorCalibData currentMotorCalib;
int16_t preEnds[2];

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
        //digitalWrite(motENpin, HIGH);
        digitalWrite(motENpin, abs(tempVal) > 0); //this may be worse (it could allow idling instead of shorting both motor terminals to GND)
      #endif
    }
  #endif
#endif

#ifdef EEPROMmotorCalib
  uint16_t calcCRC(motorCalibData data) {
    uint16_t result = 0;
    //voidPtr, see what i did in serialEncoding.h lib
    for(uint8_t i=0; i<sizeof(motorCalibData); i++) {
      //find some existing CRC function or something, IDK
    }
    return(result)
  }
#endif

float calcSpeed() {
  
}

float posToMotPow(float currentPos=0.0, float desiredPos=0.0) {
  
}

//void calibrate(uint8_t N=1) {
//  if(N < 1) {
//    debugSerial.println("can't calibrate with less than 1 pass");
//  }
//  //move (gently) to one end (maybe increase power if movement is insufficient (complicated!))
//  //record the max, average (std dev?) and variability and print them (and then just use the average for the potEnds[])
//  //do that N times
//}


void setup() {
  debugSerial.begin(115200);
  debugSerial.println("boot");
  pinMode(motApin, OUTPUT);
  pinMode(motBpin, OUTPUT);
  pinMode(potPin, INPUT);
  #ifdef motENpin
    pinMode(motENpin, OUTPUT);
  #endif

  #ifdef EEPROMmotorCalib
    //load motor calibration data from EEPROM into currentMotorCalib (and get stored CRC value)
    if(!(calcCRC(currentMotorCalib) == storedCRC)) {
      currentMotorCalib = defualtMotorCalib;
    }
  #else
    currentMotorCalib = defualtMotorCalib;
  #endif
  preEnds[0] = defaultPreEnds[0];  preEnds[1] = defaultPreEnds[1];

//  applyMotPow(1.00);
//  delay(1000);
//  applyMotPow(-1.00);
//  delay(1000);
//  applyMotPow(0.00);
}

void loop() {
//  int16_t ADCval = analogRead(potPin);
//  //float currentPos = (ADCval >= potMid) ? ((constrain(ADCval, potMid, potEnds[1])-potMid)/(potEnds[1]-potMid)) : (potMid-(constrain(ADCval, potEnds[0], potMid))/(potEnds[1]-potMid));
//  float currentPos = (ADCval - potMid) / 512.0;
//  posHistFIFO.put(currentPos);

//  if(interruptFlag) {
//    interruptFlag = false;
//    
//  }
}
