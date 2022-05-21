/*
This is an example/explanatory sketch for the AS5600 I2C (twi) library i (Thijs van Liempd) made around 2021/2022.
I needed to read the AS5600 magnetic rotary encoder as fast as the bus would allow,
 so i made my own low-level code for the atmega328p (and later also for the ESP32)
In 2022 i added the loopReadWithCallback() function which lightly abuses the I2C bus by using a never-ending read packet.
The limitation of the sensor is the angle sampling rate, which this library seems to exceed (depending on filter config).
for insight into how/why i do things, please refer to the library code (only 1 header file becuase i'm too lazy to write a seperate .cpp file),
 and of course the datasheet of the sensor: https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf .

In this sketch i showcase most of the functions and how long they take to execute.
*/

#ifdef ARDUINO_ARCH_ESP32
  const uint8_t AS5600_SDApin = 18;
  const uint8_t AS5600_SCLpin = 19;
#endif


//#define AS5600_unlock_burning  //enable compilation of burn functions: burnAngle() and burnSetting()
//#define AS5600_return_esp_err  //makes certain functions return esp_err_t instead of bool, to allow for on-the-fly user debugging or whatever. AS5600debugPrint (if defined) should also print the errors

// some handy defines (i think you need to do these before you import the library, not 100% sure tho)
#define AS5600debugPrint(x)  Serial.println(x)    //you can undefine these printing functions no problem, they are only for printing I2C errors
//#define AS5600debugPrint(x)  log_d(x)

#include "AS5600.h"


AS5600 sensor; //on the atmega328p it's a nearly static class, i think i can make the code run faster (eliminate stack setup time like inline functions) by changing the class structure
// on the ESP32 the class should probably hold the I2C_port_num, and also it's fast enough sothat 'inlining' doesnt matter as much


const uint8_t exampleDataCount = 10;
uint32_t timers[exampleDataCount+1];
uint8_t timerItt = 0;

void printTimerValues(uint8_t spiltEvery = 0) {
  Serial.println("timers:");
  for(uint8_t i=1; i<timerItt; i++) {
    Serial.println(timers[i] - timers[i-1]);
    if(spiltEvery > 0) {
      if((i % spiltEvery) == 0) {
        Serial.println();
      }
    }
  }
  Serial.println(":timers\n");
}

uint16_t angles[exampleDataCount*4]; //record some sensor data just to prove it is indeed working as intended
uint16_t callbackItt = exampleDataCount*2; //in the callback function there is no implicit itterator

void setup() {
  Serial.begin(115200);  Serial.println();
  #ifdef ARDUINO_ARCH_ESP32
//    pinMode(AS5600_SDApin, INPUT_PULLUP); //not needed, as twoWireSetup() does the pullup stuff for you
//    pinMode(AS5600_SCLpin, INPUT_PULLUP);
    esp_err_t initErr = sensor.init(1000000, AS5600_SDApin, AS5600_SCLpin); //on the ESP32 (almost) any pins can be I2C pins
    if(initErr != ESP_OK) { Serial.print("I2C init fail. error:"); Serial.println(esp_err_to_name(initErr)); Serial.println("while(1){}..."); while(1) {} }
    //note: on the ESP32 the actual I2C frequency is lower than the set frequency (by like 20~40% depending on pullup resistors, 1.5kOhm gets you about 800kHz)
  #else //328p arduino I2C
    pinMode(SDA, INPUT_PULLUP); //A4    NOT SURE IF THIS INITIALIZATION IS BEST (external pullups are strongly recommended anyways)
    pinMode(SCL, INPUT_PULLUP); //A5
    sensor.init(800000); //anything beyond 800kHz doesnt seem to work properly
  #endif
  
  sensor.resetConfig(); //writes all 0s to the configuration registers

  //sensor.setSF(3); //set slow-filter to the mode with the least delay
  //sensor.setFTH(7); //set fast filter threshold to something... idk yet
  
  sensor.printConfig(); //shows you the contents of the configuration registers
  Serial.println();
  sensor.printStatus(); //shows you the status of the sensor (not whether it's connected, but whether the magnet is correctly positioned and stuff)
  Serial.println();
  
  //////////////traditional reading
  timers[timerItt] = micros();  timerItt++;
  for(uint8_t i=0; i<exampleDataCount; i++) {
    angles[i] = sensor.getAngle(); //retrieve angle the traditional way (by sending a full I2C packet including request for the angle register
    //note: getAngle() is really just a macro for requestReadInt(AS5600_RAW_ANGLE_H);
    //if you want the angle that is affected by the ZPOS/MPOS/MANG registers, you want getAltAngle() which is a macro for requestReadInt(AS5600_ANGLE_H)
    timers[timerItt] = micros();  timerItt++;
  }
  Serial.println("full request+reads times:"); printTimerValues();  timerItt = 0;

  timers[timerItt] = micros();  timerItt++;
  for(uint8_t i=exampleDataCount; i<(exampleDataCount*2); i++) {
    angles[i] = sensor.onlyReadInt(); //retrieve data without requesting a specific register to read from (works because of read cursor rollover explained on page 
    timers[timerItt] = micros();  timerItt++;
  }
  Serial.println("only reads times:"); printTimerValues();  timerItt = 0;

  #ifndef ARDUINO_ARCH_ESP32  //i haven't made the loopReadWithCallback functions for the ESP32 (yet???) becuase the ESP's I2C peripheral does not seem to like it.
  
    timers[timerItt] = micros();  timerItt++;
    sensor.loopReadWithCallback(callback, exampleDataCount); //returned false when an error occurs during reading
  //  if(!sensor.loopReadWithCallback(callback, exampleDataCount)) { //check for errors
  //    Serial.println("loopReadWithCallback returned FALSE!");
  //  }
    Serial.println("loopReadWithCallback finite times:"); printTimerValues();  timerItt = 0;
  
    timers[timerItt] = micros();  timerItt++;
    sensor.loopReadWithCallback(callback, -1); //will run forever, unless a communication exception occurs or it is stopped using 'sensor.keepLooping = false'
    Serial.println("loopReadWithCallback infinite times:"); printTimerValues();  timerItt = 0;
    
  #endif //ESP32 can't do this one
  
  Serial.println("\n angles:");
  for(uint16_t i=0; i<(exampleDataCount*4); i++) {
    Serial.println(angles[i]);
  }
}


void loop() {
}


void callback(uint16_t angle) { //with loopReadWithCallback you can call a function whenever new angle data is received (without ending the I2C packet)
  //Serial.println(angle);
  angles[callbackItt] = angle;    callbackItt++;
  timers[timerItt] = micros();  timerItt++;
  // to exit the loop early (or at all, if it's set to infinite), you can do:
  if(callbackItt >= (exampleDataCount*4)) {
    Serial.print("stopping loopReadWithCallback manually \t"); Serial.println(callbackItt);
    sensor.keepLooping = false;
  }
} //i haven't done extensive testing, but it seems like this function is allowed to take 100us without problems. At some point the sensor may complain and cut off the I2C packet.
// in general, i think this function should be as quick as possible (consider using 'inline'), for example, i put a motor_control_update function here.
