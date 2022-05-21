/*
i made several options for communication protocols. You can switch between them by using the defines.
options are:
-  CAN bus (1991 CAN (2.0), not CAN-FD. The ESP32 calls it TWAI these days)
-  UART ASCII (legible, slow)
-  UART raw (fast, reliable, illegible)
-  PWM (servo PWM input and output)
-  All of the above, selectable using jumpers at arduino startup/reset

each input frame only needs to be 1 byte (especially in PWM mode, but not in UART ASCII mode): the desired angle
  (if you really want, you could use a second byte to transmit commands, like asking for centerCalib. For now i'll just do that manually)
  in UART ASCII mode, the entered value is the desiredPos. Makes for easy debugging but hard actual functioning. Also, '\n' is used to split inputs
each output frame contains 2 bytes (except for PWM):
  of the 16 bits, 13 bits for the angle (for humans that'll be 12 integer bits and 1 sign bit)
  the remaining bits are for debugging info:
    whether the servo is centered (or if calibration went wrong)
    TBD
    TBD
  in PWM mode the output data is only the angle (in the form of servo PWM)

equally possible communication protocols include:
  both arduino and ESP32:
    - ADC (arduino only input, as it has no DAC)
    - I2C (slave, i'm like 90% sure the arduino can do this one as well)
    - SPI (slave (arduino is a maybe))
  ESP32 only:
    - I2S ?
    - bluetooth
    - wifi
    - infra-red remote control
Also, the ESP32 has duplicates of most peripheral interfaces (I2C) and 2 cores, it could easily run 2 (or 4) motors on 1 device if you wanted
*/

//#define comm_CAN
//#define comm_UART_ASCII
//#define comm_UART_RAW
//#define comm_PWM
//#define comm_ALL_JUMPERS


#ifdef comm_ALL_JUMPERS
  #undef comm_CAN
  #undef comm_UART_ASCII
  #undef comm_UART_RAW
  #undef comm_PWM
#endif

#if defined(comm_CAN) || defined(comm_ALL_JUMPERS)
  #error("CAN is TBD")
  #ifdef ARDUINO_ARCH_ESP32
    #include "driver/twai.h"
    #define comm_CAN_TXpin 
    #define comm_CAN_RXpin 
    //https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html
    //https://github.com/espressif/esp-idf/tree/master/examples/peripherals/twai/twai_network
  #else
    //using MCP2515 SPI CAN bus module
    //https://www.electronicshub.org/arduino-mcp2515-can-bus-tutorial/
  #endif
#endif
#if defined(comm_UART_ASCII) || defined(comm_UART_RAW) || defined(comm_ALL_JUMPERS)
  #define comm_UART_baudRate 115200
  #ifdef ARDUINO_ARCH_ESP32
    #define comm_UART_serial Serial2
    #define comm_UART_serial_RX 16
    #define comm_UART_serial_TX 17
  #else
    #define comm_UART_serial Serial
  #endif
  #if defined(comm_UART_ASCII) || defined(comm_ALL_JUMPERS)
    String receivedString = "";
  #endif
  #if defined(comm_UART_RAW) || defined(comm_ALL_JUMPERS)
    //#define syncByte '\n'
    #ifdef syncByte
      const uint8_t syncByteInterval = 0;  //how many (outgoing) messages to skip before adding another syncByte (0 == every message, 1 == every other, etc.)
      uint8_t syncByteCounter = 0;
      const uint8_t syncByteRepeat = 0; //how many times to repeat the syncByte (0 == only 1 syncByte, 1 == 2 syncBytes)
    #endif
  #endif
#endif
#if defined(comm_PWM) || defined(comm_ALL_JUMPERS)
  uint32_t inPulseStartTime; //micros() at the rising edge of the PWM signal 
  volatile uint32_t inPulseLength;
  volatile bool inPulseNewData = false; //mostly there to play nicely when multiple communication protocols are enabled at once
  //#define comm_PWM_OUT_fullRange     //full range instead of servo range pulse length
  //#define comm_PWM_OUT_lowLevelPWM
  //#define comm_PWM_IN_lowLevelInt
  //#define comm_PWM_alternatingInt
  
  #ifdef ARDUINO_ARCH_ESP32
    #define comm_PWM_INpin 5 //can be almost any pin(?)
    #define comm_PWM_INintPin  comm_PWM_INpin //same thing on ESP32 (makes more sense TBH)
    #define comm_PWM_OUTpin 21 //can be any PWM pin, but using pins 5 or 6 means you cant change the frequency (due to millis() interrupt on that timer)
    // you can set any frequency you like, just remember to calculate 1000us and 2000us
    #define comm_PWM_OUT_channel 3
    #define comm_PWM_OUT_res 11   //max 16 bits
    #define comm_PWM_OUT_freq (1000000UL / (0xFFFF >> (16-comm_PWM_OUT_res)))   //calculate frequency sothat PWMval equals microseconds.
    //at 11bit resolution and (1000000 / 2047) = 488Hz the period is 2047us long, so PWMval == microseconds pulse length
    
    #ifdef comm_PWM_OUT_lowLevelPWM
      #error("comm_PWM_OUT_lowLevelPWM for ESP32 is TBD")
      //see esp32-hal-ledc.c in the core in Arduino15 in %appdata%
    #endif
    #ifdef comm_PWM_IN_lowLevelInt
      #error("comm_PWM_IN_lowLevelInt for ESP32 is TBD")
      
    #elif defined(comm_PWM_alternatingInt)
      void ARDUINO_ISR_ATTR comm_PWM_risingInt() {
        attachInterrupt(comm_PWM_INintPin, comm_PWM_fallingInt, FALLING);
        inPulseStartTime = micros();
      }
      void ARDUINO_ISR_ATTR comm_PWM_fallingInt() {
        attachInterrupt(comm_PWM_INintPin, comm_PWM_risingInt, RISING);
        inPulseLength = micros() - inPulseStartTime;
        inPulseNewData = true;
      }
    #else
      void ARDUINO_ISR_ATTR comm_PWM_changeInt() {
        if(digitalRead(comm_PWM_INpin)) {
          inPulseStartTime = micros();
        } else {
          inPulseLength = micros() - inPulseStartTime;
          inPulseNewData = true;
        }
      }
    #endif
  #else //328p arduino PWM
    #define comm_PWM_INpin 2 //must be 2 or 3, becuase those are interrupt pins (0 and 1 respectively)
    #define comm_PWM_INintPin 0 //digitalPinToInterrupt() or something
    #define comm_PWM_OUTpin 11 //can be any PWM pin, but using pins 5 or 6 means you cant change the frequency (due to millis() interrupt on that timer)
    // timer1 and 2 are in phase-correct PWM mode by default with a 64x clock prescaler
    // the PWM frequency is F_CPU/(prescaler*510)  so the total pulseLength (micros) is (prescaler*510)/16 = (64*510)/16 = 2040
    // that gives you PWM values for 1000us and 2000us, 125 and 250 respectively.
    
    #ifdef comm_PWM_OUT_lowLevelPWM
      #error("comm_PWM_OUT_lowLevelPWM is TBD")
      #define comm_PWM_OUTpin_init   TBD   //set the frequency (and output-method) using the right PWM control registers
      #define comm_PWM_OUTpin_OCRnx  TBD   //the OutputCompareRegister that goes with the pin
    #endif
    #ifdef comm_PWM_IN_lowLevelInt
      #error("comm_PWM_IN_lowLevelInt is TBD")
      #define comm_PWM_INpin_init      EICRA |= 0b00000011;  EIMSK = 1; //set INT0 to rising edge mode and enable INT0 (disable INT1)
      #define comm_PWM_INpin_register  TBD
      #define comm_PWM_INpin_bitMask   TBD
      ISR(INT0_vect) {   //fastest way to do it.
        if(comm_PWM_INpin_register & comm_PWM_INpin_bitMask) { //rising
          inPulseStartTime = micros();
        } else { //falling
          inPulseLength = micros() - inPulseStartTime;
          inPulseNewData = true;
        }
      }
    #elif defined(comm_PWM_alternatingInt)
      void comm_PWM_risingInt() {
        attachInterrupt(comm_PWM_INintPin, comm_PWM_fallingInt, FALLING);
        inPulseStartTime = micros();
      }
      void comm_PWM_fallingInt() {
        attachInterrupt(comm_PWM_INintPin, comm_PWM_risingInt, RISING);
        inPulseLength = micros() - inPulseStartTime;
        inPulseNewData = true;
      }
    #else
      void comm_PWM_changeInt() {
        if(digitalRead(comm_PWM_INpin)) {
          inPulseStartTime = micros();
        } else {
          inPulseLength = micros() - inPulseStartTime;
          inPulseNewData = true;
        }
      }
    #endif
  #endif //328p arduino PWM
  
  
#endif

#ifdef comm_UART_ASCII
  struct commOutPacket : public Printable
#else
  struct commOutPacket
#endif
{
  int16_t currentPos; //13 bit
  bool centered;
  bool errorOne;
  bool errorTwo;
  
  commOutPacket(int16_t currentPos, bool centered, bool errorOne=false, bool errorTwo=false) : currentPos(currentPos), centered(centered), errorOne(errorOne), errorTwo(errorTwo) {}
  
  uint16_t encode() {
    //first 3 bits are debug info, remaining 13 are angle data
    currentPos = constrain(currentPos, -4096, 4095);
    uint16_t returnVal = currentPos & 0x1FFF; //13 bits of angle data
    returnVal = (centered ? (returnVal | 0x8000) : returnVal); //MSB
    returnVal = (errorOne ? (returnVal | 0x4000) : returnVal);
    returnVal = (errorTwo ? (returnVal | 0x2000) : returnVal);
    return(returnVal);
  }
  void encodeIntoBufferPointer(uint8_t* bufferPointer) {
    uint16_t returnVal = encode();
    uint8_t* returnValBytePointer = (uint8_t*) &returnVal; //probably not the cleanest/fastest way, but i'm hoping the compiler will understand what i'm trying to do
    bufferPointer[0] = returnValBytePointer[0];
    bufferPointer[1] = returnValBytePointer[1];
  }
//  void decode(uint16_t rawData) {
//    centered = (rawData & 0x8000);
//    errorOne = (rawData & 0x4000);
//    errorTwo = (rawData & 0x2000);
//    currentPos = (rawData & 0x1000) ? (rawData | 0xF000) : rawData; //copy MSB of 13bit angle to play nice with signed integers.
//  }
  
  #ifdef comm_UART_ASCII
    size_t printTo(Print& p) const{
      return(p.print(String(currentPos)+','+String(centered)+','+String(errorOne)+','+String(errorTwo)));
    }
  #endif
};

void commInit() {
  #ifdef comm_ALL_JUMPERS
    //read jumper pins here
  #endif
  
  #if defined(comm_CAN) || defined(comm_ALL_JUMPERS)
    #ifdef ARDUINO_ARCH_ESP32
      //use builtin CAN hardware
      #error("CAN TBD")
    #else //328p arduino CAN
      //use external/manual CAN hardware
      #error("CAN TBD")
    #endif //328p arduino CAN
  #endif
  #if defined(comm_UART_ASCII) || defined(comm_UART_RAW) || defined(comm_ALL_JUMPERS)
    #ifdef ARDUINO_ARCH_ESP32
      #if defined(comm_UART_serial_RX) && defined(comm_UART_serial_TX)
        comm_UART_serial.begin(comm_UART_baudRate, SERIAL_8N1, comm_UART_serial_RX, comm_UART_serial_TX);
      #else
        comm_UART_serial.begin(comm_UART_baudRate);
      #endif
    #else
      comm_UART_serial.begin(comm_UART_baudRate);
    #endif
  #endif
  #if defined(comm_PWM) || defined(comm_ALL_JUMPERS)
    pinMode(comm_PWM_INpin, INPUT_PULLUP);
    pinMode(comm_PWM_OUTpin, OUTPUT);
    #ifdef ARDUINO_ARCH_ESP32
      #ifdef comm_PWM_OUT_lowLevelPWM
        #error("comm_PWM_OUT_lowLevelPWM for ESP32 is TBD")
      #else
        ledcSetup(comm_PWM_OUT_channel, comm_PWM_OUT_freq, comm_PWM_OUT_res);
        ledcAttachPin(comm_PWM_OUTpin, comm_PWM_OUT_channel);
      #endif
    #elif defined(comm_PWM_OUT_lowLevelPWM)  //328p arduino PWM and comm_PWM_OUT_lowLevelPWM
      #error("comm_PWM_OUT_lowLevelPWM is TBD")
      comm_PWM_OUTpin_init;
    #endif
    #ifdef comm_PWM_IN_lowLevelInt
      #ifdef ARDUINO_ARCH_ESP32
        #error("comm_PWM_IN_lowLevelInt is TBD")
      #else
        comm_PWM_INpin_init;
      #endif
    #elif defined(comm_PWM_alternatingInt)
      attachInterrupt(comm_PWM_INintPin, comm_PWM_risingInt, RISING);
    #else
      attachInterrupt(comm_PWM_INintPin, comm_PWM_changeInt, CHANGE);
    #endif
  #endif
}

void commLoop() {
  commOutPacket outData = commOutPacket(currentPos, centered);
  #ifdef comm_CAN
    #error("CAN is TBD")
    desiredPos = ; //CAN read
    
    //CAN send(outData.encode());
  #endif
  
  #ifdef comm_UART_ASCII
    //String receivedString = "";
    if(comm_UART_serial.available()) {
      while(comm_UART_serial.available()) {
        receivedString += (char) comm_UART_serial.read();
        //if(receivedString.length() > 100) { //prevent String overflow if more than 255 bytes of new data are available
        //  receivedString.substring(receivedString.length() - 12);
        //}
      }
      bool useString = false; //whether to process the string at all (in case of partial data)
      String tempString = "";
      if(!receivedString.endsWith("\n")) { //if the newest value is still incomplete
        int16_t newLineIndex = receivedString.lastIndexOf('\n'); //if it contains a newLine somewhere in there
        tempString = receivedString.substring(newLineIndex+1); //take everything after the newLine
        receivedString = receivedString.substring(0, constrain(newLineIndex, 0, 100)); //remove what was just put in tempString (and the '\n')
      } else { //if the message has a '\n' at the end
        receivedString.remove(receivedString.length()-1); //remove '\n' from the end
        useString = true;
      }
      int16_t newLineIndex = -1;
      if((newLineIndex = receivedString.lastIndexOf('\n')) >= 0) { //if there is another (older) message still in there
        receivedString = receivedString.substring(newLineIndex+1);
        useString = true;
      }
      if(useString) {
        int32_t parsedInt = receivedString.toInt();
        if(String(parsedInt) == receivedString) {
          desiredPos = parsedInt;
        } else {
          Serial.print("WARNING, parsedInt != String(receivedString) ??? ");
          Serial.print(receivedString); Serial.print(" = "); Serial.print(parsedInt); Serial.print(" = "); Serial.println(String(parsedInt));
        }
      }
      receivedString = tempString;
    }
    
    comm_UART_serial.println(outData); //the commOutPacket class is 'printable'
  #endif
  
  #ifdef comm_UART_RAW
    int8_t receivedByte = comm_UART_serial.read();
    if(centered) {
      desiredPos = map(receivedByte, -128, 127, calibratedEnds[0], calibratedEnds[1]); //slow?
    } else {
      #ifdef quickCenterKnownRange
        desiredPos = map(receivedByte, -128, 127, quickCenterKnownRange/-2, quickCenterKnownRange/2); //slow?
      #else
        Serial.println("WARNING, not centered, can't set desiredPos");
      #endif
    }

    //byte outByteArray[2];
    //outData.encodeIntoBufferPointer(outByteArray);
    //comm_UART_serial.write(outByteArray, 2);
    comm_UART_serial.write((byte*) outData.encode(), 2);
    #ifdef syncByte
      #warning("syncByte untested!")
      syncByteCounter++;
      if(syncByteCounter > syncByteInterval) {
        syncByteCounter = 0;
        for(uint8_t i=0; i <= syncByteRepeat; i++) {
          comm_UART_serial.write(syncByte);
        }
      }
    #endif
  #endif
  
  #ifdef comm_PWM
    if(inPulseNewData) {
      inPulseNewData = false;
      if(centered) {
        desiredPos = map(constrain(inPulseLength, 1000, 2000), 1000, 2000, calibratedEnds[0], calibratedEnds[1]); //slow?
      } else {
        #ifdef quickCenterKnownRange
          desiredPos = map(constrain(inPulseLength, 1000, 2000), 1000, 2000, quickCenterKnownRange/-2, quickCenterKnownRange/2); //slow?
        #else
          Serial.println("WARNING, not centered, can't set desiredPos");
        #endif
      }
    }

    #ifdef ARDUINO_ARCH_ESP32
      //the ESP32 has a lot of options in terms of ESP resolution and frequency
      #ifdef comm_PWM_OUT_fullRange
        //uint16_t outPWMval = 4096 + constrain(outData.currentPos, -4096, 4095); //13bit pulse length
        //uint8_t outPWMval = (4096 + constrain(outData.currentPos, -4096, 4095)) >> 5; //convert to 8bit pulse length
        #error("comm_PWM_OUT_fullRange for ESP32 is TBD (should be pretty easy though)")
      #else //output servo PWM range
        uint16_t outPWMval = map(constrain(outData.currentPos, -4096, 4095), -4096, 4095, 1000, 2000); //convert to between 1000 and 2000us
      #endif
      #ifdef comm_PWM_OUT_lowLevelPWM
        #error("comm_PWM_OUT_lowLevelPWM for ESP is TBD")
      #else
        ledcWrite(comm_PWM_OUT_channel, outPWMval);
      #endif
    #else //328p arduino PWM
      #ifdef comm_PWM_OUT_fullRange
        //uint16_t outPWMval = 4096 + constrain(outData.currentPos, -4096, 4095); //13bit pulse length
        uint8_t outPWMval = (4096 + constrain(outData.currentPos, -4096, 4095)) >> 5; //convert to 8bit pulse length
      #else //output servo PWM range
        uint8_t outPWMval = map(constrain(outData.currentPos, -4096, 4095), -4096, 4095, 125, 250); //convert to between 1000 and 2000us
      #endif
      #ifdef comm_PWM_OUT_lowLevelPWM
        //comm_PWM_OUTpin_OCRnx = outPWMval;
      #else
        analogWrite(comm_PWM_OUTpin, outPWMval);
      #endif
    #endif
  #endif
}
