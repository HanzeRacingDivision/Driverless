/*

*/
#pragma once

// idk if a namespace is really needed for the communications tab. I guess if it gets more complex/big than i'll start using one
//namespace comm { //this is to make clear to other sketches where the variables come from (and now i can use the same variablename in different namespaces)

//#define comm_CAN  //TBD
//#define comm_UART_ASCII //TBD
//#define comm_UART_RAW //TBD
#define comm_UART_DEPRICATED  //the RC car's communication format, only intended for testing (and the open day)

#define noInstructStop //stop driving if no instruction has been sent for a while    TBD!!!

/*
pins for comm: 23, 19, 18, 5
*/

#ifdef comm_CAN
  #error("CAN is TBD")
  #include "driver/twai.h"
  #define comm_CAN_RXpin 19
  #define comm_CAN_TXpin 23
  //https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html
  //https://github.com/espressif/esp-idf/tree/master/examples/peripherals/twai/twai_network
#endif
#if defined(comm_UART_ASCII) || defined(comm_UART_RAW)
  #define comm_UART_baudRate 115200
  #define comm_UART_serial Serial2
  #define comm_UART_serial_RX 19
  #define comm_UART_serial_TX 23
  #ifdef comm_UART_ASCII
    String receivedString = ""; //not the most efficient, a constant char array should probably replace this
  #endif
  #ifdef comm_UART_RAW
    //#define syncByte '\n'
    #ifdef syncByte
      const uint8_t syncByteInterval = 0;  //how many (outgoing) messages to skip before adding another syncByte (0 == every message, 1 == every other, etc.)
      uint8_t syncByteCounter = 0;
      const uint8_t syncByteRepeat = 0; //how many times to repeat the syncByte (0 == only 1 syncByte, 1 == 2 syncBytes)
    #endif
  #endif
#endif
#ifdef comm_UART_DEPRICATED
  #define comm_UART_serial Serial
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

  uint32_t encoCountSim;
  uint32_t encoCountSimTimer;

  uint32_t realSpeedToEncoCount(float realSpeed) { // this is a terrible hack to convert a speed value to some sort of RC-car-encoder equivalent
    //realSpeed = (sin(millis() / 1000.0)+1.0) / 2.0; //overwrite for debug
    float increment = realSpeed * (constrain(millis()-encoCountSimTimer, 0, 200)/1000.0) * 113.2448633; // the RC cart moved roughly 8.8 millimeters forward per encoder step
    if(increment >= 1.0) { //prevent small increments from being discarded by not updating the timer
      encoCountSim += increment;
      encoCountSimTimer = millis(); //update timer
    }
    //Serial.print(realSpeed); Serial.print('\t'); Serial.println(encoCountSim);
    return(encoCountSim);
  }
  int16_t degreesToSteerVal(float degreeInput) { //convert from degrees to AS5600 ADCticks
    return(degreeInput * ((-3600) / 25));  //based on absolutely nothing, just for basic testing purposes (much like this whole comm format)
  }
  float steerValToDegrees(int16_t steerValInput) { //convert from AS5600 ADCticks to degrees
    return((float)steerValInput * (25.0 / (-3600.0))); //avoid integer overflow!
  }
  void sendSerialFeedback(uint32_t encoVal, int16_t servoAngle, uint32_t timestamp) {
    comm_UART_serial.write(serialFeedbackStartSyncBytes, 2);
    //i think serial writing has some overhead, which may be lessened by writing from a buffer, instead of 1-byte-at-a-time
    uint8_t writeBuf[sizeof(encoVal)+sizeof(servoAngle)+sizeof(timestamp)]; // math should be precompiled
    writeBuf[0] = encoVal;     writeBuf[1] = encoVal>>8;    writeBuf[2] = encoVal>>16;    writeBuf[3] = encoVal>>24;
    writeBuf[4] = servoAngle;  writeBuf[5] = servoAngle>>8;
    writeBuf[6] = timestamp;   writeBuf[7] = timestamp>>8;  writeBuf[8] = timestamp>>16;  writeBuf[9] = timestamp>>24;
    comm_UART_serial.write(writeBuf, sizeof(encoVal)+sizeof(servoAngle)+sizeof(timestamp)); // math should be precompiled
    comm_UART_serial.write(serialFeedbackEndSyncBytes, 2);
  }
#endif

#ifdef noInstructStop
  uint32_t noInstructTimer;
  const uint32_t noInstructTimeout = 1000; //(millis) if no insctruction has been received for this long, stop moving
#endif


//#ifdef comm_UART_ASCII
//  struct commOutPacket : public Printable
//#else
//  struct commOutPacket
//#endif
//{
//  const static uint8_t packetSize = ;
//  commOutPacket() :  {}
//
//  template<typename T> uint8_t* _valToBuf(T val, uint8_t* target) {
//    uint8_t* bytePointer = static_cast<uint8_t*>(&T);//= (uint8_t*) &T; //cast to bytepointer
//  }
//  
//  uint8_t* encode(uint8_t* byteBuffer=NULL) {
//    if(!byteBuffer) { byteBuffer = (uint8_t*) malloc(packetSize); }
//    
//    //uint8_t* returnValBytePointer = (uint8_t*) &returnVal; //probably not the cleanest/fastest way, but i'm hoping the compiler will understand what i'm trying to do
//    //for(uint8_t i=0; i<packetSize; i++) { byteBuffer[i] = returnValBytePointer[i]; }
//    return(byteBuffer);
//  }
////  void decode(uint8_t* byteBuffer) {
////    centered = (rawData & 0x8000);
////    errorOne = (rawData & 0x4000);
////    errorTwo = (rawData & 0x2000);
////    currentPos = (rawData & 0x1000) ? (rawData | 0xF000) : rawData; //copy MSB of 13bit angle to play nice with signed integers.
////  }
//  
//  #ifdef comm_UART_ASCII
//    size_t printTo(Print& p) const{
//      return(p.print(String(currentPos)+','+String(centered)+','+String(errorOne)+','+String(errorTwo)));
//    }
//  #endif
//};

void commInit() {
  #ifdef comm_CAN
    //use builtin CAN hardware
    #error("CAN TBD")
  #endif
  #if defined(comm_UART_ASCII) || defined(comm_UART_RAW)
    #if defined(comm_UART_serial_RX) && defined(comm_UART_serial_TX)
      comm_UART_serial.begin(comm_UART_baudRate, SERIAL_8N1, comm_UART_serial_RX, comm_UART_serial_TX);
    #else
      comm_UART_serial.begin(comm_UART_baudRate);
    #endif
  #endif
  #ifdef comm_UART_DEPRICATED
    Serial.begin(115200); //uses serial over USB
  #endif
}

void commLoop() {
  #ifdef comm_UART_DEPRICATED
    //serial input
    if(comm_UART_serial.available()) {
      bool longEnoughInput = true;
      if(comm_UART_serial.available() < minSerialInputLength) { //either not all the bytes have made it into the serial buffer yet, or something went wrong
        //you could skip this one loop(), and try again the next time, but the current code is not in that much of a hurry, so just sit and wait (doing nothing)
        uint32_t serialInMinLenStart = micros(); //normally i'd add timeout value to this value, and the use "while(micros() < this)", but that's not rollover compatible
        bool keepWaiting = true;
        while(((micros()-serialInMinLenStart) < serialInMinLenTimeout) && keepWaiting) { //the ESP is fast enough that this (rollover safe!) math takes basically no time
          keepWaiting = (comm_UART_serial.available() < minSerialInputLength); //if bytes do hit the minimum, it will stop the while-loop
        }
        //if((micros()-serialInMinLenStart) >= serialInMinLenTimeout) { //this method saves a few microseconds, but wrongfully discards messages where the last byte came in at the very last microsecond
        if(comm_UART_serial.available() < minSerialInputLength) { //if at this point, there still isnt a full message in the buffer, just chuck it all out
          //Serial.println("minLength timeout (flushing)");
          while(comm_UART_serial.available()) { //everything in buffer
            comm_UART_serial.read(); //just read out, dont store it anywhere
          } //a flush command would be faster, but arduino serial flush functions were spotty in the (distant?) past, this is safer
          //return;  //return the whole loop()
          longEnoughInput = false; //using return(); could delay the motor control loop in a super worst-case-scenario (floating-pin serial input noise)
        }
      }
      if(longEnoughInput) { //if the message length is >= minSerialInputLength
        //read message
        String serialInWholeString = "";
        uint32_t serialInReadStart = micros(); //normally i'd add timeout value to this value, and the use "while(micros() < this)", but that's not rollover compatible
        while((comm_UART_serial.available()) && ((micros()-serialInReadStart) < serialInReadTimeout)) {
          serialInWholeString += (char)comm_UART_serial.read();
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
//          throttle::debugPrintTimer = millis() + throttle::debugPrintTimout;  // speed calibration / debug
          throttle::targetSpeed = serialInputSpeed;
          constrain(throttle::targetSpeed, throttle::targetSpeedLimits[0], throttle::targetSpeedLimits[1]);
          #ifdef noInstructStop
            noInstructTimer = millis() + noInstructTimeout;
          #endif
        } //else { Serial.println("bad message, couldn't parse speed"); }
        serialInputSeperatorLocationLast = serialInputSeperatorLocation;
        serialInputSeperatorLocation = serialInWholeString.indexOf(serialInputSeperator, serialInputSeperatorLocationLast+1); //get index of next seperator
        if(serialInputSeperatorLocation < 0) { serialInputSeperatorLocation = serialInWholeString.length(); } //if there are no more seperators, then read whole remainder
        String angleString = serialInWholeString.substring(serialInputSeperatorLocationLast, serialInputSeperatorLocation); //get substring up to (not including) seperator
        float serialInputAngle = angleString.toFloat();
        if((serialInputAngle > serialInputAngleMin) && (serialInputAngle < serialInputAngleMax) && (steering::centered)) {
          steering::desiredPos = degreesToSteerVal(serialInputAngle);
          constrain(steering::desiredPos, steering::calibratedEnds[0], steering::calibratedEnds[1]);
        } //else { Serial.println("bad message, couldn't parse angle"); }
        //flush serial for ultra safety:
        while(comm_UART_serial.available()) {
          comm_UART_serial.read();
        }
      }
    #ifdef noInstructStop
      } else if(millis()>noInstructTimer) {
        //Serial.println("no instruction received for too long, stopping...");
        noInstructTimer = 4294967295; //stop timer
        throttle::targetSpeed = 0.0;
      #endif
    }
    
    //sensor feedback (over serial)
    if(millis() >= serialFeedbackTimer) {
      serialFeedbackTimer = millis() + serialFeedbackInterval;
      float steeringDegrees = steerValToDegrees(steering::currentPos); // replace currentPos with desiredPos for debugging
      int16_t intifiedSteeringDegrees = constrain(steeringDegrees, -30.0, 30.0) * angleIntifyMult;
      Serial.print(throttle::targetSpeed); Serial.print('\t'); Serial.print(steerValToDegrees(steering::currentPos)); Serial.print('\t'); Serial.println(millis());
      //sendSerialFeedback(realSpeedToEncoCount(throttle::targetSpeed), intifiedSteeringDegrees, millis());  // replaced currentForwSpeed with targetSpeed for debugging
    }
  #endif
  
//  commOutPacket outData = commOutPacket(currentPos, centered);
//  #ifdef comm_CAN
//    #error("CAN is TBD")
//    desiredPos = ; //CAN read
//    
//    //CAN send(outData.encode());
//  #endif
//  
//  #ifdef comm_UART_ASCII
//    //String receivedString = "";
//    if(comm_UART_serial.available()) {
//      while(comm_UART_serial.available()) {
//        receivedString += (char) comm_UART_serial.read();
//        //if(receivedString.length() > 100) { //prevent String overflow if more than 255 bytes of new data are available
//        //  receivedString.substring(receivedString.length() - 12);
//        //}
//      }
//      bool useString = false; //whether to process the string at all (in case of partial data)
//      String tempString = "";
//      if(!receivedString.endsWith("\n")) { //if the newest value is still incomplete
//        int16_t newLineIndex = receivedString.lastIndexOf('\n'); //if it contains a newLine somewhere in there
//        tempString = receivedString.substring(newLineIndex+1); //take everything after the newLine
//        receivedString = receivedString.substring(0, constrain(newLineIndex, 0, 100)); //remove what was just put in tempString (and the '\n')
//      } else { //if the message has a '\n' at the end
//        receivedString.remove(receivedString.length()-1); //remove '\n' from the end
//        useString = true;
//      }
//      int16_t newLineIndex = -1;
//      if((newLineIndex = receivedString.lastIndexOf('\n')) >= 0) { //if there is another (older) message still in there
//        receivedString = receivedString.substring(newLineIndex+1);
//        useString = true;
//      }
//      if(useString) {
//        int32_t parsedInt = receivedString.toInt();
//        if(String(parsedInt) == receivedString) {
//          desiredPos = parsedInt;
//        } else {
//          Serial.print("WARNING, parsedInt != String(receivedString) ??? ");
//          Serial.print(receivedString); Serial.print(" = "); Serial.print(parsedInt); Serial.print(" = "); Serial.println(String(parsedInt));
//        }
//      }
//      receivedString = tempString;
//    }
//    
//    comm_UART_serial.println(outData); //the commOutPacket class is 'printable'
//  #endif
//  
//  #ifdef comm_UART_RAW
//    int8_t receivedByte = comm_UART_serial.read();
//    if(centered) {
//      desiredPos = map(receivedByte, -128, 127, calibratedEnds[0], calibratedEnds[1]); //slow?
//    } else {
//      #ifdef quickCenterKnownRange
//        desiredPos = map(receivedByte, -128, 127, quickCenterKnownRange/-2, quickCenterKnownRange/2); //slow?
//      #else
//        Serial.println("WARNING, not centered, can't set desiredPos");
//      #endif
//    }
//
//    //byte outByteArray[2];
//    //outData.encodeIntoBufferPointer(outByteArray);
//    //comm_UART_serial.write(outByteArray, 2);
//    comm_UART_serial.write((byte*) outData.encode(), 2);
//    #ifdef syncByte
//      #warning("syncByte untested!")
//      syncByteCounter++;
//      if(syncByteCounter > syncByteInterval) {
//        syncByteCounter = 0;
//        for(uint8_t i=0; i <= syncByteRepeat; i++) {
//          comm_UART_serial.write(syncByte);
//        }
//      }
//    #endif
//  #endif
  
}
