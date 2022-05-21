/*
i made several options for communication protocols. You can switch between them by using the defines.
options are:
-  CAN bus (1991 CAN (2.0), not CAN-FD. The ESP32 calls it TWAI these days)
-  UART ASCII (legible, slow)
-  UART raw (fast, reliable(?), illegible)
-  All of the above, selectable using jumpers at arduino startup/reset

- PWM (input only) for driving the kart from a standard RC transmitter

i'm strongly considering using the same serial format i used for the small scale RC car,
 because a python reader is already mostly done,
 EXCEPT that was only intended to handle 1 speed sensor, not ...#TBD... .
i'll probably format the UART ASCII protocol like that, still.

equally possible communication protocols include:
    - I2C
    - SPI
    - bluetooth
    - wifi
    - infra-red remote control
*/

//#define comm_CAN
//#define comm_UART_ASCII
//#define comm_UART_RAW
//#define comm_ALL_JUMPERS


#ifdef comm_ALL_JUMPERS
  #undef comm_CAN
  #undef comm_UART_ASCII
  #undef comm_UART_RAW
#endif

#if defined(comm_CAN) || defined(comm_ALL_JUMPERS)
  #error("CAN is TBD")
  #include "driver/twai.h"
  #define comm_CAN_TXpin 
  #define comm_CAN_RXpin 
  //https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html
  //https://github.com/espressif/esp-idf/tree/master/examples/peripherals/twai/twai_network
#endif
#if defined(comm_UART_ASCII) || defined(comm_UART_RAW) || defined(comm_ALL_JUMPERS)
  #define comm_UART_baudRate 115200
  #define comm_UART_serial Serial2
  #define comm_UART_serial_RX 16
  #define comm_UART_serial_TX 17
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
    //use builtin CAN hardware
    #error("CAN TBD")
  #endif
  #if defined(comm_UART_ASCII) || defined(comm_UART_RAW) || defined(comm_ALL_JUMPERS)
    #if defined(comm_UART_serial_RX) && defined(comm_UART_serial_TX)
      comm_UART_serial.begin(comm_UART_baudRate, SERIAL_8N1, comm_UART_serial_RX, comm_UART_serial_TX);
    #else
      comm_UART_serial.begin(comm_UART_baudRate);
    #endif
  #endif
}

//void commLoop() {
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
//}
