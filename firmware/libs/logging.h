
#pragma once

namespace logging { //this is to make clear to other sketches where the variables come from (and now i can use the same variablename in different namespaces)

const uint16_t staticLogBufSize = 1000;
static uint8_t staticLogBuf[staticLogBufSize];
static uint16_t staticLogBufFilled = 0; //records how much of the buffer is filled with useful data

//functions using arbetrary buffer
template<typename T> inline uint16_t logStuff(T* input, uint8_t* logBuf, uint16_t buffOffset=0) { //if it already IS a pointer, this function should automatically be used
  //uint8_t* tempBytePtr = (uint8_t*) input;
  //for(uint8_t i=0; i<sizeof(input); i++) { logBuf[i] = tempBytePtr[i + buffOffset]; } //manual copy
  memcpy(input, logBuf + buffOffset, sizeof(input));
  buffOffset += sizeof(input);
  return(buffOffset);
}
template<typename T> inline uint16_t logStuff(T input, uint8_t* logBuf, uint16_t buffOffset=0) {
  //uint8_t* tempBytePtr = (uint8_t*) &input;
  //for(uint8_t i=0; i<sizeof(input); i++) { logBuf[i] = tempBytePtr[i + buffOffset]; }
  memcpy(&input, logBuf + buffOffset, sizeof(input));
  buffOffset += sizeof(input);
  return(buffOffset);
}
//functions using default buffer
template<typename T> inline uint16_t logStuff(T* input) { //these are seperate functions to prevent someone from only using 1 defualt argument
  return(logStuff(input, staticLogBuf, staticLogBufFilled));
}
template<typename T> inline uint16_t logStuff(T input) {
  return(logStuff(input, staticLogBuf, staticLogBufFilled));
}

}
