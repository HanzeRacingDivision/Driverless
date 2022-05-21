/*
 *  FIFO.h is a 'library' for simple First In First Out arrays
 *  
 *  i had to put all the functions in here, instead of a .cpp file, because of template<> complications (and my lack of skill in Cpp).
 *  i know that header files are not technically supposed to have substance (just headers), but it works for now.
 *  
 *  you could easily do this manually (or even with a struct or something),
 *  but this has fancy [] support and should be reasonably fast (i think)
 * 
 *  Created by Thijs van Liempd on 27/06/2021.
 */

#ifndef FIFO
#define FIFO

#include "Arduino.h"

//#define FIFO_DEBUG_SERIAL Serial

template<class T> class FIFO {
  public: //all variables are public, but some are marked with a suggestive _ (underscore) to indicate unsafety
    T* _data; //array is intialized in constructor
    size_t size; //set in constructor
    volatile size_t _cursor=0; //the 'cursor' holds the OLDEST index (which is overwritten at the next insert)
    volatile bool filled = false;
    FIFO(T* existingArray, size_t arraySize) : size(arraySize) {
      _data = existingArray;
    }
    FIFO(size_t arraySize, const bool initZero=true) : size(arraySize) { //this does mean you can't make a FIFO of (const?) booleans
      _data = new T[arraySize];
      if(initZero) {
        memset(_data, 0, sizeof(T)*arraySize);
      }
    }
    FIFO(size_t arraySize, T initialValue) : size(arraySize) {
      _data = new T[arraySize];
      for(size_t i=0; i<arraySize; i++) {
        _data[i] = initialValue;
      }
    }
    ~FIFO() { delete []_data; }
    
    T &operator[](size_t index) {
      if(index < size) {
        size_t shiftedIndex = (index<_cursor) ? (_cursor-index-1) : (size-1-(index-_cursor));
        return(_data[shiftedIndex]);
      } else {
        #if defined(ARDUINO_ARCH_ESP32)
          log_e(,"FIFO out of bounds");
        #elif defined(FIFO_DEBUG_SERIAL)
          FIFO_DEBUG_SERIAL.println("FIFO out of bounds");
        #endif
        //just consider it rollover (doesnt make sense, but i need a way to deal with it and the compiler won't let me return NULL)
        size_t shiftedIndex = index % size;
        shiftedIndex = (shiftedIndex<_cursor) ? (_cursor-shiftedIndex-1) : (size-1-(shiftedIndex-_cursor));
        return(_data[shiftedIndex]);
      }
    }
//    T operator[](size_t index) const { //i dont think you need this
//      return(_data[index]);
//    }

    T put(T dataToInsert) {
      _data[_cursor] = dataToInsert;
      _cursor++;
      //_cursor = (_cursor<size) ? _cursor : 0; //rollover done one way
      if(_cursor >= size) { //rollover done this way allows for the 'filled' bool to be set
        _cursor = 0;
        filled = true;
      }
      return(dataToInsert);
    }
};

#endif