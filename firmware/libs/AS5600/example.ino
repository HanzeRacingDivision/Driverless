/* here are the entire contents of the 'libary'
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
          ESP_LOGE(,"FIFO out of bounds");
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
*/

#define FIFO_DEBUG_SERIAL Serial


#include "FIFO.h"


//// there are 3 ways to initialize:

FIFO<uint16_t> testFIFO(5); // contructor arguments are array size and whether or not to initialize values to 0 (via memset)
//FIFO<uint16_t> testFIFO(5, false); // default is 'true', which will set all bytes to 0

//FIFO<uint16_t> testFIFO(5, 64000); // contructor arguments are array size and initial value

//uint16_t existingArray[5] = {1,2,3,4,5}; // an existing array (note: the _cursor scrolls forwards, so _data contents are read backwards)
//FIFO<uint16_t> testFIFO((uint16_t*) &existingArray, 5); // contructor arguments are an existing array and its size

// note: you may get into trouble if you want to make FIFOs of bytes or booleans, as those can make 'ambigous/overloaded' arguments
//       it may help to specify clearly like "FIFO<uint8_t> testFIFO(5, (bool) 1);" instead of "FIFO<uint8_t> testFIFO(5, 1);

//// of course, you can also go multidimensional by making an array of FIFOs:
//FIFO<float> testFIFOarray[2] = {{5, true}, {7, true}}; //initialization of each individual FIFO is slightly tricky
//#define testFIFO testFIFOarray[0]     //this is just to make the rest of the example code behave the same, you can safely ignore this

//// you can also pass structs if you want (note: the rest of the example code is not prepared to work with this)
//struct exampStruct {
//  uint8_t singleByte = 123;
//  float vector[2] = {4.56, 7.89};
//};
//FIFO<exampStruct> testFIFO(5, false);

//// you could even go so far as to do:
//struct exampStruct : public Printable {
//  uint16_t _data[3] = {100, 200, 300};
//  inline uint16_t &operator[](size_t index) { //make struct [] able (just like FIFO)
//    return(_data[index]);
//  }
//  public:
//  size_t printTo(Print& p) const{ //this is all just extra floof, but it's nice to know it's possible, right :)
//    return(p.print("exampStruct{"+String(_data[0])+','+String(_data[1])+','+String(_data[2])+"}"));
//  }
//};
//FIFO<exampStruct> testFIFO(5, false);

void setup() {
  Serial.begin(115200);
  Serial.print("size: "); Serial.println(testFIFO.size);
  Serial.println();
  Serial.println("initial values:");
  for(size_t i=0; i<testFIFO.size; i++) {
    Serial.println(testFIFO[i]);
  }
  
  Serial.println();
  Serial.println("putting some things in:");
  for(size_t i=0; i<(testFIFO.size-1); i++) {
    Serial.println(testFIFO.put(i*10));
  }
  Serial.print("filled:"); Serial.println(testFIFO.filled); //should not be filled yet
  Serial.println(testFIFO.put(69));
  Serial.print("filled:"); Serial.println(testFIFO.filled); //should be just filled
  Serial.println(testFIFO.put(420));
  Serial.print("filled:"); Serial.println(testFIFO.filled); //overwrote oldest value
  
  Serial.println();
  Serial.println("current values:");
  for(size_t i=0; i<testFIFO.size; i++) {
    Serial.println(testFIFO[i]);
  }

  Serial.println();
  Serial.println("out of bounds example:");
  Serial.println(testFIFO[testFIFO.size]); //out of bounds example
  delay(10); //the esp32 needs time to print (log_e) error messages, but the program just keeps going, so wait for that to finish

  #ifdef testFIFO     //a little hack to make this only show up in the multidimensional example, this is not good code
    Serial.println();
    Serial.println("multidimensional example");
    for(size_t j=0; j<2; j++) {
      Serial.print("FIFO: "); Serial.println(j);
      for(size_t i=0; i<testFIFOarray[j].size; i++) {
        Serial.println(testFIFOarray[j][i]);
      }
    }
  #endif
}

void loop() {
  
}