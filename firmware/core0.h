/*
core0 does all of the high-speed stuff: Steering motor coltrol and throttle control
*/
#pragma once

#include "logging.h"
#include "steering.h"
#include "throttle.h"
#if defined(useOptFlow) && (useOptFlow == 0)
  #include "opticalFlow.h"
#endif

//#define targetLoopTime 500 //(micros) 500 is a reasonable minimum, 1000 would work just fine i think. (not directly used in speed calculation)

void core0task(void* nothing) {
  disableCore0WDT(); //screw that watchdog timer

  throttle::throttleInit();

  steering::steeringInit(); //includes centerCalib(), but that could be moved to here maybe

  while(1) {
    //uint32_t loopStart = micros();

    steering::steeringLoop();
    
    throttle::throttleLoop();
  
//    Serial.print(currentPos); Serial.print('\t');
//    Serial.print(desiredPos); Serial.print('\t');
//    Serial.print(motPow); Serial.print('\t');
//    Serial.println(currentSpeed);
  
//    #if(targetLoopTime > 0) //replaced by a timer variable, becuase using any delays in your code is for n00bs
//      uint32_t loopTime = (micros() - loopStart);
//      if(loopTime < targetLoopTime) {
//        delayMicroseconds(targetLoopTime - loopTime);
//      } else {
//        #ifdef timerDebugExtra
//          Serial.print("didn't make loopTime target:"); Serial.println(loopTime);
//        #endif
//      }
//    #endif
  }
}



//    #ifdef timerDebug
//      timerItt++;
//      if(timerItt >= timerCount) {
//        timerItt = 0;
//        uint32_t sums[timeStampCount-1];  for(uint8_t j=0; j<(timeStampCount-1); j++) { sums[j] = 0; }
//        uint32_t totalUsedTime = 0;
//        for(uint8_t i=0; i<timerCount; i++) {
//          uint32_t usedTime = 0;
//          for(uint8_t j=0; j<(timeStampCount-1); j++) {
//            uint32_t dt = (timers[i][j+1] - timers[i][j]);
//            #ifdef timerDebugExtra
//              Serial.print(dt); Serial.print(' ');
//            #endif
//            sums[j] += dt;
//            usedTime += dt;
//          }
//          #ifdef timerDebugExtra
//            Serial.print("=> "); Serial.println(usedTime);
//          #endif
//          totalUsedTime += usedTime;
//        }
//    
//        Serial.print("avg: ");
//        for(uint8_t j=0; j<(timeStampCount-1); j++) {
//          sums[j] /= timerCount;
//          Serial.print(sums[j]); Serial.print(' ');
//        }
//        Serial.print("=> "); Serial.println(totalUsedTime/timerCount);
//        
//        Serial.print("looptime: "); Serial.println((timers[timerCount-1][0] - timers[0][0])/(timerCount-1));
//      }
//    #endif
