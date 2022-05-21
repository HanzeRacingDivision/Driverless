/*
this one ESP32 will handle steering motor control, throttle control, 
  efficient communication with other devices and (maybe!), optical flow sensor reading.
  the high-speed (low accuracy) sensor data will be passed to the master PC (running selfdriving code),
  as well as to the LiDAR(s). This data can be used to generate interpolation steps between the SLAM position updates.

for the sake of speed (and because i can), the tasks will be divided over both CPU cores of the ESP,
core1 (default arduino core) will handle the low-speed stuff:
  communication
  (arduino crap)
  optical flow sensor reading? (i imagine SPI communication with this sensor takes too long for core0)
  wheel encoder interrupts? (should be removed in favor of HW pulse counter)
core0 will handle the high speed control loops:
  steering motor control
  throttle control

all code for a certain component (like steering) should in a seperate header file, 
  and everything in that header file should also be in a specific namespace.
  defines are not bound to namespaces though, so be a little watchful there

*/



#define abs(x) ((x)>0?(x):-(x)) //makes abs() work with floats. Alternatively, there is fabs(), but that returns a double, which old version of arduino IDE don't love

//#define useOptFlow  1
//#define encoderIntOnCore0  //should be depricated in favor of the ESP's HW pulse counters

#include "logging.h"
#include "core0.h"
#include "communication.h"
#if defined(useOptFlow) && (useOptFlow == 1)
  #include "opticalFlow.h"
#endif
#ifndef encoderIntOnCore0  //if NOT defined
 #include "throttle.h"   //encoder interrupts on core1?
#endif

void setup() {
  Serial.begin(115200);  //for debugging (may be changed by comm_UART_ASCII if comm_UART_serial is defined as Serial

  xTaskCreatePinnedToCore(core0task, "core0task", 4096, NULL, 1, NULL, 0); // (function, name, stack size, (void*) passed parameters, priority (0 == highest), not sure, core)

  commInit();
  #if defined(useOptFlow) && (useOptFlow == 1)
    opticalFlowInit();
  #endif
  
}


void loop() {
  commLoop();
}

/*
current pin commitments:
39, 36, 35, 34: encoders
33, 25: throttle ADC and DAC
15: brakes maybe
23, 19, 18, 5: PC communication
22, 17: steering AS5600
16, 4, 2: steering motor controller (H-bridge)
27, 26, 14, 13, 12: optical flow maybe
free pins:
32: extra pin (for encoders or throttle?)
0: BOOT pin
21: not on ESP32-LITE
15 = brakes MAYBE
*/
