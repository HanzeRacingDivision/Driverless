/*
proper and abundant logging is paramount for ease of debugging,
and it's probably also the easiest way to calibrate the throttle gains.
if the optical flow sensor doens't work out, we can just use those pins to interface with an SD card (they're already SPI pins),
but there are also excess pc-comm pins, maybe we can use the third serial port for logging or some shit

let's be cool guys and invent a custom logging library and file type to go along with it, becuase fuck it, right
like a fucked up excel sheet where the only constant column is the timestamp (all should probably be in SI units if possible, or in raw data if SI units are derived)
all other columns should be able to differ every frickin row, if a value is not given for a particular column some rows, who cares. Just total chaos, with a proper interpreter.
i'd like to keep the logging a 1-way street, where the data-source just spouts some shit, and it's up to the logging MCU to make sense of it
i'll have to write the following codes:
- logging library: 25hrs
  + for local logging and serial/remote logging: 25hrs
    * for both arduino and python: 50 hrs
- receiver for that serial/remote logging: 10hrs
  + both arduino and python: 25 hrs
- interpreter for log (python only): 50hrs
  + any kind of logfile, with minimal effort: 25hrs
    * maybe a funky little API for visualizing arbetrary log data: 25hrs
      (for every column title (each data type) a custom function (with some pre-existing functions for graph's n shit) to easily visualize the data)

after some further consideration, i have come to the conclusion that:
 if we want to study logfiles using the simulation tools, and the simulation tools are basically the same code as the real-life code,
 then either i need to make an extensive API to translate normal (simulation or real) variables into log files, and the other way around
 OR, i find a compromise, where the simulation/real-life code uses the same (or nearly same) storage structure as the logfiles.
 Now, this poses many issues, mostly in terms of how stringent and limited the codebase for the driverless code becomes,
  also, the logfiles may end up being huge, and the sheer bandwidth required to make a logfile at runtime will significantly impact the performance of the driverless code (which is bad)
 The easiest solution might be to compartmentalize the driverless code, sothat a seperate visualization program can make use of them.
  Maybe, it could make sense to seperate the simulation code from the realtime code, and integrate the simulation into the logfile visualization...
 As important as proper debugging tools are, they should not impact the performance of the real-time driverless code too much,
  and one should not introduce complexity in the way driverless code MUST be written, for the sake of easier log visualization.
*/

#pragma once

namespace logging { //this is to make clear to other sketches where the variables come from (and now i can use the same variablename in different namespaces)

const uint16_t staticLogBufSize = 1000;
static uint8_t staticLogBuf[staticLogBufSize];

template<typename T> uint16_t logStuff(T input, uint8_t* logBuf = staticLogBuf) {
  
}


}
