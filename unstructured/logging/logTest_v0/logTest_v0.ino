#include "logging.h"

void setup() {
  Serial.begin(115200);

  uint16_t lenResult = logging::logStuff("test");
  Serial.print("result"); Serial.println(lenResult);
  Serial.write(logging::staticLogBuf, lenResult);
  
  lenResult = logging::logStuff(5);
  Serial.print("result"); Serial.println(lenResult);
  Serial.write(logging::staticLogBuf, lenResult);
}

void loop() {
  
}
