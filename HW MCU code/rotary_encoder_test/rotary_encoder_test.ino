
#define encoPin 15

const float encoHoles = 1.0/6.0; //6 light-pulse-triggering holes in the encoder disc
const float encoGearRatio = 22.0/37.0; //37T gear on wheel side, 22T gear on encoder disc side
const float encoCountPerRev = encoHoles * encoGearRatio; //counts * this = wheel revolutions
//the previous values are absolute and precise, the wheel circumference is measured/calibrated and can vary a bit
const float encoWheelCirc = PI * 63.0; //2*pi*r = pi*d
const float encoCountPerMM = encoCountPerRev * encoWheelCirc; //counts * this = mm traveled (does not consider direction, assumes always forward)

volatile uint32_t encoCount = 0;
uint32_t encoCountLast;
uint32_t encoTimer; //stores timestamp associated with encoCountLast

IRAM_ATTR void encoISR() {
  encoCount += 1;
}


uint32_t printTimer;


void setup() {
  Serial.begin(115200);
  
  pinMode(encoPin, INPUT_PULLUP);
  attachInterrupt(encoPin, encoISR, FALLING);
}

void loop() {
  if(millis() > printTimer) {
    printTimer = millis() + 200;

    uint32_t countDif = encoCount - encoCountLast;
    float mmDif = countDif * encoCountPerMM;
    float speedVal = mmDif / (millis() - encoTimer); //millimeters / milliseconds = meters/second
    Serial.println(speedVal);
    
    encoCountLast = encoCount;
    encoTimer = millis();
  }
}
