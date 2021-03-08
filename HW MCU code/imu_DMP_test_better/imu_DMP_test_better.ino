
volatile uint32_t timers[10];


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary

// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 imu;
//MPU6050 imu(0x69); // <-- use for AD0 high

#define imuIntPin 17 //arduino has interrupts 0 and 1 (on pins 2 and 3 respectively), on ESP32 any pin can be an interrupt pin

uint16_t packetSize;    // expected DMP packet size (default is 42 bytes), used to store the info from dmpGetFIFOPacketSize(), which takes about 3micros to run
#define imuIntStatus_DMPint 0b00000010
#define imuIntStatus_FIFOovf 0b00010000

volatile bool imuIntTrig = false;
//void IRAM_ATTR imuISR(void* arg) { //for use with attachInterruptArg(pin, func, &arg, RISING);
void IRAM_ATTR imuISR() {
  imuIntTrig = true;
//  timers[8] = micros() - timers[9];
//  timers[9] = micros();
}


void setup() {
  Serial.begin(115200);
  I2Cdev::initialize(4,16,600000); //SDA, SCL, freq (esp32 is bugged, 400 -> 350Khz, 600Khz -> 500Khz)
  
  imu.initialize();
  Serial.println(imu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  uint8_t initError = imu.dmpInitialize(); // 1 = initial memory load failed,  2 = DMP configuration updates failed  (if it's going to break, usually the code will be 1)
  while(initError) { Serial.print("dmpInit error "); Serial.println(initError); delay(4000); } //if initError==0 then it's all good

//  imu.setXAccelOffset(-2698);
//  imu.setYAccelOffset(463);
//  imu.setZAccelOffset(1185);
//  imu.setXGyroOffset(-1);
//  imu.setYGyroOffset(-13);
//  imu.setZGyroOffset(121);

//  imu.CalibrateAccel(6);
//  imu.CalibrateGyro(6);
//  imu.PrintActiveOffsets();

  imu.setDMPEnabled(true);
  pinMode(imuIntPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(imuIntPin), imuISR, RISING);
  imu.getIntStatus(); //??
  //imu.setRate(0); //set accel/gyro sampling speed to max (data goes to DMP, esp only gets processed results every 9ms)
  
  Serial.println(imu.getDMPConfig1()); //function unknown
  Serial.println(imu.getDMPConfig2()); //function unknown
  Serial.println(imu.getDLPFMode()); //low pass, MPU6050_DLPF_BW_256=0=8khz_gyro
  //imu.setDLPFMode(1);
  Serial.println(imu.getDHPFMode()); //high pass, 
  Serial.println(imu.getFullScaleAccelRange()); //0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
  Serial.println(imu.getFullScaleGyroRange()); //0 = +/- 250 degrees/sec, 1 = +/- 500, 2 = +/- 1000, 3 = +/- 2000
  
  packetSize = imu.dmpGetFIFOPacketSize();
  Serial.print("packetSize: "); Serial.println(packetSize);
}

void loop() {
  if(imuIntTrig) {
    imuIntTrig = false;
    timers[0] = micros();
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    fifoCount = imu.getFIFOCount();  //takes about 300us
    timers[1] = micros();
    //Serial.print("fifocount:"); Serial.println(fifoCount);
    //Serial.print("getFIFOCount: "); Serial.println(timers[1]-timers[0]);
    
    if(fifoCount >= 200) {
      imu.resetFIFO();
      Serial.print(F("FIFO overflow: ")); Serial.print(fifoCount); Serial.println("!");
    } else if(fifoCount < packetSize) {
//      Serial.print("fifoCount < packetSize ...");
//      timers[0] = micros();
//      while (fifoCount < packetSize) { fifoCount = imu.getFIFOCount(); }
//      timers[1] = micros();
//      Serial.print("  waited "); Serial.println(timers[1]-timers[0]);
      //Serial.println("nope");
      return;
    }
    timers[8] = micros() - timers[9];
    timers[9] = micros();
    
    timers[1] = micros();
    uint8_t fifoBuffer[packetSize]; // FIFO storage buffer
    imu.getFIFOBytes(fifoBuffer, packetSize); //takes 1.5ms
    timers[2] = micros();
    fifoCount -= packetSize;
    if(fifoCount > 0) {
      Serial.print("fifoCount >0: "); Serial.println(fifoCount);
      imu.resetFIFO();
    }
//    Serial.print("getFIFOBytes: "); Serial.println(timers[2]-timers[1]);
//    Serial.println(timers[8]); //time between interrupts
    
//    int16_t rawQuaternion[4];
//    imu.dmpGetQuaternion(rawQuaternion, fifoBuffer);
//    Serial.print(rawQuaternion[0]);
//    Serial.print("\t");
//    Serial.print(rawQuaternion[1]);
//    Serial.print("\t");
//    Serial.print(rawQuaternion[2]);
//    Serial.print("\t");
//    Serial.println(rawQuaternion[3]);
    
    Quaternion q; // [w, x, y, z]
    imu.dmpGetQuaternion(&q, fifoBuffer); //get floating point quaternion (int quaternion divided by 16384)
    
    VectorFloat gravity;
    imu.dmpGetGravity(&gravity, &q);
    float ypr[3];
    imu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print(degrees(ypr[0]));
    Serial.print("\t");
    Serial.print(degrees(ypr[1]));
    Serial.print("\t");
    Serial.println(degrees(ypr[2]));

//    float euler[3];
//    imu.dmpGetEuler(euler, &q);
//    Serial.print(degrees(euler[0]));
//    Serial.print("\t");
//    Serial.print(degrees(euler[1]));
//    Serial.print("\t");
//    Serial.println(degrees(euler[2]));

    Serial.println();
  }
}


/*
offset calibration:
           XAccel                  YAccel                   ZAccel                   XGyro                YGyro                ZGyro
[-2699,-2698] --> [-13,6]  [463,464] --> [-6,8]  [1185,1186] --> [16382,16399] [-1,0] --> [0,7]  [-14,-13] --> [-3,1]  [120,121] --> [-3,1]

*/
