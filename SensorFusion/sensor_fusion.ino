
#include <Wire.h> //used in BN080
#include "SparkFun_BNO080_Arduino_Library.h" // https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library
#include "Streaming.h" 		// needed for the Serial output https://github.com/geneReeves/ArduinoStreaming
#include "SensorFusion.h"   // https://github.com/aster94/SensorFusion 

SF fusion;
BNO080 myIMU;
float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float pitch, roll, yaw;
float deltat;
int status;

#define EULER_DATA
//#define RAW_DATA
//#define PROCESSING
//#define SERIAL_PLOTER

void setup() {

    //serial to display data
    Serial.begin(115200);
    while (!Serial) {}
    Serial.println();
    Serial.println("BNO080 Read Example");

    status= myIMU.begin();
        if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while (1) {}
    }

    Wire.begin();
    // start communication with IMU (initialized outside with #define SS_PIN PB12, SPIClass mySPI (2) and MPU9250 IMU(mySPI, SS_PIN);)
    if (myIMU.begin() == false){
        Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));
        while (1);
    }
    Wire.setClock(400000); //Increase I2C data rate to 400kHz
    //Enable dynamic calibration for accel, gyro, and mag
    myIMU.calibrateAll(); //Turn on cal for Accel, Gyro, and Mag

    myIMU.enableLinearAccelerometer(50); //Send data update every 50ms
    Serial.println(F("Linear Accelerometer enabled"));
    Serial.println(F("Output in form x, y, z, in m/s^2"));

    myIMU.enableGyro(50); //Send data update every 50ms
    Serial.println(F("Gyro enabled"));    

    myIMU.enableMagnetometer(50); //Send data update every 50ms
    Serial.println(F("Magnetometer enabled"));
    //Once magnetic field is 2 or 3, run the Save DCD Now command
    Serial.println(F("Calibrating. Press 's' to save to flash"));
    Serial.println(F("Output in form x, y, z, in uTesla"));
}

//Given a accuracy number, print what it means
void printAccuracyLevel(byte accuracyNumber){
  if(accuracyNumber == 0) Serial.print(F("Unreliable"));
  else if(accuracyNumber == 1) Serial.print(F("Low"));
  else if(accuracyNumber == 2) Serial.print(F("Medium"));
  else if(accuracyNumber == 3) Serial.print(F("High"));
}

void loop() {
    if(Serial.available()){
        byte incoming = Serial.read();
        if(incoming == 's'){
            myIMU.saveCalibration(); //Saves the current dynamic calibration data (DCD) to memory
            myIMU.requestCalibrationStatus(); //Sends command to get the latest calibration status

            //Wait for calibration response, timeout if no response
            int counter = 100;
            while(1){
                if(--counter == 0) break;
                if(myIMU.dataAvailable() == true){
                //The IMU can report many different things. We must wait
                //for the ME Calibration Response Status byte to go to zero
                    if(myIMU.calibrationComplete() == true){
                        Serial.println("Calibration data successfully stored");
                        delay(1000);
                        break;
                    }
                }
                delay(1);
            }
            if(counter == 0){
                Serial.println("Calibration data failed to store. Please try again.");
            }
        //myIMU.endCalibration(); //Turns off all calibration
        //In general, calibration should be left on at all times. The BNO080
        //auto-calibrates and auto-records cal data roughly every 5 minutes
        }
    }

    //sensorFusion.h uses IMU.readSensor(); and their own getAccelX_mss, getGyroX_rads(), getMagX_uT
    //we will use IMU's library to ensure we get there
    //can be source of conflict in the data fusion

    if (myIMU.dataAvailable() == true){
        ax = myIMU.getAccelX();
        ay = myIMU.getAccelY();
        az = myIMU.getAccelZ();

        Serial.print(x, 2);
        Serial.print(F(","));
        Serial.print(y, 2);
        Serial.print(F(","));
        Serial.print(z, 2);
        Serial.print(F(","));

        Serial.println();

        //gyro
        gx = myIMU.getGyroX();
        gy = myIMU.getGyroY();
        gz = myIMU.getGyroZ();

        Serial.print(x, 2);
        Serial.print(F(","));
        Serial.print(y, 2);
        Serial.print(F(","));
        Serial.print(z, 2);
        Serial.print(F(","));

        Serial.println();

        //magnetometer
        mx = myIMU.getMagX();
        my = myIMU.getMagY();
        mz = myIMU.getMagZ();
        
        byte accuracy = myIMU.getMagAccuracy();        
        Serial.print(x, 2);
        Serial.print(F(","));
        Serial.print(y, 2);
        Serial.print(F(","));
        Serial.print(z, 2);
        Serial.print(F(","));
        printAccuracyLevel(accuracy);
        Serial.print(F(","));
        Serial.println();
        //we can also do the same thing with quaternions with getQuatI/J/K and storing in indep variables and making acurracy
    }

#ifdef RAW_DATA
  Serial << "From last Update:\t"; Serial.println(deltat, 6);
  Serial << "GYRO:\tx:" << gx << "\t\ty:" << gy << "\t\tz:" << gz << newl;
  Serial << "ACC:\tx:" << ax << "\t\ty:" << ay << "\t\tz:" << az << newl;
  Serial << "MAG:\tx:" << mx << "\t\ty:" << my << "\t\tz:" << mz << newl;
  Serial << "TEMP:\t" << temp << newl << newl;
#endif

    temp = IMU.getTemperature_C();
    deltat = fusion.deltatUpdate();
    //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony is suggested if there isn't the mag
    fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick

    roll = fusion.getRoll();
    pitch = fusion.getPitch();
    yaw = fusion.getYaw();

#ifdef EULER_DATA
  Serial << "Pitch:\t" << pitch << "\t\tRoll:\t" << roll << "\t\tYaw:\t" << yaw << newl << newl;
#endif

#ifdef PROCESSING
  roll = fusion.getRollRadians();
  pitch = fusion.getPitchRadians();
  yaw = fusion.getYawRadians();
  Serial  << pitch << ":" << roll << ":" << yaw << newl;
#endif

#ifdef SERIAL_PLOTER
  Serial << pitch << " " << roll << " " << yaw << endl;
#endif

  delay(200); //for readability
}