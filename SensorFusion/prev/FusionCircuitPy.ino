//import time - i don't know what this was for
//import board
#include <Adafruit_I2CDevice.h> //why don't this appear as suggestions even though they are installed and sparkfun's do
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_BNO08x.h> // shouldn't this be sparkfun one 
#include <blinka.h> //Not necessary yet since i found adafruit's
#define I2C_ADDRESS 0x60
#define BNO08X_CS 10
#define BNO08X_INT 9
Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice(I2C_ADDRESS);


void setup() {
  while (!Serial) { delay(10); }
  Serial.begin(115200);
  Serial.println("I2C device register test");

  if (!i2c_dev.begin()) {
    Serial.print("Did not find device at 0x");
    Serial.println(i2c_dev.address(), HEX);
    while (1);
  }
  Serial.print("Device found on address 0x");
  Serial.println(i2c_dev.address(), HEX);

  Adafruit_BusIO_Register id_reg = Adafruit_BusIO_Register(&i2c_dev, 0x0C, 2, LSBFIRST);
  uint16_t id;
  id_reg.read(&id);
  Serial.print("ID register = 0x"); Serial.println(id, HEX);

  Adafruit_BusIO_Register thresh_reg = Adafruit_BusIO_Register(&i2c_dev, 0x01, 2, LSBFIRST);
  uint16_t thresh;
  thresh_reg.read(&thresh);
  Serial.print("Initial threshold register = 0x"); Serial.println(thresh, HEX);

  thresh_reg.write(~thresh);

  Serial.print("Post threshold register = 0x"); Serial.println(thresh_reg.read(), HEX);
}
// i wish it was that easy just to import board and call it
// i2c = busio.I2C(board.SCL, board.SDA, frequency=400000);
bno = BNO08X_I2C(i2c);

bno.enable_feature(BNO_REPORT_ACCELEROMETER);
bno.enable_feature(BNO_REPORT_GYROSCOPE);
bno.enable_feature(BNO_REPORT_MAGNETOMETER);
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR);


while (){
  accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
  gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
  mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
  quat_i, quat_j, quat_k, quat_real = bno.quaternion  # pylint:disable=no-member
}
    