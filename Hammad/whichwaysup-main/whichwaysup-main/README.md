# whichwaysup
Simple webservice for Adafruit BNO08x IMU on Raspberry Pi i2c interface. IMU Inertial measurement unit.

Details of the BNO08x can be found here https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/overview

## env

Python env created with
```bash
python3 -m venv env
```
Activate the env with
```
source env/bin/activate
```

## Dependencies

Add web.py and the circuit python library for the board
```bash
pip3 install web.py
sudo pip3 install adafruit-circuitpython-bno08x
```

Setting up the PI, works best setting the i2c baud rate to 400kHz, by adding to the /boot/config.txt file and rebooting the pi

```bash
dtparam=i2c_arm_baudrate=400000
```


# Running

To run
```
./imu_webservice.py
```


Point web browser http://raspberrypi.local:8080/ and we should get something like:
```
{"acceleration":[0.976562,-0.800781,9.664062],"gyro":[0.000000,0.001953,0.001953],"magnetometer":[-169.000000,-42.687500,-36.750000],"quaternione":[-0.042725,-0.050964,0.001831],"quaternione_real":0.997803}
```
