# sensor-data-collection

Scripts to collect and plot raw IMU (Witmotion IMU) and GPS data in csv format

## Usage

Run the following command to install required python libraries

    pip3 install -r requirements.txt
Run the following before recording any data
    
    mkdir -p transvahan_data_collect/Data

### IMU data

IMU_collect.py stores accelerometer values (along x and y axes) and gyrometer values (along z axis)  
To collect the IMU data run the following command

    python3 IMU_collect.py <name of the test run>

### GPS data

gps_collect.py stores the x and y cartesian coordinates w.r.t. start pose  
Change the Serial port for GPS according to requirement before running the script  
Run the following to collect GPS data

    python3 gps_collect.py <name of the test run>

### Extras

plot.py plot the data recorded in csv format  
Use following command to plot the desired datasets

    python3 plot.py <path of stored datafile> <column index (corresponding to axes) of data to plot> <argument_3>
argument_3 -> possible values = {1,0}  
argument_3 is 1 -> plot the GPS trajectory  
argument_3 is 0 -> plot the IMU sensor values
