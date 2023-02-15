from multiprocessing.sharedctypes import Value
from .bno055 import BNO055
from .bn220 import BN220
from .utils import *
import time
import json

class StrapDown():
    GRAV_AVG = 100.
    def __init__(self):
        self._imu = BNO055()
        self._gps = BN220()
        self._imu.initialize()
        status, self_test, error = self._imu.get_system_status() #IDK why but you need this
        self._attitude = Quaternion()
        self._omega = Quaternion(0)
        self._velocity = Quaternion(0)
        self._position = Quaternion(0)
        self._gyro = Quaternion(0)
        self._accl = Quaternion(0)
        self._prev_a_b = Quaternion(0)
        self._prev_omega = Quaternion(0)
        self._prev_time = time.time()
        self._events = []
        self._gravity = None
        self._count = 0
        self._loop()

    def _loop(self):
        self._get_gravity()
        self._start_time = time.time()
        while True:
            self._thread_function()
            event = json.dumps({'time_stamp':self._prev_time,
                'atitude':self._attitude.json(),
                'position':self._position.json(),
                'velocity':self._velocity.json(),
                'gyro':self._gyro.json(),
                'imu':self._accl.json(),
                'omega':self._omega.json()})
            self._events.append(event)
            if self._attitude.mag() > 5:
                self._attitude = self._attitude.unit()
    
    def _thread_function(self):
        #Data Collection
        a,m,w,t = self._imu.read_accl_mag_gyro()
        wx,wy,wz = w
        ax,ay,az = a

        #Time Step
        dt = t - self._prev_time
        self._prev_time = t
        
        #sanity Check values
        if abs(wx) > 40 or  abs(wy) > 40 or abs(wz) > 40:
            print(f"Rotational velocity too high ({wx},{wy},{wz}")
        if abs(ax) > 100 or  abs(ay) > 100 or abs(az) > 100:
            print(f"Acceleration too hgih: ({ax},{ay},{az})")
        omega = Quaternion(0,wx,wy,wz)
        a_b = Quaternion(0,ax,ay,az)
        self._gyro = omega
        self._accl = a_b

        #Integrate Gyroscope to update Attitude
        dq_k = (self._attitude * ((omega+self._prev_omega)*.5)) * .5
        self._prev_omega = omega
        self._attitude += dq_k * dt

        #Error Correction
        error = ((self._attitude*self._attitude.conjugate())-Quaternion())*.5
        self._attitude = (Quaternion() - error)*self._attitude
        print("----------------------------------------")
        omega.print()
        error.print()
        self._attitude.print()

        #Subtract out gravity, convert acceleration to Inertial Frame and Integrate to update position
        a_i_g = self._attitude * a_b * self._attitude.conjugate()
        g = self._attitude * self._gravity * self._attitude.conjugate()
        a_i = a_i_g - g
        v_i = a_i * dt
        self._velocity = self._velocity + v_i
        position = self._velocity * dt
        position._a = 0 # Kinda Hacky but there is no rotation just xyz position
        self._position = self._position + position
    
    def _get_gravity(self):
        # Get Gravity
        x = 0.
        y = 0.
        z = 0.

        for i in range(int(self.GRAV_AVG)):
            ax,ay,az = self._imu.read_accelerometer()
            x += ax
            y += ay
            z += az
        self._gravity = Quaternion(0,x/self.GRAV_AVG,y/self.GRAV_AVG,z/self.GRAV_AVG)
    
    def __del__(self):
        file_name = "accl_" + str(self._start_time) + ".json"
        with open(file_name, 'w') as f:
            for event in self._events:
                f.write(event + '\n')