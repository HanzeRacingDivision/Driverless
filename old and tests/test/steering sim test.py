import numpy as np

steering_angle = -23
dt = 1/30
steerSimSubdt = 0.01

steering_max = 25.0 # (constant) (degrees) maximum steering deflection (in ackerman: atan(wheelbase / turning_radius), so NOT wheel deflection)
steering_simulated = 23
steering_velocity = 0.0
steering_accel_max = 1850 # (constant) steering acceleration at full motor power
steering_friction = 7.5 # (constant) as rotation speed increases, so does friction
steering_target_margin = 0.1 # (constant) acceptable margin of error for steering control loop

graphArray_time = []
graphArray_steering = []
graphArray_steering_simulated = []
graphArray_steering_velocity = []
graphArray_steering_accel = []

for i in range(30):
    if(i == 7):
        steering_angle *= -1 # suddenly change your mind when you're almost there
        print("i changed my mind, i actually want to go to:", steering_angle)
        print()

    ## simulate steering motor
    steering_angle = min(max(steering_angle, -steering_max), steering_max) # constrain steering target to within acceptable range
    for j in range(int(round(dt/steerSimSubdt, 0))): # run a sub-simulation just for the steering motor
        steerAbsDiff = abs(steering_simulated - steering_angle) # absolute angle difference
        # calculate ratio between time_remaining_if_you_start_braking_now and current_velocity using the area under the curve, 2*dist (2x because i'm thinking square, not triangle). note: doesnt use steering_friction!
        steerRequiredDecel = ((steering_velocity**2) / (2*steerAbsDiff) if (steerAbsDiff > 0.000001) else steering_accel_max) # (see comment above for math) avoid divide by 0 and substitute some high number in case it is 0
        if((steerAbsDiff > steering_target_margin) or (steerRequiredDecel > (steering_accel_max * 0.25))):
            steerAccel = (steering_accel_max if (steering_angle > steering_simulated) else -steering_accel_max)
            if(steerRequiredDecel > (steering_accel_max * 0.75)): # simulate the microcontroller's decision to start decelerating
                steerAccel = (-steering_accel_max if (steering_velocity > 0.0) else steering_accel_max) # brake steering motor
            steerAccel -= steering_friction * steering_velocity # higher steering velocity means more friction.
            steering_velocity += steerSimSubdt * steerAccel # update steering velocity
            steering_simulated += steerSimSubdt*steering_velocity + ((steerSimSubdt**2)/2.0)*steerAccel # SUVAT applied to rotational distance

            graphArray_time.append(i*dt+j*steerSimSubdt);
            graphArray_steering.append(steering_angle);
            graphArray_steering_simulated.append(steering_simulated);
            graphArray_steering_velocity.append(steering_velocity);
            graphArray_steering_accel.append(steerAccel);
        else:
            break
        print(round(steering_angle, 1), round(steering_simulated, 2), round(steering_velocity, 1), round(steerAccel, 1), round(steerRequiredDecel, 1))
    print()

import matplotlib.pyplot as plt

plt.plot(graphArray_time, graphArray_steering)
plt.plot(graphArray_time, graphArray_steering_simulated)
#plt.plot(graphArray_time, graphArray_steering_velocity)
#plt.plot(graphArray_time, graphArray_steering_accel)
plt.show()