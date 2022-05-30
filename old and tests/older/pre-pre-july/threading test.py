import threading as thr

import carMCUclass as RC

someCar = RC.carMCU(comPort='COM5')

import time

try:
    threadKeepRunning = [True]
    someThread = thr.Thread(target=someCar.runOnThread, name="test", args=(threadKeepRunning, True, ), daemon=True)
    someThread.start()
    while True:
        print(thr.active_count())
        time.sleep(2)
except KeyboardInterrupt:
    print("main thread keyboard interrupt")
except Exception as excep:
    print("main thread exception:", excep)
finally:
    try:
        threadKeepRunning[0] = False #signal the function to stop its while() loop (the list is just a manual boolean pointer (hack))
        someThread.join(1)
        print("alive:", someThread.is_alive())
    except:
        print("couldn't stop thread?")