import picamera
import multiprocessing as mp
from PtsTimeOutput import PtsTimeOutput
import time


def ptsrecord(finished):
    ts = time.time()
    ptso = PtsTimeOutput(None, f'output/v_{ts}.h264', f'output/pts_{ts}.json')
    #Sensor mode 4 is full FoV, 2x2 binning, 1640x1232 (4x3), a wide range of framerates.
    #When the camera’s clock mode is 'reset' (the default), the timestamp is relative to the start of the video recording.
    #When the camera’s clock_mode is 'raw', it is relative to the last system reboot.
    with picamera.PiCamera(sensor_mode=4, resolution=(1024, 768), framerate=30, clock_mode='reset') as camera:
        ptso.camera = camera
        camera.start_recording(ptso, format='h264')
        print("Camera process is recording")
        while not finished.is_set():
            camera.wait_recording(1)
        print('stopping recording...')
        camera.stop_recording()
    print(f"Camera closed, PTSO cam start: {ptso.start_cam_ts}, Pi start: {ptso.start_pi_ts}")


if __name__ == '__main__':
    print('Why, hello there.')
    finished = mp.Event()
    q = mp.SimpleQueue()
    capture_proc = mp.Process(target=ptsrecord, args=(finished,))
    capture_proc.start()
    #double_record()
    print("Recording started")
    print("Press Enter to stop recording")
    k = input()
    finished.set()
    capture_proc.join()
    print('OK BYE!')