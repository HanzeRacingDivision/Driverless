import picamera
import multiprocessing as mp
import time


def double_record(q, finished):
    frames = []
    #Sensor mode 4 is full FoV, 2x2 binning, 1640x1232 (4x3), a wide range of framerates.
    #When the camera’s clock mode is 'reset' (the default), the timestamp is relative to the start of the video recording.
    #When the camera’s clock_mode is 'raw', it is relative to the last system reboot.
    with picamera.PiCamera(sensor_mode=4, resolution=(1024, 768), framerate=30, clock_mode='reset') as camera:
        camera.start_recording(f'output/v_{time.time()}.h264')
        for i, thing in enumerate(camera.capture_continuous('output/f_{counter}_{timestamp}.jpg', use_video_port=True, splitter_port=2, format='jpeg')):
            ts = time.time()
            f = camera.frame
            if f and f.complete and f.timestamp:
                frames.append({"fn": thing, "frame_ts": f.timestamp, "sys_ts": ts})

            #"thing" is whateverthe output type is. Filename, output object, etc.
            if finished.is_set():
                print(f)
                break
        print('Capture Continuous done')
        #camera.wait_recording(5)
        print('stopping recording...')
        #camera.stop_recording(splitter_port=2)
        camera.stop_recording()
    q.put(frames)

if __name__ == '__main__':
    print('Why, hello there.')
    finished = mp.Event()
    q = mp.SimpleQueue()
    capture_proc = mp.Process(target=double_record, args=(q, finished))
    capture_proc.start()
    #double_record()
    print("Recording started")
    print("Press Enter to stop recording")
    k = input()
    finished.set()
    capture_proc.join()
    frames = q.get()
    print(f"got {len(frames)} frame timestamps, here's the first: {frames[0]}")
    print('OK BYE!')