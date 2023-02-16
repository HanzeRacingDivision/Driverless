import numpy as np
import cv2
import os, io, time
import multiprocessing as mp
from queue import Empty
from PIL import Image
import picamera


def do_capture(queue, finished):
    w, h = (640, 480)
    with picamera.PiCamera(resolution=(w, h), framerate=30) as camera:
        print("Capture starting")
        while not finished.wait(1.0 / 30):
            image = np.empty((h, w, 3), dtype=np.uint8)
            camera.capture(image, 'rgb')
            queue.put([time.time(), image])
        print("Capture complete")
    print("Capture disposed")
    queue.close()
    print("Queue closed")
    print("Leaving capture")


def test_proc(queue, finished):
    w, h = (320, 240)
    with picamera.PiCamera(resolution=(w, h), framerate=30) as camera:
        print("Initialized camera")
        while not finished.wait(1.0 / 30):
            image = np.empty((h, w, 3), dtype=np.uint8)
            camera.capture(image, 'rgb')
            #IF THIS ARRAY GETS BING ENOUGH, the program never terminates???
            queue.put([time.time(), np.zeros((320,240,3), dtype=np.uint8)])
            #queue.put([time.time(), np.zeros_like(image)])
            #queue.put([time.time(), image])
            print(f'Queue is size ~{queue.qsize()}')
    print("Stopped camera")
    print("Leaving test proc")


def do_processing(queue, finished):
    while not finished.wait(0.1) or not queue.empty():
        try:
            t, image = queue.get(False) #nonblocking, raises Empty if none available.
        except Empty:
            pass
        else:
            # Pretend it takes some time to process the frame
            #time.sleep(0.5)
            im = Image.fromarray(image)
            #im.save("PILtest.jpg")
            print(f'{os.getpid()}: T={t}, Processing image with shape {image.shape}')


if __name__ == '__main__':
    queue = mp.Queue()
    finished = mp.Event()
    capture_proc = mp.Process(target=test_proc, args=(queue, finished))
    processing_procs = [
        mp.Process(target=do_processing, args=(queue, finished))
        for i in range(4)
        ]
    for proc in processing_procs:
        proc.start()
    capture_proc.start()
    time.sleep(5) # Or seme button press or whatever...
    finished.set()
    print("Finished")
    capture_proc.join()
    print(f'AFTER CAPTURE JOIN Queue is size {queue.qsize()}')
    for proc in processing_procs:
        proc.join()
    print("Processing complete")
    print(f'FINAL Queue is size {queue.qsize()}')
    print("OK BYE")