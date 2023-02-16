# From https://www.raspberrypi.org/forums/viewtopic.php?t=106930
import io
import picamera
import time

class PtsOutput(object):
    def __init__(self, camera, video_filename, pts_filename):
        self.camera = camera
        self.video_output = io.open(video_filename, 'wb')
        self.pts_output = io.open(pts_filename, 'w')
        self.start_time = None

    def write(self, buf):
        self.video_output.write(buf)
        if self.camera.frame.complete and self.camera.frame.timestamp:
            if self.start_time is None:
                self.start_time = self.camera.frame.timestamp
            self.pts_output.write('# timecode format v2\n')
            self.pts_output.write('%f\n' % ((self.camera.frame.timestamp - self.start_time) / 1000.0))

    
    def flush(self):	
        self.video_output.flush()
        self.pts_output.flush()
        
    def close(self):
        self.video_output.close()
        self.pts_output.close()