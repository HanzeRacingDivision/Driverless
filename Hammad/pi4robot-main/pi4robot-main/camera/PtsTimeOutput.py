# From https://www.raspberrypi.org/forums/viewtopic.php?t=106930
import io
import time
import json


class PtsTimeOutput(object):
    def __init__(self, camera, video_filename, pts_filename):
        self.camera = camera
        self.video_output = io.open(video_filename, 'wb')
        self.pts_output = io.open(pts_filename, 'w')
        self.pts_output.write('[\n')
        self.start_cam_ts = None
        self.start_pi_ts = None

    def write(self, buf):
        self.video_output.write(buf)
        tsnow = time.time()
        f = self.camera.frame
        if f.complete and f.timestamp:
            
            if self.start_cam_ts is None:
                self.start_pi_ts = tsnow
                self.start_cam_ts = f.timestamp
            # The important data in the frame is the index, frame_type, and timestamp
            myframe = {
                'index': f.index,
                'frame_type': f.frame_type,
                'cam_ts': f.timestamp,
                'pi_ts': tsnow
            }
            self.pts_output.write(f'{json.dumps(myframe)},\n')

    def flush(self):
        self.video_output.flush()
        self.video_output.close()
        self.pts_output.write(']')
        self.pts_output.flush()
        self.pts_output.close()
