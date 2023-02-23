import numpy as np
import cv2

video_capture = cv2.VideoCapture(0)

while(True):
    ret, frame = video_capture.read()

    cv2.imshow("Frame", frame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()