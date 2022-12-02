import cv2
import time
import numpy as np 

#GLOBAL VARIABLES
Focal_Lenght = 3.67
Known_Width = 225

capture = cv2.VideoCapture(1)
confidenceThreshold = 0.8
nmsThreshold = 0.3  # Higher threshold => better dettection => Lower FPS

# Used to record the time when we processed last frame
prev_frame_time = 0

# Used to record the time at which we processed current frame
new_frame_time = 0

classesFile = 'coco.names'
classNames = []

with open(classesFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
# In case in future we still use coco.names this is for asigning 
# the names to an array cuz I am too lazy to do it manually

modelConfiguration = './Configurations/custom-yolov4-tiny-detector.cfg'
modelWeights = './Weights/custom-yolov4-tiny-detector_best.weights'
# To be replaced with future configurations

net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV) 
net.setPreferableTarget(cv2.dnn.DNN_BACKEND_DEFAULT)
# When running on Jetson both setPreferableBackend and setPreferableTarget
# Needs to be changed to cv2.dnn.DNN_BACKEND_CUDA

def findObject(outputs, img) :
     hT, wT, cT = img.shape
     bbox = [] # bbox stands for bounding box
     classIds = []
     confs = [] # conf stands for confidence 

     for output in outputs:
          for detection in output:
               scores = detection[5:]
               classId = np.argmax(scores)
               confidence = scores[classId]
               if confidence > confidenceThreshold:
                    w, h = int(detection[2] * wT), int(detection[3] * hT)
                    x, y = int((detection[0] * wT) - w / 2), int ((detection[1] * hT) - h / 2)
                    bbox.append([x, y, w, h])
                    classIds.append(classId)
                    confs.append(float(confidence))

     indices = cv2.dnn.NMSBoxes(bbox, confs, confidenceThreshold, nmsThreshold)  
     # NMS means Non Max Supression, a method to select the best bounding box
     # Follow the link at the end for more details 

     for i in indices:
          box = bbox[i]
          x, y, w, h = box[0], box[1], box[2], box[3]
          marker = (x, y), (x + w, y + h)
          distance = ((Known_Width * Focal_Lenght) / marker[1][0])
          cv2.rectangle(img, (x, y), (x + w, y + h), (255, 255, 0), 2)
          cv2.putText(img, f'{classNames[classIds[i]].upper()} {int(confs[i] * 100)}% Distance: {round(distance, 2)}', (x, y - 10), cv2.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 0))
          # Drawing bounding boxes and labes the detection


while True:
     success, img = capture.read()

     blob = cv2.dnn.blobFromImage(img, 1/255, (416, 416), [0, 0, 0], crop=False)
     net.setInput(blob)
     # See YOLO does not know how to read a video stream
     # But it knows to read blobs soo..

     layerNames = net.getLayerNames()
     outputNames = [layerNames[i - 1] for i in net.getUnconnectedOutLayers()]
     # Selecting just the output layers of the CNN made by YOLO
     
     outputs = net.forward(outputNames)

     findObject(outputs, img)

     gray = success

     font = cv2.FONT_HERSHEY_SIMPLEX
     
     new_frame_time = time.time()
     # Time when we finish processing for this frame

     gray = cv2.resize(gray, (500, 300))

     # Calculating the fps
 
     # Fps will be number of frame processed in given time frame
     # Since their will be most of time error of 0.001 second
     # We will be subtracting it to get more accurate result
     fps = 1/(new_frame_time-prev_frame_time)
     prev_frame_time = new_frame_time
 
     # Converting the fps into integer
     fps = int(fps)
 
     # Converting the fps to string so that we can display it on frame
     # By using putText function
     fps = str(fps)
 
     # Putting the FPS count on the frame
     cv2.putText(img, fps, (7, 30), font, 1, (100, 255, 0), 3, cv2.LINE_AA)

     cv2.imshow('Video Stream', img)

     if cv2.waitKey(1) & 0xFF == ord('q'):
          break


# NMS: https://medium.com/analytics-vidhya/non-max-suppression-nms-6623e6572536