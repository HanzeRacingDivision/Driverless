from pathlib import Path
import sys
from turtle import st
import cv2
import depthai as dai
import numpy as np
import time
import os

Type = {
    'avi': cv2.VideoWriter_fourcc(*'XVID'),
    'mp4v': cv2.VideoWriter_fourcc(*'H264'),
    # 'mp4v': cv2.VideoWriter_fourcc(*'XVID'),
}


def start_Camera():
    # Get argument first
     nnBlobPath = "D:/Development/HARD/Car_Simulation/processing/computer_vision/YOLOv5/416_half_shave.blob"
     #nnBlobPath = "416_half_shave.blob"
     #nnBlobPath = "416_half_shave.blob"

     if not Path(nnBlobPath).exists():
          import sys
          raise FileNotFoundError(
               f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

     syncNN = True

     # Create pipeline
     pipeline = dai.Pipeline()

     # Define sources and outputs
     camRgb = pipeline.create(dai.node.ColorCamera)
     spatialDetectionNetwork = pipeline.create(
          dai.node.YoloSpatialDetectionNetwork)
     monoLeft = pipeline.create(dai.node.MonoCamera)
     monoRight = pipeline.create(dai.node.MonoCamera)
     stereo = pipeline.create(dai.node.StereoDepth)
     nnNetworkOut = pipeline.create(dai.node.XLinkOut)

     xoutRgb = pipeline.create(dai.node.XLinkOut)
     xoutNN = pipeline.create(dai.node.XLinkOut)
     xoutBoundingBoxDepthMapping = pipeline.create(dai.node.XLinkOut)
     xoutDepth = pipeline.create(dai.node.XLinkOut)

     xoutRgb.setStreamName("rgb")
     xoutNN.setStreamName("detections")
     xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
     xoutDepth.setStreamName("depth")
     nnNetworkOut.setStreamName("nnNetwork")

     # Properties
     camRgb.setPreviewSize(416, 416)
     camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
     camRgb.setInterleaved(False)
     camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

     monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
     monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
     monoRight.setResolution(
          dai.MonoCameraProperties.SensorResolution.THE_400_P)
     monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

     # setting node configs
     stereo.setDefaultProfilePreset(
          dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
     # Align depth map to the perspective of RGB camera, on which inference is done
     stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
     stereo.setOutputSize(monoLeft.getResolutionWidth(),
                         monoLeft.getResolutionHeight())

     spatialDetectionNetwork.setBlobPath(nnBlobPath)
     spatialDetectionNetwork.setConfidenceThreshold(0.5)
     spatialDetectionNetwork.input.setBlocking(False)
     spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
     spatialDetectionNetwork.setDepthLowerThreshold(100)
     spatialDetectionNetwork.setDepthUpperThreshold(5000)

     # Yolo specific parameters
     spatialDetectionNetwork.setNumClasses(2)
     spatialDetectionNetwork.setCoordinateSize(4)
     spatialDetectionNetwork.setAnchors(
          [
               2.6171875,
               6.7734375,
               4.28125,
               9.484375,
               6.53515625,
               14.109375,
               9.5625,
               19.484375,
               12.7578125,
               26.0625,
               17.640625,
               35.46875,
               23.0,
               43.90625,
               27.8125,
               57.5,
               40.25,
               78.0
          ])
     spatialDetectionNetwork.setAnchorMasks(
          {"side52": [
               0,
               1,
               2
          ],
               "side26": [
               3,
               4,
               5
          ],
               "side13": [
               6,
               7,
               8
          ]})
     spatialDetectionNetwork.setIouThreshold(0.5)

     # Linking
     monoLeft.out.link(stereo.left)
     monoRight.out.link(stereo.right)

     camRgb.preview.link(spatialDetectionNetwork.input)
     if syncNN:
          spatialDetectionNetwork.passthrough.link(xoutRgb.input)
     else:
          camRgb.preview.link(xoutRgb.input)

     spatialDetectionNetwork.out.link(xoutNN.input)
     spatialDetectionNetwork.boundingBoxMapping.link(
          xoutBoundingBoxDepthMapping.input)

     stereo.depth.link(spatialDetectionNetwork.inputDepth)
     spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
     spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)

     return pipeline


def getCones(previewQueue, detectionNNQueue, depthQueue, networkQueue, xoutBoundingBoxDepthMappingQueue, labelMap, start):
     startTime = time.monotonic()
     counter = 0
     fps = 0
     color = (255, 255, 255)

     inPreview = previewQueue.get()
     inDet = detectionNNQueue.get()
     depth = depthQueue.get()
     inNN = networkQueue.get()

     frame = inPreview.getCvFrame()
     depthFrame = depth.getFrame()  # depthFrame values are in millimeters

     depthFrameColor = cv2.normalize(
          depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
     depthFrameColor = cv2.equalizeHist(depthFrameColor)
     depthFrameColor = cv2.applyColorMap(
          depthFrameColor, cv2.COLORMAP_HOT)

     counter += 1
     current_time = time.monotonic()
     if (current_time - startTime) > 1:
          fps = counter / (current_time - startTime)
          counter = 0
          startTime = current_time

     detections = inDet.detections
     if len(detections) != 0:
          boundingBoxMapping = xoutBoundingBoxDepthMappingQueue.get()
          roiDatas = boundingBoxMapping.getConfigData()

          for roiData in roiDatas:
               roi = roiData.roi
               roi = roi.denormalize(
                    depthFrameColor.shape[1], depthFrameColor.shape[0])
               topLeft = roi.topLeft()
               bottomRight = roi.bottomRight()
               xmin = int(topLeft.x)
               ymin = int(topLeft.y)
               xmax = int(bottomRight.x)
               ymax = int(bottomRight.y)

               cv2.rectangle(depthFrameColor, (xmin, ymin),
                              (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)

     # If the frame is available, draw bounding boxes on it and show the frame
     height = frame.shape[0]
     width = frame.shape[1]

     coords = 0

     for detection in detections:
          # Denormalize bounding box
          x1 = int(detection.xmin * width)
          x2 = int(detection.xmax * width)
          y1 = int(detection.ymin * height)
          y2 = int(detection.ymax * height)
          try:
               label = labelMap[detection.label]
          except:
               label = detection.label
          frame_aug = cv2.putText(frame, str(label), (x1 + 10, y1 + 20),
                                   cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
          frame_aug = cv2.putText(frame, "{:.2f}".format(
               detection.confidence), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
          frame_aug = cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm",
                                   (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
          frame_aug = cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm",
                                   (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
          frame_aug = cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm",
                                   (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
          coords = [int(detection.spatialCoordinates.x), int(detection.spatialCoordinates.z), str(label)]

          print()

          cv2.rectangle(frame, (x1, y1), (x2, y2),
                         color, cv2.FONT_HERSHEY_SIMPLEX)

     frame_aug = cv2.putText(frame, "NN fps: {:.2f}".format(
          fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
     cv2.imshow("Depth frame", depthFrameColor)
     cv2.imshow("Cone detection", frame)

     if coords != 0:
          return (coords)

    #if cv2.waitKey(1) == ord('q'):
    #break


class DetectionModule:
     pipeline = start_Camera()

     # Tiny yolo v3 / 4 label texts
     labelMap = [
         "Blue", "Yellow"
     ]

     start = time.time()

     # Connect to device and start pipeline
     device = dai.Device(pipeline)
     #with dai.Device(pipeline) as device:

     # Output queues will be used to get the rgb frames and nn data from the outputs defined above
     previewQueue = device.getOutputQueue(
          name="rgb", maxSize=4, blocking=False)
     detectionNNQueue = device.getOutputQueue(
          name="detections", maxSize=4, blocking=False)
     xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(
          name="boundingBoxDepthMapping", maxSize=4, blocking=False)
     depthQueue = device.getOutputQueue(
          name="depth", maxSize=4, blocking=False)
     networkQueue = device.getOutputQueue(
          name="nnNetwork", maxSize=4, blocking=False)

     #while True:
      #     print(getCones(previewQueue, detectionNNQueue, depthQueue, networkQueue,
       #        xoutBoundingBoxDepthMappingQueue, labelMap, start))
        #   if cv2.waitKey(1) == ord('q'):
         #       break