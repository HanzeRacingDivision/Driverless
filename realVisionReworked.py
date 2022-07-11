# this is my (Thijs) modified/reworked version of the computervision code, to make sure it can run on the 

from pathlib import Path
from turtle import st
import depthai as dai
import numpy as np
import time
import json
import cv2
import os


Type = {
    'avi': cv2.VideoWriter_fourcc(*'XVID'),
    'mp4v': cv2.VideoWriter_fourcc(*'H264'),
    # 'mp4v': cv2.VideoWriter_fourcc(*'XVID'),
}

MAXIMUM_DISTANCE = 3200

ALLOW_BOUNDBOX_DIST_MATH_FALLBACK = False # allow the code to use the bounding box to (roughly approximate) position, ONLY IF distance measurement is not available (out of range)

class measurement:
    def __init__(self, coneColor: int=-1, conePos: np.ndarray=np.zeros(2), boundBox: np.ndarray=np.zeros((2,2)), dist: float=0.0, timestamp: float=0.0):
        self.color = coneColor
        self.pos = conePos # may be None if the distance measurement was out of range
        self.boundBox = boundBox # stores the 'bounding box' in propotional values (0.0 to 1.0), NOT pixels.  can be used to calculate an angle
        self.dist = dist
    
    # @staticmethod
    # def _calcPos(self, carToUse, dist: float=None, boundBox: np.ndarray=np.zeros((2,2))):
    #     ## calculate the position of the cone (in absolute coordinates, not relative to the car)
    #     ## TODO!


class DetectionMoule:
    def __init__(self):
        # Get argument first
        #nnBlobPath = "D:/Development/HARD/Car_Simulation/processing/computer_vision/YOLOv5/custom_model.blob"
        nnBlobPath = "416_half_shave.blob"

        if not Path(nnBlobPath).exists():
            import sys
            raise FileNotFoundError(
                f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

        filename = "Data_Cones.mp4"

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

        # Connect to device and start pipeline
        self.device = dai.Device(pipeline)

        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        self.previewQueue = self.device.getOutputQueue(
            name="rgb", maxSize=4, blocking=False)
        self.detectionNNQueue = self.device.getOutputQueue(
            name="detections", maxSize=4, blocking=False)
        self.xoutBoundingBoxDepthMappingQueue = self.device.getOutputQueue(
            name="boundingBoxDepthMapping", maxSize=4, blocking=False)
        self.depthQueue = self.device.getOutputQueue(
            name="depth", maxSize=4, blocking=False)
        # self.networkQueue = self.device.getOutputQueue(
        #     name="nnNetwork", maxSize=4, blocking=False)

        self.FPStimer = time.monotonic()
        self.frameCounter = 0
        self.fps = 0

        self.videoWrite = cv2.VideoWriter(
            filename, cv2.VideoWriter_fourcc(*'mp4v'), 25, (416, 416))

    def __del__(self):
        self.device.close()

    def getCones(self, excludeOutOfRange=False, returnFrame=False):
        framegrabTime = time.time()
        inPreview = self.previewQueue.get()
        inDet = self.detectionNNQueue.get()
        depth = self.depthQueue.get()
        # inNN = self.networkQueue.get() # (thijs) not needed???

        if(not returnFrame):
            returnList =  [] # will be a list of measurement() objects
            for detection in inDet.detections:
                if((int(detection.spatialCoordinates.z) != 0 and int(detection.spatialCoordinates.z) < MAXIMUM_DISTANCE) if excludeOutOfRange else True):
                    #conePos = measurement._calcPos() # TODO!
                    conePos = np.zeros(2)
                    returnList.append(measurement(detection.label, conePos, np.array([[detection.xmin,detection.xmax],[detection.ymin,detection.ymax]])))
            return(returnList)

        frame = inPreview.getCvFrame()
        depthFrame = depth.getFrame()  # depthFrame values are in millimeters

        depthFrameColor = cv2.normalize(
            depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depthFrameColor = cv2.equalizeHist(depthFrameColor)
        depthFrameColor = cv2.applyColorMap(
            depthFrameColor, cv2.COLORMAP_HOT)

        self.frameCounter += 1
        current_time = time.monotonic()
        if (current_time - self.FPStimer) > 1:
            self.fps = self.frameCounter / (current_time - self.FPStimer)
            self.frameCounter = 0
            self.FPStimer = current_time

        detections = inDet.detections
        if len(detections) != 0:
            boundingBoxMapping = self.xoutBoundingBoxDepthMappingQueue.get()
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

                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax,
                                                                ymax), (0, 255, 0), cv2.FONT_HERSHEY_SIMPLEX)

        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width = frame.shape[1]
        for detection in detections:
            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)
            try:
                label = (["blue", "yellow"])[detection.label]
            except:
                label = detection.label
            frame_aug = cv2.putText(frame, str(label), (x1 + 10, y1 + 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
            frame_aug = cv2.putText(frame, "{:.2f}".format(
                detection.confidence), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
            print("Label = ", end="")
            print(str(label))
            frame_aug = cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm",
                                    (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
            frame_aug = cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm",
                                    (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
            frame_aug = cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm",
                                    (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)

            Top_Left_Point = x1, y1
            Top_Right_Point = x2, y1
            Bottom_Left_Point = x1, y2
            Bottom_Right_Point = x2, y2
            
            dict = {
                "Label" : label,
                "Distance": int(detection.spatialCoordinates.z),
                "Top_Left_Point" : Top_Left_Point,
                "Top_Right_Point" : Top_Right_Point,
                "Bottom_Left_Point": Bottom_Left_Point,
                "Bottom_Right_Point": Bottom_Right_Point,
                "Time": time.time()
            }

            if int(detection.spatialCoordinates.z) != 0 and int(detection.spatialCoordinates.z) < MAXIMUM_DISTANCE :
                with open("detection.json", "r+") as file:
                    data = json.load(file)
                    data.append(dict)
                    file.seek(0)
                    json.dump(data, file)

                print(Top_Left_Point)
                print(Top_Right_Point)
                print(Bottom_Left_Point)
                print(Bottom_Right_Point)

            print()

            cv2.rectangle(frame, (x1, y1), (x2, y2),
                            (0, 255, 0), cv2.FONT_HERSHEY_SIMPLEX)

        frame_aug = cv2.putText(frame, "NN fps: {:.2f}".format(
            self.fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0))
        cv2.imshow("Depth frame", depthFrameColor)
        cv2.imshow("Cone detection", frame)

        self.videoWrite.write(frame)
