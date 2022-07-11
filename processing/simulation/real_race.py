import time
import pygame
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO, A2C, DQN
from constants import *
import depthai as dai

from CarEnv import CarEnv
from cone import *
import json


# encoders and lidars
import GF.generalFunctionsNoNumba as GF #(homemade) some useful functions for everyday ease of use
from log.HWserialConnLogging import kartMCUserialLogger
from HWserialConn import *
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
    camRgb.setResolution(
        dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    monoLeft.setResolution(
        dai.MonoCameraProperties.SensorResolution.THE_400_P)
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
        coords = [int(detection.spatialCoordinates.x), int(
            detection.spatialCoordinates.z), str(label)]

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


def disc_to_cont(action):
    if action == 0:
        return 0.5
    elif action == 1:
        return -0.5
    elif action == 2:
        return 1
    elif action == 3:
        return -1
    elif action == 4:
        return 0


class FakeCar():
    def __init__(self):
        self.desired_steering = 0
        self.desired_velocity = 0
        self.angle = 0
        self.position = [CAR_X_START_POSITION, -CAR_Y_START_POSITION]
        self.lastUpdateTimestamp = env.pp.clock.time_running
        self.velocity = 0


class RealCar(FakeCar, kartMCUserialClass, kartMCUserialLogger):
    """a class that simulates the car
        (replace Map.Car object with this)"""

    def __init__(self, clockFunc, logfilename=None):
        FakeCar.__init__(self)  # init Car object
        kartMCUserialClass.__init__(self, clockFunc)  # init Car object
        kartMCUserialLogger.__init__(self, logfilename)  # init logger


        if COLLECT_STEERING_DATA:
            self.connect(comPort=None, autoFind=True, tryAny=True, printDebug=True)
            self.doHandshakeIndef(resetESP=True, printDebug=True)
            self.requestSetSteeringEnable(self, False)
            self.requestSetPedalPassthroughEnable(self, True)

        # self.desired_steering_raw = np.int32(0) # see the @property for this value
        self.lastEncoderVals = [0, 0, 0, 0]

    def __del__(
            self):  # the automatically generated __del__ function only calls 1 (custom?) __del__ function, the first parent class's one. This solves that
        kartMCUserialClass.__del__(self)  # make sure to close the serial
        kartMCUserialLogger.__del__(self)  # make sure to close the logfile

    def setSteeringEnable(self, enabled):
        """enable/disable the steering motor (so a human can drive)
            (just a macro to requestSetSteeringEnable())"""
        self.requestSetSteeringEnable(self, enabled)  # pass 'self' as carToUse

    def setPedalPassthroughEnable(self, enabled):
        """enable/disable the throttle-pedal passthrough/override (so a human can drive)
            (just a macro to requestSetPedalPassthroughEnable())"""
        self.requestSetPedalPassthroughEnable(self, enabled)  # pass 'self' as carToUse

    @property
    def desired_steering_raw(self):
        return np.int32(self.desired_steering / STEERING_RAW_TO_RADIANS)

    def _rawSteeringToReal(self, rawSteerVal: np.int32):
        return rawSteerVal * STEERING_RAW_TO_RADIANS

    def _rawEncodersToMeters(self, rawEncoderValues: np.array):
        return (np.array([rawEncoderValues * ENCO_COUNT_TO_METERS[i] for i in range(4)], dtype=np.float32))

    def _encodersToForwardSpeed(self, encoderValues: np.array, dTime: float):
        ## TODO: calculate an appropriate forward velocity based on the distance-travelled of each wheel
        ## i'm thinking just the average of all wheels might work, or maybe just of the 2 rear wheels or something, i have to think about this some more...
        return ((encoderValues[0] - self.lastEncoderVals[0]) / dTime)

    def _encodersToForwardMovement(self, encoderValues: np.array):
        ## TODO: calculate an appropriate forward distance-travelled based on the distance-travelled of each wheel
        ## i'm thinking just the average of all wheels might work, or maybe just of the 2 rear wheels or something, i have to think about this some more...
        return (encoderValues[0] - self.lastEncoderVals[0])

    def _update_pos(self, dTime, dDist=None, applyUpdate=True):
        """ update the position of the car, based on velocity, steering and time-passage """
        if (dDist is None):
            dDist = self.velocity * dTime

        returnVal = [np.zeros((2)), 0.0]  # init var
        ## turning math
        if ((abs(self.steering) > 0.001) and (
                abs(self.velocity) > 0.001)):  # avoid divide by 0 (and avoid complicated math in a simple situation)
            turning_radius = WHEELBASE / np.tan(self.steering)
            angular_velocity = self.velocity / turning_radius
            arcMov = angular_velocity * dTime

            # one way to do it
            # turning_center = GF.distAnglePosToPos(turning_radius, self.angle+(np.pi/2), self.position) #get point around which car turns
            # newPosition = GF.distAnglePosToPos(turning_radius, self.angle+arcMov-(np.pi/2), turning_center)      #the car has traveled a a certain distancec (velocity*dt) along the circumference of the turning circle, that arc is arcMov radians long

            # another way of doing it
            forwardMov = np.sin(
                arcMov) * turning_radius  # sin(arc)*turning radius = movement paralel with (old) carAngle
            lateralMov = turning_radius - (np.cos(
                arcMov) * turning_radius)  # sin(arc)*turning radius = movement perpendicular to (old) carAngle
            movAngle = np.arctan2(lateralMov, forwardMov)  #
            diagonalMov = forwardMov / np.cos(
                movAngle)  # the length of a line between the start of the arc and the end of the arc
            newPosition = GF.distAnglePosToPos(diagonalMov, self.angle + movAngle, self.position)
            # newPosition[0] = self.position[0] + diagonalMov * np.cos(self.angle+movAngle) #same as using distAnglePosToPos
            # newPosition[1] = self.position[1] + diagonalMov * np.sin(self.angle+movAngle)

            returnVal[1] = arcMov
            # returnVal[0] = newPosition - self.position # numpy array addition
            returnVal[0] = np.array([newPosition[0] - self.position[0], newPosition[1] - self.position[1]])
            if (applyUpdate):
                ## update position
                self.angle += arcMov
                self.position[0] = newPosition[
                    0]  # keep the numpy array object the same (instead of replacing it with a whole new array every time)
                self.position[1] = newPosition[1]
        else:
            returnVal[0] = np.array([(dDist * np.cos(self.angle)), (dDist * np.sin(self.angle))])
            if (applyUpdate):
                self.position[0] = self.position[0] + returnVal[0][0]  # for some reason += doesnt work at the start
                self.position[1] = self.position[1] + returnVal[0][1]

        return returnVal  # return position and angle

    def update(self):
        dTime = self.clockFunc() - self.lastUpdateTimestamp
        if not self.is_ready:
            print("can't run realCar.update(), the connection is not ready:", self.is_ready)
            return False
        newData = self.requestKartData(carToUse=self, raw=USE_RAW_DATA)  # sends desiredSpeed and retrieves sensor data
        if newData is None:
            print("realCar.update() failed to retrieve data from the kart!")
            return False
        self.lastFeedbackTimestamp = self.clockFunc()  # save when the sensors last provided feedback
        self.logPacket(newData, self.lastFeedbackTimestamp)  # first of all, log the packet
        dDist = 0.0  # init var
        if USE_RAW_DATA:  # TODO: raw data
            self.steering = self._rawSteeringToReal(newData['steerAngle'])
            convertedEncoders = self._rawEncodersToMeters(newData['encoders'])
            self.velocity = self._encodersToForwardSpeed(convertedEncoders, dTime)
            dDist = self._encodersToForwardMovement(convertedEncoders)
        else:
            self.steering = newData['steerAngle']
            self.velocity = self._encodersToForwardSpeed(newData['encoders'], dTime)
            dDist = self._encodersToForwardMovement(newData['encoders'])
        self._update_pos(dTime, dDist, True)
        self.lastUpdateTimestamp = self.clockFunc()  # save when the car last updated its position (also done at SLAM)
        self.lastEncoderVals = newData['encoders']  # save (the distance-travelled of each wheel) for next time


def collect_cones_from_lidar(lidar, real_car, car_position):
    # 1) Receive data using HWserialConn.py
    lidar.DEBUG_checkIgnoredSerialData()
    lidar_data = lidar.requestLidarData(real_car)
    # 2) Process the data
    cone_data = processConeData(lidar_data, lidar)
    # 3) Make cone objects for the simulation
    perceived_cones = []
    for cone_info in cone_data:
        cone = Cone(cone_info[0], -cone_info[1], Side.LEFT, None)
        perceived_cones.append(cone)
    return perceived_cones

    # 4) Assign left/right label to each cone
    # TODO: For now all cones are LEFT, this is already assigned in previous step


def init_lidar(port):
    lidar = lidarESPserialClass(clockFunc=time.time, identifierIndex=0)
    lidar.connect(comPort=None, autoFind=True, tryAny=True, printDebug=True, exclusionList=[port])
    lidar.doHandshakeIndef(resetESP=True, printDebug=True)
    car = FakeCar()
    lidar.requestSetMaxRange(car, LIDAR_RANGE)
    return lidar


def match_cones(no_color, previous):
    initial_size = len(no_color)
    matched = []
    counter = 0
    for category in Side:
        for cone in previous[category]:
            for to_match in no_color:
                if (cone.position.x - to_match.position.x)**2 + (cone.position.y - to_match.position.y)**2 < DISTANCE_TO_MATCH**2:
                    to_match.category = cone.category
                    to_match.id = cone.id

                    cone.position.x = to_match.position.x
                    cone.position.y = to_match.position.y

                    matched.append(to_match)
                    no_color.remove(to_match)
                    counter += 1

    print("Perceived:", initial_size , "Matched:", counter)
    return matched, no_color


def add_new_ids(left_over_cones, current_highest_id):
    observation_with_ids = []
    for cone in left_over_cones:
        current_highest_id += 1
        cone.id = current_highest_id
        observation_with_ids.append(cone)
    return observation_with_ids


def collect_cones_from_camera(cone_data, car_position):
    perceived_cones = []
    #f = open("C:/Users/micha/Documents/GitHub/Car_Simulation/processing/computer_vision/YOLOv5/detection.json")
    #data = json.load(f)

    #for cone_data in data:
    #print(cone_data.get("label"))
    if cone_data is not None:
        if cone_data[2] == "Yellow":
            label = Side.LEFT
        else:
            label = Side.RIGHT
        x = cone_data[0]/1000
        y = cone_data[1]/1000


        # rotate
        phi = car.angle
        x, y = x * np.cos(phi) - y * np.sin(phi), x * np.cos(phi) - y * np.sin(phi)
        # add car x and y coordinates
        x += car_position.x
        y += car_position.y
        # add to the array
        print(x, y, label)
        cone = Cone(x, y, label, None)
        perceived_cones.append(cone)

    # f = open("C:/Users/micha/Documents/GitHub/Car_Simulation/processing/computer_vision/YOLOv5/detection.json", "w")
    # #data = json.load(f)
    # data = []
    # json.dump(data, f)
    return perceived_cones


if __name__ == "__main__":
    models_dir = f'models/{MODEL_NAME}-{MODEL_NUMBER}'
    env = CarEnv(mode="cont")
    check_env(env)

    if MODEL_NAME == 'PPO' or MODEL_NAME == 'PPO-cont':
        model = PPO.load(f"{models_dir}/car_model_{TIME_STEPS}")
    elif MODEL_NAME == 'A2C' or MODEL_NAME == 'A2C-cont':
        model = A2C.load(f"{models_dir}/car_model_{TIME_STEPS}")
    elif MODEL_NAME == 'DQN' or MODEL_NAME == 'DQN-cont':
        model = DQN.load(f"{models_dir}/car_model_{TIME_STEPS}")
    else:
        raise ValueError(f"{MODEL_NAME} not recognized as valid model name")

    episodes = 1

    car = RealCar(time.time)

    if COLLECT_CAMERA_DATA:
        #camera = DetectionModule()
        #camera = None
        pipeline = start_Camera()

        # Tiny yolo v3 / 4 label texts
        labelMap = [
            "Blue", "Yellow"
        ]

        start = time.time()
        
        # Connect to device and start pipeline
        device = dai.Device(pipeline)
        #with dai.Device(pipeline) as device:
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
    else:
        camera = None

    if COLLECT_LIDAR_DATA:
        lidar = init_lidar(car.comPort)
    else:
        lidar = None

    if COLLECT_STEERING_DATA and COLLECT_LIDAR_DATA:
        print("shuffleSerials success:", shuffleSerials([car, lidar]))

    for i in range(episodes):
        done = False
        observation = env.reset()
        while not done:

            # this is the desired action
            if env.pp.track_number > 0:
                action, _states = model.predict(observation)
                if CONVERSION == "disc_to_cont":
                    action = disc_to_cont(action)
                action = [action]
                #print("agent:", action)
            else:
                angle = env.pp.midpoint_steering_angle()
                action = np.interp(angle, [-120, 120], [-5, 5])
                action = [action]
                #print("midpoint:", action)

            # send these values to the car object
            car.desired_velocity = 1
            car.desired_steering = action[0]
            # update the Thijs' car object
            # it will
                # send the desired steering data
                # read the encoders
                # update its position using those encoders
            if COLLECT_STEERING_DATA:
                car.update()
                # retrieve the position from Thijs' car and update the environment
                env.pp.car.position.x = car.position[0]
                env.pp.car.position.y = -car.position[1]
                env.pp.car.true_position.x = car.position[0]
                env.pp.car.true_position.y = -car.position[1]
                env.pp.car.true_angle = -np.rad2deg(car.angle)
                env.pp.car.angle = -np.rad2deg(car.angle)

            if COLLECT_CAMERA_DATA:
                data = getCones(previewQueue, detectionNNQueue, depthQueue, networkQueue,
                               xoutBoundingBoxDepthMappingQueue, labelMap, start)
                collected_cones = collect_cones_from_camera(data, env.pp.car.position)
                matched_cones, leftover_cones = match_cones(collected_cones, env.pp.cones.visible)
                current_highest_cone_id = len(env.pp.cones.list[Side.LEFT]) + len(env.pp.cones.list[Side.RIGHT])
                new_cones = add_new_ids(leftover_cones, current_highest_cone_id)

                for new in new_cones:
                    env.pp.cones.visible[new.category].append(new)

            if COLLECT_LIDAR_DATA:
                cones_no_color = collect_cones_from_lidar(lidar, car, env.pp.car.position)
                matched_cones, leftover_cones = match_cones(cones_no_color, env.pp.cones.visible)
                current_highest_cone_id = len(env.pp.cones.list[Side.LEFT]) + len(env.pp.cones.list[Side.RIGHT])
                # TODO: Match new cones with camera data to get labels
                # new_cones = add_color_and_add_ids_to_cones(leftover_cones, current_highest_cone_id)
                new_cones = add_new_ids(leftover_cones, current_highest_cone_id)

                # Add new cones to the simulation environment
                for new in new_cones:
                    env.pp.cones.visible[new.category].append(new)

                # for category in Side:
                #     for cone in env.pp.cones.visible[category]:
                #         print(cone.position)

            # Run SLAM and all the other epic stuff
            # observation, reward, done, info = env.step(action)
            env.render()

            if done:
                print(env.total_reward)

            if env.pp.exit:
                break

    pygame.quit()
