import numpy as np

# Race
MODEL_NAME = "PPO"
MODEL_NUMBER = "1649940839"  # disc: 1649940839/ cont: 1651669206
TIME_STEPS = 500000   # disc: 500000, cont: 580000
CONVERSION = "disc_to_cont"  # disc_to_cont

# Path Planning
DISTANCE_TO_MATCH = 0.3
MODE = "race"  #
# "race": one lap slam and midpoint method, then agent without SLAM
# "testing": everything else :)
STEERING_METHOD = "autonomous"  # "user"/"autonomous"
BLANK_MAP = True  # use blank map at the start
EPISODE_TIME_LIMIT = 100
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720
PIXELS_PER_UNIT = 32
LOGGING = False  # True/False switch for logging information from the simulation
COLLECT_STEERING_DATA = False  # True/False switch to get data from the encoders
COLLECT_LIDAR_DATA = True  # True/False for trying to get data from the LiDAR

# Clock
CLOCK_SPEED = 1

# SLAM
SLAM_ACTIVATED = False
SLAM_NOISE = 1e-3
SLAM_FRAME_LIMIT = 1  # "run slam every x seconds"
MATRIX_SIZE = 200

# Car
USE_RAW_DATA = False  # True
WHEEL_CIRCUMFERENCE = np.pi * 0.3
LOG_FILE_NAME = "name.csv"
STEERING_NOISE = 1e-3
CAR_X_START_POSITION = 7
CAR_Y_START_POSITION = 10
CAR_LENGTH = 1.03
CAR_WIDTH = 0.71
CHASE_LENGTH = 1.56
CHASE_WIDTH = 1.06
WHEELBASE = 1.03
SAFETY_DISTANCE = 0.1
CRITICAL_DISTANCE = max(CHASE_WIDTH, CHASE_LENGTH)/2 + SAFETY_DISTANCE
MAX_STEERING = 25
MAX_ACCELERATION = 4.0
MAX_VELOCITY = 4.0
BRAKE_DECELERATION = 4
FREE_DECELERATION = 1
CAR_FIELD_OF_VIEW = 225
CAR_FOV_RANGE = 60
TURNING_SHARPNESS = 1.8
STEERING_RAW_TO_RADIANS = MAX_STEERING / 3500  # TBD!
ENCO_COUNT_TO_METERS = (WHEEL_CIRCUMFERENCE/12, WHEEL_CIRCUMFERENCE/12, WHEEL_CIRCUMFERENCE/12, WHEEL_CIRCUMFERENCE/12) # raw encoder count to meter convesion multiplier (for each wheel)


