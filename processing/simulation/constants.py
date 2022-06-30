# Race
MODEL_NAME = "PPO"
MODEL_NUMBER = "1651669206"  # disc: 1649940839/ cont: 1651669206
TIME_STEPS = 580000   # disc: 500000, cont: 580000
CONVERSION = "none"  # disc_to_cont

# Path Planning
DISTANCE_TO_MATCH = 0.15
MODE = "race"  #
# "race": one lap slam and midpoint method, then agent without SLAM
# "testing": everything else :)
STEERING_METHOD = "autonomous"  # "user"/"autonomous"
BLANK_MAP = False  # use blank map at the start
EPISODE_TIME_LIMIT = 100
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720
PIXELS_PER_UNIT = 32

# Clock
CLOCK_SPEED = 1

# SLAM
SLAM_ACTIVATED = True
SLAM_NOISE = 1e-3
SLAM_FRAME_LIMIT = 1  # "run slam every x seconds"
MATRIX_SIZE = 200

# Car
STEERING_NOISE = 1e-3
CAR_X_START_POSITION = 7
CAR_Y_START_POSITION = 10
CAR_LENGTH = 1.5
MAX_STEERING = 25
MAX_ACCELERATION = 4.0
MAX_VELOCITY = 4.0
BRAKE_DECELERATION = 4
FREE_DECELERATION = 1
CAR_FIELD_OF_VIEW = 225
CAR_FOV_RANGE = 60
TURNING_SHARPNESS = 1.8


