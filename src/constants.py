# VR detector
# Arm movement
WRIST_HOME_STATE = {
    "translation": [0, 0, 0],
    "rotation_matrix": [1, 0, 0, 0, 1, 0, 0, 0, -1],
}

# Joint Information
OCULUS_NUM_KEYPOINTS = 24
# VR_THUMB_BOUND_VERTICES = 8
VR_THUMB_BOUND_VERTICES = 4
GRIPPER_OPEN = 0
GRIPPER_CLOSE = 1

OCULUS_JOINTS = {
    "metacarpals": [2, 6, 9, 12, 15],
    "knuckles": [6, 9, 12, 16],
    "thumb": [2, 3, 4, 5, 19],
    "index": [6, 7, 8, 20],
    "middle": [9, 10, 11, 21],
    "ring": [12, 13, 14, 22],
    "pinky": [15, 16, 17, 18, 23],
}

OCULUS_VIEW_LIMITS = {
    "x_limits": [-0.04, 0.04],
    "y_limits": [-0.02, 0.25],
    "z_limits": [-0.04, 0.04],
}

RECORD_FREQ = 30
VR_FREQ = 60
BIMANUAL_VR_FREQ = 90
TRANS_FREQ = 60

# Topics
XARM_NOTIFIER_TOPIC = "XARM_NOTIFER"

# Realsense Camera parameters
NUM_CAMS = 1
CAM_FPS = 30
CAM_FPS_SIM = 60
WIDTH = 1280
HEIGHT = 720
PROCESSING_PRESET = 1  # High accuracy post-processing mode
VISUAL_RESCALE_FACTOR = 2
VIZ_PORT_OFFSET = 500
DEPTH_PORT_OFFSET = 1000


STATUS_RUNNING = "1"
STATUS_STOP = "0"
