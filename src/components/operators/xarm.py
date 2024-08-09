from .operator import Operator
from src.utils.network import ZMQKeypointSubscriber
from src.components.robot.xarm import Xarm
from src.utils.timer import FrequencyTimer
from src.constants import *

from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial.transform import Rotation as R
import numpy as np
from copy import deepcopy as copy
import zmq


class XarmOperator(Operator):
    def __init__(
        self,
        host,
        transformed_keypoints_port,
        xarm_ip,
    ):
        self.notify_component_start("xarm operator")
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic="transformed_hand_frame"
        )
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic="transformed_hand_coords"
        )

        self._robot = Xarm(xarm_ip)
        # Frequency timer
        self._timer = FrequencyTimer(VR_FREQ)

        self.is_first_frame = True

    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot

    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber

    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber

    # Get the hand frame
    def _get_hand_frame(self):
        for i in range(10):
            data = self.transformed_arm_keypoint_subscriber.recv_keypoints(
                flags=zmq.NOBLOCK
            )
            if not data is None:
                break
        if data is None:
            return None
        return np.asanyarray(data).reshape(4, 3)

        # Converts a frame to a homogenous transformation matrix

    def _turn_frame_to_homo_mat(self, frame):
        t = frame[0]
        R = frame[1:]

        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = np.transpose(R)
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1

        return homo_mat

    # Reset the teleoperation
    def _reset_teleop(self):
        print("****** RESETTING TELEOP ****** ")
        self.robot_init_H = self.robot.get_rotation_matrix()

        first_hand_frame = self._get_hand_frame()
        while first_hand_frame is None:
            first_hand_frame = self._get_hand_frame()
        self.hand_init_H = self._turn_frame_to_homo_mat(first_hand_frame)
        self.is_first_frame = False

        return first_hand_frame

    # Apply retargeted angles
    def _apply_retargeted_angles(self, log=False):
        if self.is_first_frame:
            moving_hand_frame = (
                self._reset_teleop()
            )  # Should get the moving hand frame only once
        else:
            moving_hand_frame = self._get_hand_frame()

        current_robot_position = self.robot.get_cartesian_position()

        # hand current transform matrix
        self.hand_moving_H = self._turn_frame_to_homo_mat(moving_hand_frame)

        final_pose = (
            self.robot_init_H @ np.linalg.pinv(self.hand_init_H) @ self.hand_moving_H
        )

        self.robot.move_coords(final_pose)

    # NOTE: This is for debugging should remove this when needed
    def stream(self):
        self.notify_component_start("{} control".format(self.robot.name))
        print("Start controlling the robot hand using the Oculus Headset.\n")

        # Assume that the initial position is considered initial after 3 seconds of the start
        while True:
            try:
                if self.robot.get_joint_position() is not None:
                    self.timer.start_loop()

                    # Retargeting function
                    self._apply_retargeted_angles(log=False)

                    self.timer.end_loop()
            except KeyboardInterrupt:
                break

        self.transformed_arm_keypoint_subscriber.stop()
        self.transformed_hand_keypoint_subscriber.stop()
        self.robot.stop()
        print("Stopping the teleoperator!")
