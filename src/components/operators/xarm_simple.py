from .operator import Operator
from src.utils.network import (
    ZMQKeypointSubscriber,
    create_response_socket,
    create_request_socket,
    ZMQKeypointPublisher,
)
from src.components.robot.xarm import Xarm
from src.utils.timer import FrequencyTimer, StationTimer, LogTimer
from src.utils.vectorops import *
from src.utils.retargeter import RotationRet, PositionRet, AnchorBasedRetFactory
from src.constants import *
from src.utils.retargeter import Coordinate, RotationRetFactory
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation as R
import numpy as np
from copy import deepcopy as copy
import zmq
import time

XARM_ANCHOR_O_VALUES = [462.5,7.2,127.7,179.7,-0.3,-7.1]


class XarmOperator(Operator):

    def __init__(
        self,
        host,
        transformed_keypoints_port,
        operation_stage_port,
        action_port,
        xarm_ip,
        comp_ratio=0,
        log=False,
    ):
        self.notify_component_start("xarm operator")
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic="transformed_hand_frame"
        )
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host, port=transformed_keypoints_port, topic="transformed_hand_coords"
        )
        self._operation_response_socket = create_response_socket(
            host, operation_stage_port
        )
        self.action_publisher = ZMQKeypointPublisher(host, action_port)

        self._robot = Xarm(xarm_ip)
        # Frequency timer
        self._timer = FrequencyTimer(VR_FREQ)
        self._station_timer = StationTimer(2, 0.01)

        # self._P = [[1, 0, 0], [0, 0, 1], [0, 1, 0]]
        self._P = [[0, -1, 0], [0, 0, 1], [1, 0, 0]]

        self.is_first_frame = True
        self.scale_factor = [1000, 1000, 1000]

    @property
    def timer(self):
        return self._timer

    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber

    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber

    @property
    def robot(self):
        return self._robot

    # Get the hand frame
    def _get_hand_frame_block(self):
        return self.transformed_arm_keypoint_subscriber.recv_keypoints()

    def get_gripper_state(self):
        transformed_hand_coords = (
            self._transformed_hand_keypoint_subscriber.recv_keypoints()
        )
        distance = np.linalg.norm(
            transformed_hand_coords[OCULUS_JOINTS["index"][-1]]
            - transformed_hand_coords[OCULUS_JOINTS["thumb"][-1]]
        )
        if distance > 0.06:
            return 1
        else:
            return 0

    # Reset the teleoperation
    def _reset_teleop(self):
        print("****** RESETTING TELEOP ****** ")
        self.robot.move_coords(XARM_ANCHOR_O_VALUES)

        # wait for VR request
        self._operation_response_socket.recv()
        self._operation_response_socket.send_string(f"{0}")
        self._operation_response_socket.recv()
        self._operation_response_socket.send_string(f"{1}")
        self._operation_response_socket.recv()
        self._operation_response_socket.send_string(f"{2}")
        # close _operation_response_socket
        self._operation_response_socket.recv()
        self._operation_response_socket.close()

        self.init_hand_frame = self._get_hand_frame_block()
        o, x, y, z = self.init_hand_frame
        self.init_o = Coordinate(o, o + x, o + z, left_hand=True)

        robot_state = self.robot.get_cartesian_position()
        robot_pos = robot_state[:3]
        self.init_robot_o = Coordinate(
            robot_pos, robot_pos + np.array([0, 1, 0]), robot_pos + np.array([1, 0, 0])
        )

        rotation_mat = self.robot.get_rotation_matrix()[:3,:3]
        self.rotation_ret = RotationRetFactory(rotation_mat, self._P)
        self.rotation_ret.reset(self.init_hand_frame)

        self.is_first_frame = False
        self.gripper_state = 1
        self.robot.move_gripper_percentage(1)


    # Apply retargeted angles
    def _apply_retargeted_angles(self):
        if self.is_first_frame:
            self._reset_teleop()
            return
        moving_hand_frame = self._get_hand_frame_block()
        cur_o, cur_x, cur_y, cur_z = moving_hand_frame

        v = np.array(cur_o) - self.init_o.o
        shift = (
            self.scale_factor
            * (v @ self.init_o.get_column_matrix())
            @ self.init_robot_o.get_row_matrix()
        )
        final_position = self.init_robot_o.o + shift

        final_rotation = self.rotation_ret.get_target(moving_hand_frame)
        final_rotation = Rotation.from_matrix(final_rotation).as_euler("xyz", degrees=True)

        if np.linalg.norm(moving_hand_frame[0]) < 1e-5:
            print(f"error {moving_hand_frame[0]}")
            return
        
        final_pose = list(final_position) + list(final_rotation)
        self.robot.move_coords(final_pose, speed=1000)
        # TODO:
        gripper_state = self.get_gripper_state()
        if gripper_state != self.gripper_state:
            self.robot.move_gripper_percentage(
                gripper_state, wait=False, wait_motion=False
            )
            self.gripper_state = gripper_state

    # NOTE: This is for debugging should remove this when needed
    def stream(self):
        self.notify_component_start("{} control".format(self.robot.name))
        print("Start controlling the robot hand using the Oculus Headset.\n")

        # Assume that the initial position is considered initial after 3 seconds of the start
        while True:
            try:
                if not self.robot.if_shutdown():
                    self.timer.start_loop()

                    # Retargeting function
                    self._apply_retargeted_angles()

                    self.timer.end_loop()
            except KeyboardInterrupt:
                break

        self.anchor_socket.close()
        self.transformed_arm_keypoint_subscriber.stop()
        self.transformed_hand_keypoint_subscriber.stop()
        self.robot.stop()
        print("Stopping the teleoperator!")
