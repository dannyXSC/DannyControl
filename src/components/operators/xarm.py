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

from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation as R
import numpy as np
from copy import deepcopy as copy
import zmq
import time

XARM_ANCHOR_O_VALUES = [151.2, 312.3, -1, 180, 0, 90]
XARM_ANCHOR_P1_VALUES = [159.9, 620, -1, 180, 0, 90]
XARM_ANCHOR_P2_VALUES = [336.3, 309.9, -1, 180, 0, 90]


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

        self._P = [[1, 0, 0], [0, 0, 1], [0, 1, 0]]

        self.is_first_frame = True

        self.retarget_factory = AnchorBasedRetFactory(
            o=XARM_ANCHOR_O_VALUES[:3],
            p1=XARM_ANCHOR_P1_VALUES[:3],
            p2=XARM_ANCHOR_P2_VALUES[:3],
            eta=1200,
            euler_angles=XARM_ANCHOR_O_VALUES[3:],
            P=self._P,
        )

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

        while True:
            hand_frame = self._get_hand_frame_block()
            anchors_counts = self.retarget_factory.reset(hand_frame)
            # send stage infomation
            self._operation_response_socket.recv()
            self._operation_response_socket.send_string(f"{anchors_counts}")
            if self.retarget_factory.reset_completed():
                break
        # close _operation_response_socket
        self._operation_response_socket.recv()
        self._operation_response_socket.close()

        self.is_first_frame = False
        # gripper settings
        # 1 for open
        self.gripper_state = 1
        self.robot.move_gripper_percentage(1)

    # Apply retargeted angles
    def _apply_retargeted_angles(self):
        if self.is_first_frame:
            self._reset_teleop()
            return

        moving_hand_frame = self._get_hand_frame_block()

        result = self.retarget_factory.get_target(moving_hand_frame)
        final_rotation = result["rotation"]
        final_position = result["position"]
        final_position[2] = max(final_position[2], 0)
        final_pose = [*final_position] + [*final_rotation]
        if np.linalg.norm(moving_hand_frame[0]) < 1e-5:
            print(f"error {moving_hand_frame[0]}")
            return

        self.robot.move_coords(final_pose, speed=1000)
        # TODO:
        gripper_state = self.get_gripper_state()
        if gripper_state != self.gripper_state:
            self.robot.move_gripper_percentage(
                gripper_state, wait=False, wait_motion=False
            )
            self.gripper_state = gripper_state
        # publish pose + gripper astate
        self.action_publisher.pub_keypoints(
            final_pose + [gripper_state], topic_name="action"
        )

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
