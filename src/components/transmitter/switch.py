from src.components import Component
from src.utils.network import (
    ZMQKeypointSubscriber,
    ZMQStringPublisher,
    create_response_socket,
)
from src.utils.timer import FrequencyTimer, SwitchStationTimer
from src.constants import *

import numpy as np


class Switch(Component):

    def __init__(
        self,
        host,
        transformed_right_keypoints_port,
        switch_state_port,
    ):
        super().__init__()
        self.notify_component_start("switch")

        # listen for right hand info to set state infomation
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_right_keypoints_port,
            topic="transformed_hand_frame",
        )
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=transformed_right_keypoints_port,
            topic="transformed_hand_coords",
        )

        self.state_publisher = ZMQStringPublisher(host, switch_state_port)

        self.state = STATUS_STOP

        self.timer = FrequencyTimer(10)

        self.station_timer = SwitchStationTimer(2, 0.05)

    def switch_state(self):
        if self.state == STATUS_STOP:
            self.state = STATUS_RUNNING
        else:
            self.state = STATUS_STOP

    def stream(self):
        pre_result = False
        while True:
            try:
                self.timer.start_loop()

                # get right hand infomation
                transformed_hand_coords = (
                    self._transformed_hand_keypoint_subscriber.recv_keypoints()
                )
                distance = np.linalg.norm(
                    transformed_hand_coords[OCULUS_JOINTS["pinky"][-1]]
                    - transformed_hand_coords[OCULUS_JOINTS["thumb"][-1]]
                )

                result = self.station_timer.trigger(distance)
                if pre_result == False and result == True:
                    self.switch_state()
                pre_result = result

                self.state_publisher.pub_string(self.state, "state")

                self.timer.end_loop()

            except KeyboardInterrupt:
                break

        self._transformed_arm_keypoint_subscriber.stop()
        self._transformed_hand_keypoint_subscriber.stop()
        self.state_publisher.stop()
        print("Stopping the switch")
